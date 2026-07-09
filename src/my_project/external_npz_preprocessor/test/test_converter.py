import numpy as np
import pytest

import external_npz_preprocessor.converter as converter
from external_npz_preprocessor.converter import source_job_to_parsed_commands
from external_npz_preprocessor.process_params import (
    FiberProcessParams,
    ProcessParams,
    ResinProcessParams,
)
from external_npz_preprocessor.source_npz import LayerPaths, MaterialPath, SourceJob
from path_processing_core.types import (
    ExtrudeWait,
    GlobalCurveCommand,
    MCommand,
    MoveCommand,
    ResetECommand,
    ToolChangeCommand,
)


def _params():
    return ProcessParams(
        resin=ResinProcessParams(
            layer_height_mm=0.5,
            extrusion_scale=2.0,
            feed_mm_s=10.0,
        ),
        fiber=FiberProcessParams(extrusion_scale=0.25, feed_mm_s=6.0),
        travel_feed_mm_s=20.0,
    )


def test_converts_ordered_resin_and_fiber_paths_to_planner_commands_without_overriding_source_z():
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[
                    MaterialPath(
                        material="R",
                        order=0,
                        points=np.array(
                            [[0.0, 0.0, 2.5, 0.0, 0.0, 0.0],
                             [10.0, 0.0, 2.6, 0.0, 0.0, 0.0]],
                            dtype=np.float32,
                        ),
                    )
                ],
                fiber_paths=[
                    MaterialPath(
                        material="F",
                        order=1,
                        points=np.array(
                            [[10.0, 0.0, 3.1, 0.0, 0.0, 0.0],
                             [10.0, 4.0, 3.3, 0.0, 0.0, 0.0]],
                            dtype=np.float32,
                        ),
                    )
                ],
            )
        ],
    )

    commands = source_job_to_parsed_commands(job, _params())

    tool_changes = [cmd for cmd in commands if isinstance(cmd, ToolChangeCommand)]
    resets = [cmd for cmd in commands if isinstance(cmd, ResetECommand)]
    curves = [cmd for cmd in commands if isinstance(cmd, GlobalCurveCommand)]
    moves = [cmd for cmd in commands if isinstance(cmd, MoveCommand)]
    travel_moves = [cmd for cmd in moves if cmd.type == "TRAVEL"]
    assert [cmd.tool for cmd in tool_changes] == [1, 0]
    assert len(resets) == 2
    assert [curve.subtype for curve in curves] == ["RESIN_PRINT", "FIBER_PRINT"]
    assert [curve.cmd for curve in curves] == ["POLYLINE", "POLYLINE"]
    assert curves[0].feedrate == 600.0
    assert round(curves[0].delta_e, 6) == round((10.0 ** 2 + 0.1 ** 2) ** 0.5 * 2.0 * 0.5 * 2.0, 6)
    assert curves[1].feedrate == 360.0
    assert round(curves[1].delta_e, 6) == round((4.0 ** 2 + 0.2 ** 2) ** 0.5 * 0.25, 6)
    assert curves[0].start_pos.z == pytest.approx(2.5)
    assert curves[0].control_points[-1].z == pytest.approx(2.6)
    assert curves[1].start_pos.z == pytest.approx(3.1)
    assert curves[1].control_points[-1].z == pytest.approx(3.3)
    assert travel_moves[0].start_pos.z == pytest.approx(2.6)
    assert travel_moves[0].pos.z == pytest.approx(3.1)
    assert curves[0].layer == 0
    assert curves[1].layer == 0


def test_adds_retract_then_prime_before_and_after_every_material_path():
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[
                    MaterialPath(
                        material="R",
                        order=0,
                        points=np.array(
                            [[0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                             [10.0, 0.0, 0.5, 0.0, 0.0, 0.0]],
                            dtype=np.float32,
                        ),
                    )
                ],
                fiber_paths=[
                    MaterialPath(
                        material="F",
                        order=0,
                        points=np.array(
                            [[10.0, 0.0, 0.6, 0.0, 0.0, 0.0],
                             [20.0, 0.0, 0.6, 0.0, 0.0, 0.0]],
                            dtype=np.float32,
                        ),
                    )
                ],
            )
        ],
    )

    waits = [cmd for cmd in source_job_to_parsed_commands(job, ProcessParams()) if isinstance(cmd, ExtrudeWait)]

    assert [(cmd.delta_e, cmd.feedrate, cmd.subtype) for cmd in waits] == [
        (-15.0, 1800.0, "RESIN_PRINT"),
        (18.0, 900.0, "RESIN_PRINT"),
        (-15.0, 1800.0, "RESIN_PRINT"),
        (18.0, 900.0, "RESIN_PRINT"),
        (-10.0, 300.0, "FIBER_PRINT"),
        (12.0, 300.0, "FIBER_PRINT"),
        (-10.0, 300.0, "FIBER_PRINT"),
        (12.0, 300.0, "FIBER_PRINT"),
    ]
    assert [round(cmd.wait_sec, 6) for cmd in waits] == [
        0.5,
        1.2,
        0.5,
        1.2,
        2.0,
        2.4,
        2.0,
        2.4,
    ]


def test_inserts_cut_after_each_fiber_path_before_trailing_retract_prime():
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[],
                fiber_paths=[
                    MaterialPath(
                        material="F",
                        order=0,
                        points=np.array(
                            [[0.0, 0.0, 0.6, 0.0, 0.0, 0.0],
                             [10.0, 0.0, 0.6, 0.0, 0.0, 0.0]],
                            dtype=np.float32,
                        ),
                    ),
                    MaterialPath(
                        material="F",
                        order=1,
                        points=np.array(
                            [[20.0, 0.0, 0.6, 0.0, 0.0, 0.0],
                             [30.0, 0.0, 0.6, 0.0, 0.0, 0.0]],
                            dtype=np.float32,
                        ),
                    ),
                ],
            )
        ],
    )

    commands = source_job_to_parsed_commands(job, ProcessParams())
    compact = [
        ("print", None)
        if isinstance(cmd, GlobalCurveCommand) and cmd.type == "PRINT"
        else ("wait", cmd.delta_e)
        if isinstance(cmd, ExtrudeWait)
        else ("cut", cmd.params)
        if isinstance(cmd, MCommand) and cmd.code == "CUT"
        else None
        for cmd in commands
    ]
    compact = [item for item in compact if item is not None]

    assert compact == [
        ("wait", -10.0),
        ("wait", 12.0),
        ("print", None),
        ("cut", {"P": 1.0}),
        ("wait", -10.0),
        ("wait", 12.0),
        ("wait", -10.0),
        ("wait", 12.0),
        ("print", None),
        ("cut", {"P": 1.0}),
        ("wait", -10.0),
        ("wait", 12.0),
    ]


def test_initializes_both_heads_before_first_path_and_resets_after_tool_change():
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[
                    MaterialPath(
                        material="R",
                        order=0,
                        points=np.array(
                            [[0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                             [10.0, 0.0, 0.5, 0.0, 0.0, 0.0]],
                            dtype=np.float32,
                        ),
                    )
                ],
                fiber_paths=[
                    MaterialPath(
                        material="F",
                        order=0,
                        points=np.array(
                            [[10.0, 0.0, 0.6, 0.0, 0.0, 0.0],
                             [20.0, 0.0, 0.6, 0.0, 0.0, 0.0]],
                            dtype=np.float32,
                        ),
                    )
                ],
            )
        ],
    )

    commands = source_job_to_parsed_commands(job, ProcessParams())

    startup_events = [cmd for cmd in commands[:4] if isinstance(cmd, MCommand)]
    assert [(cmd.code, cmd.tool, cmd.params) for cmd in startup_events] == [
        ("M106", 1, {"T": 1.0}),
        ("M106", 0, {"T": 0.0}),
        ("M104", 1, {"S": 250.0, "T": 1.0}),
        ("M104", 0, {"S": 250.0, "T": 0.0}),
    ]

    command_kinds = [
        type(cmd).__name__
        for cmd in commands
        if isinstance(cmd, (MCommand, ToolChangeCommand, ResetECommand))
    ]
    assert command_kinds[:6] == [
        "MCommand",
        "MCommand",
        "MCommand",
        "MCommand",
        "ToolChangeCommand",
        "ResetECommand",
    ]

    fiber_tool_idx = next(
        idx for idx, cmd in enumerate(commands)
        if isinstance(cmd, ToolChangeCommand) and cmd.tool == 0
    )
    assert isinstance(commands[fiber_tool_idx + 1], ResetECommand)


def test_process_layer_heights_are_extrusion_references_only_not_z_generation():
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[MaterialPath("R", 0, np.array([[0.0, 0.0, 8.0, 0.0, 0.0, 0.0], [1.0, 0.0, 8.2, 0.0, 0.0, 0.0]], dtype=np.float32))],
                fiber_paths=[MaterialPath("F", 0, np.array([[0.0, 1.0, 9.0, 0.0, 0.0, 0.0], [1.0, 1.0, 9.4, 0.0, 0.0, 0.0]], dtype=np.float32))],
            ),
        ],
    )
    params = ProcessParams(
        resin=ResinProcessParams(layer_height_mm=0.01),
        fiber=FiberProcessParams(layer_height_mm=99.0),
    )

    moves = [
        cmd
        for cmd in source_job_to_parsed_commands(job, params)
        if isinstance(cmd, GlobalCurveCommand)
    ]

    assert [(cmd.subtype, pytest.approx(cmd.start_pos.z), pytest.approx(cmd.control_points[-1].z)) for cmd in moves] == [
        ("RESIN_PRINT", pytest.approx(8.0), pytest.approx(8.2)),
        ("FIBER_PRINT", pytest.approx(9.0), pytest.approx(9.4)),
    ]


def test_start_xy_offsets_source_paths_and_inserts_initial_travel_without_z_override():
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[
                    MaterialPath(
                        material="R",
                        order=0,
                        points=np.array(
                            [
                                [0.0, 0.0, 2.5, 1.0, 2.0, 3.0],
                                [10.0, 0.0, 2.6, 1.0, 2.0, 3.0],
                            ],
                            dtype=np.float32,
                        ),
                    )
                ],
                fiber_paths=[],
            )
        ],
    )
    params = ProcessParams(
        travel_feed_mm_s=20.0,
        default_a=4.0,
        default_b=5.0,
        default_c=6.0,
        start_x_mm=50.0,
        start_y_mm=60.0,
    )

    moves = [
        cmd for cmd in source_job_to_parsed_commands(job, params)
        if isinstance(cmd, MoveCommand)
    ]
    travel_moves = [cmd for cmd in moves if cmd.type == "TRAVEL"]
    print_curves = [
        cmd for cmd in source_job_to_parsed_commands(job, params)
        if isinstance(cmd, GlobalCurveCommand)
    ]

    assert len(travel_moves) == 1
    assert travel_moves[0].raw == "external_npz_start_xy_travel"
    assert travel_moves[0].start_pos.x == pytest.approx(0.0)
    assert travel_moves[0].start_pos.y == pytest.approx(0.0)
    assert travel_moves[0].start_pos.z == pytest.approx(2.5)
    assert travel_moves[0].pos.x == pytest.approx(50.0)
    assert travel_moves[0].pos.y == pytest.approx(60.0)
    assert travel_moves[0].pos.z == pytest.approx(2.5)
    assert (
        travel_moves[0].start_pos.a,
        travel_moves[0].start_pos.b,
        travel_moves[0].start_pos.c,
    ) == (4.0, 5.0, 6.0)
    assert print_curves[0].start_pos.x == pytest.approx(50.0)
    assert print_curves[0].start_pos.y == pytest.approx(60.0)
    assert print_curves[0].start_pos.z == pytest.approx(2.5)
    assert print_curves[0].control_points[-1].x == pytest.approx(60.0)
    assert print_curves[0].control_points[-1].y == pytest.approx(60.0)
    assert print_curves[0].control_points[-1].z == pytest.approx(2.6)


def test_external_npz_start_xy_places_part_lower_left_at_requested_position():
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[
                    MaterialPath(
                        material="R",
                        order=0,
                        points=np.array(
                            [
                                [-10.0, 5.0, 0.5, 0.0, 0.0, 0.0],
                                [-5.0, 5.0, 0.5, 0.0, 0.0, 0.0],
                            ],
                            dtype=np.float32,
                        ),
                    ),
                    MaterialPath(
                        material="R",
                        order=1,
                        points=np.array(
                            [
                                [-8.0, -3.0, 0.5, 0.0, 0.0, 0.0],
                                [-6.0, -3.0, 0.5, 0.0, 0.0, 0.0],
                            ],
                            dtype=np.float32,
                        ),
                    ),
                ],
                fiber_paths=[],
            )
        ],
    )

    curves = [
        cmd
        for cmd in source_job_to_parsed_commands(
            job, ProcessParams(start_x_mm=50.0, start_y_mm=60.0)
        )
        if isinstance(cmd, GlobalCurveCommand)
    ]
    points = []
    for curve in curves:
        points.append(curve.start_pos)
        points.extend(curve.control_points)

    assert min(point.x for point in points) == pytest.approx(50.0)
    assert min(point.y for point in points) == pytest.approx(60.0)
    assert curves[0].start_pos.x == pytest.approx(50.0)
    assert curves[0].start_pos.y == pytest.approx(68.0)


def test_external_npz_print_path_uses_polyline_fast_path_without_extra_bspline_fit(monkeypatch):
    class FakePlanner:
        def __init__(self):
            self.last_fit_profile = {}

        def fit_global_curve(self, moves, **kwargs):
            return GlobalCurveCommand(
                type="PRINT_FIT",
                cmd="SPLINE",
                start_pos=moves[0].start_pos,
                control_points=[move.pos for move in moves],
                e_val=moves[-1].e_val,
                delta_e=sum(move.delta_e for move in moves),
                feedrate=moves[0].feedrate,
                line=moves[0].line,
                raw="fake_fit",
                original_moves=list(moves),
            )

    monkeypatch.setattr(converter, "GlobalSplinePlanner", FakePlanner)
    monkeypatch.setattr(converter, "_curve_max_bidirectional_error_mm", lambda curve, points: 0.01, raising=False)
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[
                    MaterialPath(
                        material="R",
                        order=0,
                        points=np.array(
                            [
                                [0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                                [1.0, 0.2, 0.5, 0.0, 0.0, 0.0],
                                [2.0, 0.4, 0.5, 0.0, 0.0, 0.0],
                                [3.0, 0.5, 0.5, 0.0, 0.0, 0.0],
                                [4.0, 0.5, 0.5, 0.0, 0.0, 0.0],
                            ],
                            dtype=np.float32,
                        ),
                    )
                ],
                fiber_paths=[],
            )
        ],
    )

    curves = [
        cmd
        for cmd in source_job_to_parsed_commands(job, ProcessParams(spline_max_error_mm=0.05))
        if isinstance(cmd, GlobalCurveCommand)
    ]

    assert len(curves) == 1
    assert curves[0].cmd == "POLYLINE"
    assert curves[0].type == "PRINT"
    assert curves[0].raw == "external_npz_polyline"
    assert curves[0].subtype == "RESIN_PRINT"
    assert curves[0].layer == 0


def test_external_npz_print_path_smooths_sharp_corners_before_polyline_fast_path(monkeypatch):
    class FakePlanner:
        def __init__(self):
            self.last_fit_profile = {}

        def fit_global_curve(self, moves, **kwargs):
            return GlobalCurveCommand(
                type="PRINT_FIT",
                cmd="SPLINE",
                start_pos=moves[0].start_pos,
                control_points=[move.pos for move in moves],
                e_val=moves[-1].e_val,
                delta_e=sum(move.delta_e for move in moves),
                feedrate=moves[0].feedrate,
                line=moves[0].line,
                raw="fake_fit",
                original_moves=list(moves),
            )

    errors = iter([0.2, 0.01])
    monkeypatch.setattr(converter, "GlobalSplinePlanner", FakePlanner)
    monkeypatch.setattr(
        converter,
        "_curve_max_bidirectional_error_mm",
        lambda curve, points: next(errors),
        raising=False,
    )
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[
                    MaterialPath(
                        material="R",
                        order=0,
                        points=np.array(
                            [
                                [0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                                [1.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                                [1.0, 1.0, 0.5, 0.0, 0.0, 0.0],
                                [2.0, 1.0, 0.5, 0.0, 0.0, 0.0],
                            ],
                            dtype=np.float32,
                        ),
                    )
                ],
                fiber_paths=[],
            )
        ],
    )

    curves = [
        cmd
        for cmd in source_job_to_parsed_commands(job, ProcessParams(spline_max_error_mm=0.05))
        if isinstance(cmd, GlobalCurveCommand)
    ]

    assert len(curves) == 1
    assert curves[0].cmd == "POLYLINE"
    assert curves[0].raw == "external_npz_smoothed_polyline"
    assert curves[0].type == "PRINT"
    assert converter._polyline_max_turn_angle_deg(
        [curves[0].start_pos] + curves[0].control_points,
        min_segment_mm=0.01,
    ) < 45.0


def test_external_npz_rejects_bspline_when_sampled_turn_angle_exceeds_limit(monkeypatch):
    curve = GlobalCurveCommand(
        type="PRINT",
        cmd="SPLINE",
        start_pos=converter.Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        control_points=[
            converter.Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            converter.Position(1.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            converter.Position(2.0, 1.0, 0.0, 0.0, 0.0, 0.0),
        ],
        e_val=1.0,
        delta_e=1.0,
        feedrate=600.0,
        line=1,
        raw="fake_fit",
    )
    monkeypatch.setattr(
        converter,
        "_sample_spline_positions",
        lambda curve, points, sample_count=None: [curve.start_pos] + curve.control_points,
    )

    assert converter._curve_max_turn_angle_deg(curve, [], min_segment_mm=0.01) == pytest.approx(90.0)


def test_external_npz_polyline_fast_path_does_not_cross_separate_source_paths(monkeypatch):
    fit_starts = []

    class FakePlanner:
        def __init__(self):
            self.last_fit_profile = {}

        def fit_global_curve(self, moves, **kwargs):
            fit_starts.append((moves[0].start_pos.x, moves[0].start_pos.y))
            return GlobalCurveCommand(
                type="PRINT_FIT",
                cmd="SPLINE",
                start_pos=moves[0].start_pos,
                control_points=[move.pos for move in moves],
                e_val=moves[-1].e_val,
                delta_e=sum(move.delta_e for move in moves),
                feedrate=moves[0].feedrate,
                line=moves[0].line,
                raw="fake_fit",
                original_moves=list(moves),
            )

    monkeypatch.setattr(converter, "GlobalSplinePlanner", FakePlanner)
    monkeypatch.setattr(converter, "_curve_max_bidirectional_error_mm", lambda curve, points: 0.0, raising=False)
    job = SourceJob(
        meta={},
        layers=[
            LayerPaths(
                index=0,
                resin_paths=[
                    MaterialPath(
                        material="R",
                        order=0,
                        points=np.array(
                            [
                                [0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                                [1.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                                [1.0, 1.0, 0.5, 0.0, 0.0, 0.0],
                            ],
                            dtype=np.float32,
                        ),
                    ),
                    MaterialPath(
                        material="R",
                        order=1,
                        points=np.array(
                            [
                                [10.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                                [11.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                            ],
                            dtype=np.float32,
                        ),
                    ),
                ],
                fiber_paths=[],
            )
        ],
    )

    commands = source_job_to_parsed_commands(job, ProcessParams())
    curves = [cmd for cmd in commands if isinstance(cmd, GlobalCurveCommand)]
    travels = [cmd for cmd in commands if isinstance(cmd, MoveCommand) and cmd.type == "TRAVEL"]

    assert len(curves) == 2
    assert [curve.cmd for curve in curves] == ["POLYLINE", "POLYLINE"]
    assert fit_starts == []
    assert curves[1].start_pos.x == pytest.approx(10.0)
    assert curves[1].start_pos.y == pytest.approx(0.0)
    assert len(travels) == 1
    assert travels[0].start_pos.x == pytest.approx(1.0)
    assert travels[0].start_pos.y == pytest.approx(1.0)
    assert travels[0].pos.x == pytest.approx(10.0)
    assert travels[0].pos.y == pytest.approx(0.0)
