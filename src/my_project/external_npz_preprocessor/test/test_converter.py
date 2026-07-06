import numpy as np
import pytest

from external_npz_preprocessor.converter import source_job_to_parsed_commands
from external_npz_preprocessor.process_params import (
    FiberProcessParams,
    ProcessParams,
    ResinProcessParams,
)
from external_npz_preprocessor.source_npz import LayerPaths, MaterialPath, SourceJob
from path_processing_core.types import ExtrudeWait, MCommand, MoveCommand, ResetECommand, ToolChangeCommand


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
    moves = [cmd for cmd in commands if isinstance(cmd, MoveCommand)]
    print_moves = [cmd for cmd in moves if cmd.type == "PRINT"]
    travel_moves = [cmd for cmd in moves if cmd.type == "TRAVEL"]
    assert [cmd.tool for cmd in tool_changes] == [1, 0]
    assert len(resets) == 2
    assert [move.subtype for move in print_moves] == ["RESIN_PRINT", "FIBER_PRINT"]
    assert print_moves[0].feedrate == 600.0
    assert round(print_moves[0].delta_e, 6) == round((10.0 ** 2 + 0.1 ** 2) ** 0.5 * 2.0 * 0.5 * 2.0, 6)
    assert print_moves[1].feedrate == 360.0
    assert round(print_moves[1].delta_e, 6) == round((4.0 ** 2 + 0.2 ** 2) ** 0.5 * 0.25, 6)
    assert print_moves[0].start_pos.z == pytest.approx(2.5)
    assert print_moves[0].pos.z == pytest.approx(2.6)
    assert print_moves[1].start_pos.z == pytest.approx(3.1)
    assert print_moves[1].pos.z == pytest.approx(3.3)
    assert travel_moves[0].start_pos.z == pytest.approx(2.6)
    assert travel_moves[0].pos.z == pytest.approx(3.1)
    assert print_moves[0].layer == 0
    assert print_moves[1].layer == 0


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
        if isinstance(cmd, MoveCommand) and cmd.type == "PRINT"
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

    moves = [cmd for cmd in source_job_to_parsed_commands(job, params) if isinstance(cmd, MoveCommand) and cmd.type == "PRINT"]

    assert [(cmd.subtype, pytest.approx(cmd.start_pos.z), pytest.approx(cmd.pos.z)) for cmd in moves] == [
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
    print_moves = [cmd for cmd in moves if cmd.type == "PRINT"]

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
    assert print_moves[0].start_pos.x == pytest.approx(50.0)
    assert print_moves[0].start_pos.y == pytest.approx(60.0)
    assert print_moves[0].start_pos.z == pytest.approx(2.5)
    assert print_moves[0].pos.x == pytest.approx(60.0)
    assert print_moves[0].pos.y == pytest.approx(60.0)
    assert print_moves[0].pos.z == pytest.approx(2.6)
