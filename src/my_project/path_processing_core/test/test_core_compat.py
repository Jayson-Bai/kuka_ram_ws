def test_core_exports_match_legacy_gcode_planner_imports():
    from path_processing_core.npz_exporter import export_npz as core_export_npz
    from path_processing_core.types import MoveCommand as CoreMoveCommand
    from path_processing_core.head_calibration import DEFAULT_DATA_ROOT as core_data_root

    from gcode_planner.npz_exporter import export_npz as legacy_export_npz
    from gcode_planner.types import MoveCommand as LegacyMoveCommand
    from gcode_planner.head_calibration import DEFAULT_DATA_ROOT as legacy_data_root

    assert core_export_npz is legacy_export_npz
    assert CoreMoveCommand is LegacyMoveCommand
    assert core_data_root == legacy_data_root
