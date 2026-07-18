from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def test_print_test_does_not_wait_for_target_temperature_but_keeps_tool_and_fan_checks():
    src = UI_PANEL.read_text(encoding="utf-8")
    confirm_block = src.split("    def _on_print_test_confirm_height", 1)[1].split(
        "    def _on_print_test_confirm_resin_height", 1
    )[0]
    ready_block = src.split("    def _print_test_head_ready", 1)[1].split(
        "    def _print_test_matrix_target", 1
    )[0]

    assert "_PRINT_TEST_TEMP_TOLERANCE_C" not in src
    assert "current_temp" not in confirm_block
    assert "target_temp" not in ready_block
    assert "_PRINT_TEST_TEMP_TOLERANCE_C" not in ready_block
    assert "current_tool" in ready_block
    assert "fan_ok_resin" in ready_block
    assert "fan_ok_cf" in ready_block
