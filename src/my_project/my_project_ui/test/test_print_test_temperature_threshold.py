from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def test_print_test_temperature_ready_threshold_is_twenty_celsius():
    src = UI_PANEL.read_text(encoding="utf-8")
    confirm_block = src.split("    def _on_print_test_confirm_height", 1)[
        1].split("    def _on_print_test_confirm_resin_height", 1)[0]

    assert "_PRINT_TEST_TEMP_TOLERANCE_C = 20.0" in src
    assert "abs(current_temp - temp_target) > _PRINT_TEST_TEMP_TOLERANCE_C" in confirm_block
    assert "temp_target - 5.0" not in src
