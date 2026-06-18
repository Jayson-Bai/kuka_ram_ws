from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def test_print_test_temperature_ready_threshold_is_five_celsius():
    src = UI_PANEL.read_text(encoding="utf-8")
    confirm_block = src.split("    def _on_print_test_confirm_height", 1)[
        1].split("    def _run_print_test_job", 1)[0]

    assert "temp_target - 5.0" in confirm_block
    assert "temp_target - 2.0" not in confirm_block
