from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def _source():
    return UI_PANEL.read_text(encoding="utf-8")


def test_print_test_matrix_export_keeps_extrude_wait_segments():
    src = _source()
    worker = src.split('    def _run_print_test_job', 1)[1].split('    # ---- Offset persistence ----', 1)[0]

    assert 'generate_test_matrix_gcode(' in worker
    assert 'enable_extrude_wait=True' in worker
    assert 'enable_extrude_wait=(job_type == "line")' not in worker
