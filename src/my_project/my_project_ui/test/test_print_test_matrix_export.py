from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def _source():
    return UI_PANEL.read_text(encoding="utf-8")


def test_print_test_matrix_export_keeps_extrude_wait_segments():
    src = _source()
    worker = src.split(
        '    def _run_print_test_job', 1)[1].split(
        '    # ---- Offset persistence ----', 1)[0]

    assert 'generate_test_matrix_gcode(' in worker
    assert 'enable_extrude_wait=True' in worker
    assert 'enable_extrude_wait=(job_type == "line")' not in worker


def test_formal_npz_export_keeps_extrude_wait_segments():
    src = _source()
    formal_export = src.split(
        "    def _on_export_npz", 1)[1].split(
        "    def _on_export_progress", 1)[0]

    assert "parse_gcode_lines(lines)" in formal_export
    assert "enable_extrude_wait=True" in formal_export


def test_formal_npz_export_passes_resin_z_print_compensation():
    src = _source()
    formal_export = src.split(
        "    def _on_export_npz", 1)[1].split(
        "    def _on_export_progress", 1)[0]

    assert (
        "resin_z_print_compensation_mm=self.current_resin_z_print_compensation()"
        in formal_export
    )


def test_npz_selection_validates_saved_resin_z_print_compensation():
    src = _source()

    assert "def current_resin_z_print_compensation(self):" in src
    assert "resin_z_print_compensation_mm" in src
    assert "NPZ 中的树脂轴 Z 打印补偿与当前界面设置不一致。" in src
    assert "NPZ 保存的树脂轴 Z 打印补偿" in src

def test_print_test_job_generation_supports_resin_fiber_and_composite_modes():
    src = UI_PANEL.read_text(encoding="utf-8")
    block = src.split("    def _run_print_test_job", 1)[1].split(
        "    # ---- Offset persistence ----", 1
    )[0]

    assert 'job_type == "resin_matrix"' in block
    assert 'job_type == "fiber_matrix"' in block
    assert 'job_type == "composite_matrix"' in block
    assert "generate_head_test_matrix_gcode" in block
    assert "generate_composite_test_matrix_gcode" in block
    assert "save_head_calibration" in block
