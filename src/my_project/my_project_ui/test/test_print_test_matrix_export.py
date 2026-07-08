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


def test_formal_npz_export_exposes_and_passes_cut_lift_parameters():
    src = _source()
    formal_export = src.split(
        "    def _on_export_npz", 1)[1].split(
        "    def _on_export_progress", 1)[0]

    assert '("cut_lift_mm", "20.0", "剪切抬升距离（mm）")' in src
    assert '("cut_wait_s", "15.0", "剪切等待时间（s）")' in src
    assert '"cut_lift_mm": float(self._planner_inputs["cut_lift_mm"].text())' in formal_export
    assert '"cut_wait_s": float(self._planner_inputs["cut_wait_s"].text())' in formal_export
    assert 'cut_lift_mm=params["cut_lift_mm"]' in formal_export
    assert 'cut_wait_s=params["cut_wait_s"]' in formal_export


def test_npz_selection_validates_saved_resin_z_print_compensation():
    src = _source()

    assert "def current_resin_z_print_compensation(self):" in src
    assert "resin_z_print_compensation_mm" in src
    assert "NPZ 中的树脂 Z 打印补偿与当前界面设置不一致。" in src
    assert "NPZ 保存的树脂 Z 打印补偿" in src


def test_print_test_job_generation_supports_resin_fiber_and_composite_modes():
    src = UI_PANEL.read_text(encoding="utf-8")
    block = src.split("    def _run_print_test_job", 1)[1].split(
        "    # ---- Offset persistence ----", 1
    )[0]
    anchors = (
        'job_type == "resin_matrix"',
        'job_type == "fiber_matrix"',
        'job_type == "composite_matrix"',
    )

    for anchor in anchors:
        assert anchor in block

    def branch_for(anchor):
        start = block.index(anchor)
        next_starts = [
            block.index(other) for other in anchors if block.index(other) > start
        ]
        end = min(next_starts) if next_starts else len(block)
        return block[start:end]

    resin_branch = branch_for('job_type == "resin_matrix"')
    fiber_branch = branch_for('job_type == "fiber_matrix"')
    composite_branch = branch_for('job_type == "composite_matrix"')

    assert "generate_head_test_matrix_gcode" in resin_branch
    assert 'tool="resin"' in resin_branch
    assert "generate_head_test_matrix_gcode" in fiber_branch
    assert 'tool="fiber"' in fiber_branch
    assert "generate_composite_test_matrix_gcode" in composite_branch
    assert "save_head_calibration" in block


def test_print_test_fiber_exports_start_from_fiber_tool():
    src = _source()
    worker = src.split(
        '    def _run_print_test_job', 1)[1].split(
        '    # ---- Offset persistence ----', 1)[0]

    assert 'initial_tool_id=(' in worker
    assert 'if job_type in ("fiber_matrix", "composite_matrix")' in worker
    assert '_PRINT_TEST_FIBER_TOOL_ID' in worker
    assert '_PRINT_TEST_RESIN_TOOL_ID' in worker


def test_print_test_fiber_z_offset_input_requires_positive_value():
    src = _source()
    build_section = src.split(
        "self._test_fiber_z_comp_input = QtWidgets.QLineEdit", 1
    )[1].split("self._btn_test_prepare", 1)[0]

    fiber_z_validator = build_section.split(
        "self._test_fiber_z_comp_input.setMaximumWidth(72)", 1
    )[1].split("self._test_fiber_z_comp_input.setValidator", 1)[0]

    assert "QtGui.QDoubleValidator(" in fiber_z_validator
    assert "0.001, 1000.0, 3" in fiber_z_validator
    assert "self._test_fiber_z_comp_input" in fiber_z_validator


def test_print_test_npz_load_checks_resin_z_floor_before_emit():
    src = _source()
    confirm_resin = src.split(
        "    def _on_print_test_confirm_resin_height", 1
    )[1].split("    def _on_print_test_continue_fiber", 1)[0]
    worker = src.split(
        '    def _run_print_test_job', 1)[1].split(
        '    # ---- Offset persistence ----', 1)[0]

    assert (
        "self._print_test_resin_z_floor = "
        "float(self._print_test_current_correction[2])"
    ) in confirm_resin
    assert "def _validate_print_test_npz_z_floor(self, npz_path):" in src
    assert "np.load(str(part_path))" in src
    assert "min_z < floor - 1e-6" in src
    assert "低于已确认树脂高度警戒线" in src
    publisher = src.split(
        "    def _on_print_test_load_npz", 1
    )[1].split("    def _auto_npz_launch_path", 1)[0]

    assert "if not self._validate_print_test_npz_z_floor(npz_path):" in worker
    assert "self.print_test_load_npz_submit.emit(npz_path)" in worker
    assert "if not self._widget._validate_print_test_npz_z_floor(path):" in publisher
    assert "self._print_test_load_pub.publish(msg)" in publisher
