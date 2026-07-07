from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def _source():
    return UI_PANEL.read_text(encoding="utf-8")


def test_uart_log_matches_rsi_detail_view_pattern():
    src = _source()

    assert "class _LogDetailDialog" in src
    assert '_btn_uart_log_detail = QtWidgets.QPushButton("放大查看")' in src
    assert "self._btn_uart_log_detail.clicked.connect(self._show_uart_log_detail)" in src
    assert "self._uart_log_text.setLineWrapMode(QtWidgets.QPlainTextEdit.WidgetWidth)" in src
    assert "self._uart_log_text.setMinimumHeight(0)" in src
    assert "self._uart_log_latest_display" in src
    assert "self._uart_log_history" in src
    assert "def _show_uart_log_detail" in src
    assert 'detail_text = "\\n".join(self._uart_log_history)' in src
    assert 'payload.startswith("EWARN")' in src


def test_left_column_keeps_system_status_first_and_logs_stretch_to_shared_bottom():
    src = _source()

    dynamic_align = src.split(
        "    def _dynamic_align(self):", 1)[1].split(
        "    def _build_mode_select_page", 1)[0]
    assert "self._align_timer" in src
    assert "target_top = _top_in_content(self._uart_log_box)" in dynamic_align
    assert "target_bottom = viewport.height() - margins.bottom() - 1" in dynamic_align
    assert "self._content_scroll.viewport()" in dynamic_align
    assert "system_h = target_top - system_top - fixed_between - spacing_h" not in dynamic_align
    assert "_set_box_target_height(self._system_box" not in dynamic_align
    assert "log_h = max(_LOG_BOX_MIN_HEIGHT, target_bottom - target_top + 1)" in dynamic_align
    assert "_set_box_target_height(self._rsi_log_box, log_h)" in dynamic_align
    assert "_set_box_target_height(self._uart_log_box, log_h)" in dynamic_align
    assert "col0_layout.setAlignment(QtCore.Qt.AlignTop)" in src
    assert "col1_layout.setAlignment(QtCore.Qt.AlignTop)" in src

    left_section = src.split(
        "        system_box = QtWidgets.QGroupBox", 1)[1].split(
        "        layout.addLayout(col0_layout, 1, 0)", 1)[0]
    assert "col0_layout.addStretch(1)" not in left_section
    assert left_section.index("col0_layout.addWidget(system_box)") < left_section.index(
        "col0_layout.addWidget(kuka_box)")
    assert left_section.index("col0_layout.addWidget(kuka_box)") < left_section.index(
        "col0_layout.addWidget(traj_box)")
    assert left_section.index("col0_layout.addWidget(traj_box)") < left_section.index(
        "col0_layout.addWidget(rsi_log_box)")

    middle_section = src.split("        col1_layout = QtWidgets.QVBoxLayout()", 1)[
        1].split("        # ======== Print Control 区域", 1)[0]
    assert "col1_layout.addStretch(1)" not in middle_section
    assert middle_section.index("col1_layout.addWidget(ph_overview_box)") < middle_section.index(
        "col1_layout.addWidget(ph_tools_box)")
    assert middle_section.index("col1_layout.addWidget(ph_tools_box)") < middle_section.index(
        "col1_layout.addWidget(uart_log_box)")


def test_uart_log_export_button_is_standalone_at_right_column_bottom():
    src = _source()

    print_test_section = src.split(
        '        # ======== Print Test 区域 ========', 1)[1].split(
        '        # ======== Launch Control 区域 ========', 1)[0]
    col2_section = src.split('        # Add all boxes to col2_layout in the desired order', 1)[
        1].split('        layout.addLayout(col0_layout, 1, 0)', 1)[0]
    assert 'QtWidgets.QPushButton("导出诊断日志")' in src
    assert "self._btn_export_uart_log.clicked.connect(self._on_export_diagnostic_log)" in src
    assert 'print_test_layout.addWidget(self._btn_export_uart_log)' not in print_test_section
    assert 'col2_layout.addStretch(1)' in col2_section
    assert 'col2_layout.addWidget(self._btn_export_uart_log)' in col2_section
    assert col2_section.index('col2_layout.addStretch(1)') < col2_section.index(
        'col2_layout.addWidget(self._btn_export_uart_log)')


def test_diagnostic_log_exports_time_aligned_runtime_flags_as_jsonl():
    src = _source()

    assert "self._diagnostic_log_history = []" in src
    assert "_DIAGNOSTIC_LOG_LIMIT = 200000" in src
    assert "def _append_diagnostic(self, source, kind, detail):" in src
    assert '"time_epoch": epoch' in src
    assert '"source": str(source)' in src
    assert '"kind": str(kind)' in src
    assert "json.dumps(record, ensure_ascii=False, sort_keys=True)" in src

    assert 'out_dir = str(_DEFAULT_DATA_ROOT / "diagnostic_logs")' in src
    assert (
        'out_dir = str(_DEFAULT_DATA_ROOT / "print_test" / "tmp" / "diagnostic_logs")'
        not in src
    )
    assert "diagnostic_log_" in src
    assert ".jsonl" in src
    assert "诊断日志已导出" in src

    uart_section = src.split(
        "    def _on_uart_log(self, line_text):", 1)[1].split(
        "    def _on_export_diagnostic_log", 1)[0]
    assert 'self._append_diagnostic("uart", "raw"' in uart_section
    assert 'payload.startswith("EWARN")' in uart_section
    assert uart_section.index(
        'self._append_diagnostic("uart", "raw"'
    ) < uart_section.index('payload.startswith("EWARN")')

    update_ui_section = src.split(
        "    def _update_ui(self, msg: UiStatus):", 1)[1].split(
        "    def _on_pause", 1)[0]
    for token in (
        '"ready_for_motion": bool(msg.ready_for_motion)',
        '"traj_backlog": int(msg.traj_backlog)',
        '"event_pending": int(msg.event_pending)',
        '"printhead_status_valid": bool(msg.printhead_status_valid)',
        '"rsi_heartbeat_valid": bool(msg.rsi_heartbeat_valid)',
        '"current_event": self._event_diagnostic(msg.current_event)',
        '"current_traj": self._trajectory_diagnostic(msg.current_traj)',
    ):
        assert token in update_ui_section


def test_rsi_and_uart_log_boxes_are_dynamically_resized_while_system_status_is_fixed():
    src = _source()

    assert "_SYSTEM_STATUS_MIN_HEIGHT = 84" in src
    assert "_SYSTEM_STATUS_TARGET_HEIGHT = 145" in src
    assert "self._system_box.setFixedHeight(_SYSTEM_STATUS_TARGET_HEIGHT)" in src
    assert "_LOG_BOX_MIN_HEIGHT = 0" in src
    assert "_set_box_target_height(self._rsi_log_box, log_h)" in src
    assert "_set_box_target_height(self._uart_log_box, log_h)" in src
    assert "rsi_log_box.setFixedHeight" not in src
    assert "uart_log_box.setFixedHeight" not in src
