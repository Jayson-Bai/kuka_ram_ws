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
    assert "def _show_uart_log_detail" in src


def test_left_column_keeps_system_status_first_and_logs_stretch_to_shared_bottom():
    src = _source()

    dynamic_align = src.split("    def _dynamic_align(self):", 1)[1].split("    def _build_mode_select_page", 1)[0]
    assert "self._align_timer" in src
    assert "target_top = _top_in_content(self._uart_log_box)" in dynamic_align
    assert "target_bottom = self._mode_content_page.height() - margins.bottom() - 1" in dynamic_align
    assert "system_h = target_top - system_top - fixed_between - spacing_h" not in dynamic_align
    assert "_set_box_target_height(self._system_box" not in dynamic_align
    assert "log_h = max(_LOG_BOX_MIN_HEIGHT, target_bottom - target_top + 1)" in dynamic_align
    assert "_set_box_target_height(self._rsi_log_box, log_h)" in dynamic_align
    assert "_set_box_target_height(self._uart_log_box, log_h)" in dynamic_align
    assert "col0_layout.setAlignment(QtCore.Qt.AlignTop)" in src
    assert "col1_layout.setAlignment(QtCore.Qt.AlignTop)" in src

    left_section = src.split("        system_box = QtWidgets.QGroupBox", 1)[1].split("        layout.addLayout(col0_layout, 1, 0)", 1)[0]
    assert "col0_layout.addStretch(1)" not in left_section
    assert left_section.index("col0_layout.addWidget(system_box)") < left_section.index("col0_layout.addWidget(kuka_box)")
    assert left_section.index("col0_layout.addWidget(kuka_box)") < left_section.index("col0_layout.addWidget(traj_box)")
    assert left_section.index("col0_layout.addWidget(traj_box)") < left_section.index("col0_layout.addWidget(rsi_log_box)")

    middle_section = src.split("        col1_layout = QtWidgets.QVBoxLayout()", 1)[1].split("        # ======== Print Control 区域", 1)[0]
    assert "col1_layout.addStretch(1)" not in middle_section
    assert middle_section.index("col1_layout.addWidget(ph_overview_box)") < middle_section.index("col1_layout.addWidget(ph_tools_box)")
    assert middle_section.index("col1_layout.addWidget(ph_tools_box)") < middle_section.index("col1_layout.addWidget(uart_log_box)")


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
