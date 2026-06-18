from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def _source():
    return UI_PANEL.read_text(encoding="utf-8")


def test_rsi_log_has_detail_view_and_readable_wrapping():
    src = _source()

    assert "class _LogDetailDialog" in src
    assert "def _format_rsi_xml_for_display" in src
    assert '_btn_rsi_log_detail = QtWidgets.QPushButton("放大查看")' in src
    assert "self._btn_rsi_log_detail.clicked.connect(self._show_rsi_log_detail)" in src
    assert "self._rsi_log_text.setLineWrapMode(QtWidgets.QPlainTextEdit.WidgetWidth)" in src


def test_rsi_log_height_is_a_bounded_micro_adjustment():
    src = _source()

    assert "self._rsi_log_text.setMinimumHeight(0)" in src
    assert "sys_val.setMinimumHeight(40)" in src
    assert "_SYSTEM_STATUS_MIN_HEIGHT = 84" in src
    assert "_SYSTEM_STATUS_TARGET_HEIGHT = 145" in src
    assert "self._system_box.setFixedHeight(_SYSTEM_STATUS_TARGET_HEIGHT)" in src
    assert "self._rsi_log_box" in src


def test_rsi_xml_display_keeps_latest_formatted_text_for_dialog():
    src = _source()

    assert "self._rsi_log_latest_display = """ in src
    assert "display_text = _format_rsi_xml_for_display(xml_text)" in src
    assert "self._rsi_log_latest_display = display_text" in src
    assert '_LogDetailDialog("RSI XML 放大查看", self._rsi_log_latest_display' in src
