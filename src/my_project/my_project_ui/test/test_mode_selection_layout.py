from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def _source():
    return UI_PANEL.read_text(encoding="utf-8")


def test_rqt_starts_on_mode_selection_page():
    src = _source()

    assert "_MODE_PAGE_SELECT" in src
    assert '_btn_mode_test = QtWidgets.QPushButton("测试模式")' in src
    assert '_btn_mode_print = QtWidgets.QPushButton("正式打印")' in src
    assert "self._mode_stack.setCurrentWidget(self._mode_select_page)" in src


def test_test_mode_keeps_test_controls_and_launch_controls_and_hides_print_only_sections():
    src = _source()

    assert '"test": [' in src
    assert "self._print_test_box" in src
    assert "self._export_box" in src
    assert "self._launch_box" in src
    assert "self._control_box" in src
    assert "self._latency_box" in src

    test_section = src.split('"test": [', 1)[1].split("],", 1)[0]
    assert "self._print_test_box" in test_section
    assert "self._launch_box" in test_section
    assert "self._export_box" not in test_section
    assert "self._control_box" not in test_section
    assert "self._latency_box" not in test_section


def test_test_mode_launch_can_bootstrap_without_selected_npz():
    src = _source()

    assert "def _test_mode_bootstrap_npz_path(self):" in src
    assert "if self._widget.active_mode() == _MODE_PAGE_TEST:" in src
    assert "self._do_launch(self._test_mode_bootstrap_npz_path())" in src
    assert "测试模式节点已启动" in src


def test_print_mode_excludes_print_test_controls():
    src = _source()

    print_section = src.split('"print": [', 1)[1].split("],", 1)[0]
    assert "self._export_box" in print_section
    assert "self._launch_box" in print_section
    assert "self._control_box" in print_section
    assert "self._latency_box" in print_section
    assert "self._print_test_box" not in print_section



def test_print_test_range_inputs_use_separate_fields_and_units():
    src = _source()

    assert "self._test_layer_height_min_input" in src
    assert "self._test_layer_height_max_input" in src
    assert "self._test_scale_min_input" in src
    assert "self._test_scale_max_input" in src
    assert "QtWidgets.QLabel(\"-\")" in src
    assert "mm" in src
    assert "mm/s" in src
