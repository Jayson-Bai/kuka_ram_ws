from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"
PROJECT_SRC = Path(__file__).resolve().parents[2]
STARTUP_LAUNCH = PROJECT_SRC / "my_project_startup" / "launch" / "startup.launch.py"
CENTER_NODE = PROJECT_SRC / "control_center" / "src" / "center_node.cpp"


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
    assert "_DEFAULT_NPZ_PATH" in src
    assert 'return self._launch_params.get("npz_path") or _DEFAULT_NPZ_PATH' in src
    assert '_LAUNCH_DEFAULTS["npz_path"]' not in src


def test_formal_print_prefers_flat_npz_over_manifest():
    src = _source()

    resolver = src.split("def _resolve_npz_launch_path_from_dir", 1)[1].split(
        "def _offset_sidecar_candidates", 1
    )[0]
    assert 'root.glob("*.npz")' in resolver
    assert 'root.glob("*_part*.npz")' in resolver
    assert "manifest" not in resolver.lower()

    auto_path = src.split("    def _auto_npz_launch_path", 1)[1].split(
        "    def _current_npz_launch_path", 1
    )[0]
    assert "npz_file = os.path.join(data_root, base + \".npz\")" in auto_path
    assert "npz_part = os.path.join(data_root, base + \"_part0000.npz\")" in auto_path
    assert "_resolve_npz_launch_path_from_dir(npz_dir)" in auto_path
    assert "manifest" not in auto_path.lower()


def test_split_export_is_not_enabled_by_default_for_formal_print():
    src = _source()

    planner_params = src.split("_PLANNER_PARAMS = [", 1)[1].split("]", 1)[0]
    assert '("split_by_layer_type", "false", "按层+类型拆分 NPZ")' in planner_params
    assert '("plot_layer_xy", "true", "每层生成 XY 路径图")' in planner_params


def test_stop_paths_send_current_tool_heat_off_before_shutdown():
    src = _source()

    assert "def current_tool_id(self):" in src
    assert "def _send_current_tool_heat_off(self):" in src
    assert "self._send_current_tool_heat_off()" in src.split("    def _on_command_submit", 1)[1].split("    def _on_uart_command_submit", 1)[0]
    assert "self._send_current_tool_heat_off()" in src.split("    def _on_stop_launch", 1)[1].split("    def _check_launch_process", 1)[0]
    assert "self._send_current_tool_heat_off()" in src.split("    def shutdown_plugin", 1)[1]
    assert "EV 0 heat_cf 0\\n" in src
    assert "EV 0 heat_resin 0\\n" in src


def test_extrusion_precision_defaults_preserve_4ms_e_values():
    ui_src = _source()
    startup_src = STARTUP_LAUNCH.read_text(encoding="utf-8")
    center_src = CENTER_NODE.read_text(encoding="utf-8")

    assert '("e_decimals", "6", "挤出小数精度", "中心节点")' in ui_src
    e_decimals_section = startup_src.split("DeclareLaunchArgument(\n            \"e_decimals\"", 1)[1].split("        ),", 1)[0]
    assert "default_value=\"6\"" in e_decimals_section
    assert "int e_decimals_{6};" in center_src
    assert 'declare_parameter<int>("e_decimals", 6)' in center_src


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


def test_launch_title_changes_for_test_mode():
    src = _source()

    assert 'self._launch_box.setTitle("启动通信")' in src
    assert 'self._launch_box.setTitle("启动")' in src
    assert 'self._btn_launch = QtWidgets.QPushButton("启动")' in src


def test_latency_nozzle_lever_defaults_from_kuka_tool_data():
    ui_src = _source()
    startup_src = STARTUP_LAUNCH.read_text(encoding="utf-8")

    assert '("robot_match_nozzle_lever_mm", "401.68", "TCP 姿态误差等效臂长（mm）", "延迟监控")' in ui_src
    lever_section = startup_src.split('DeclareLaunchArgument(\n            "robot_match_nozzle_lever_mm"', 1)[1].split("        ),", 1)[0]
    assert 'default_value="401.68"' in lever_section
    assert 'TCP 姿态误差等效臂长' in lever_section
