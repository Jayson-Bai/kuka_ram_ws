import ast
from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"
PROJECT_SRC = Path(__file__).resolve().parents[2]
STARTUP_LAUNCH = PROJECT_SRC / "my_project_startup" / "launch" / "startup.launch.py"
CENTER_NODE = PROJECT_SRC / "control_center" / "src" / "center_node.cpp"
SYSTEM_MANAGER = PROJECT_SRC / "control_center" / "src" / "system_manager_node.cpp"
QUEUE_MANAGER = PROJECT_SRC / "control_center" / "src" / "queue_manager.cpp"
NPZ_LOADER_HPP = PROJECT_SRC / "control_center" / "include" / "control_center" / "npz_loader.hpp"
NPZ_LOADER_CPP = PROJECT_SRC / "control_center" / "src" / "npz_loader.cpp"
TRAJECTORY_MSG = PROJECT_SRC / "my_project_interfaces" / "msg" / "TrajectoryPoint.msg"
NPZ_EXPORTER = PROJECT_SRC / "gcode_planner" / "gcode_planner" / "npz_exporter.py"


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


def test_print_test_mode_uses_requested_resin_and_fiber_defaults():
    src = _source()
    init_section = src.split("        self._test_temp_input =", 1)[1].split(
        "        range_inputs = (", 1
    )[0]

    for expected in (
        'self._test_layer_height_min_input = QtWidgets.QLineEdit("0.5")',
        'self._test_layer_height_max_input = QtWidgets.QLineEdit("0.5")',
        'self._test_scale_min_input = QtWidgets.QLineEdit("1.0")',
        'self._test_scale_max_input = QtWidgets.QLineEdit("1.0")',
        'self._test_prime_length_input = QtWidgets.QLineEdit("18.0")',
        'self._test_prime_speed_input = QtWidgets.QLineEdit("15.0")',
        'self._test_retract_length_input = QtWidgets.QLineEdit("15.0")',
        'self._test_retract_speed_input = QtWidgets.QLineEdit("30.0")',
        'self._test_fiber_layer_height_min_input = QtWidgets.QLineEdit("0.05")',
        'self._test_fiber_layer_height_max_input = QtWidgets.QLineEdit("0.05")',
        'self._test_fiber_scale_min_input = QtWidgets.QLineEdit("1.0")',
        'self._test_fiber_scale_max_input = QtWidgets.QLineEdit("1.0")',
        'self._test_fiber_prime_length_input = QtWidgets.QLineEdit("12.0")',
        'self._test_fiber_prime_speed_input = QtWidgets.QLineEdit("5.0")',
        'self._test_fiber_retract_length_input = QtWidgets.QLineEdit("10.0")',
        'self._test_fiber_retract_speed_input = QtWidgets.QLineEdit("5.0")',
    ):
        assert expected in init_section


def test_formal_print_file_dialogs_start_in_data_dirs_and_npz_selects_file():
    src = _source()

    assert '"/home/jayson/kuka_ram_ws/data' not in src
    assert '"~/kuka_ram_ws/data' not in src
    assert 'os.path.expanduser("~/kuka_ram_ws' not in src
    assert '_DEFAULT_DATA_ROOT = DEFAULT_DATA_ROOT' in src
    assert '_DEFAULT_GCODE_INPUT_DIR = str(_DEFAULT_DATA_ROOT / "input_gcode")' in src
    assert '_DEFAULT_NPZ_OUTPUT_DIR = str(_DEFAULT_DATA_ROOT / "output_npz")' in src
    assert 'base_dir = str(_DEFAULT_DATA_ROOT / "print_test" / "tmp")' in src
    assert 'out_dir = str(_DEFAULT_DATA_ROOT / "print_test" / "tmp" / "diagnostic_logs")' in src
    assert 'def _ensure_default_data_dirs():' in src
    assert '_ensure_default_data_dirs()' in src.split('self._build_ui()', 1)[0]
    assert '_DEFAULT_DATA_ROOT / "input_gcode"' in src
    assert '_DEFAULT_DATA_ROOT / "output_npz"' in src
    assert '_DEFAULT_DATA_ROOT / "print_test" / "tmp"' in src

    browse_gcode = src.split("    def _on_browse_gcode", 1)[1].split(
        "    def _on_gcode_path_changed", 1
    )[0]
    assert "_DEFAULT_GCODE_INPUT_DIR" in browse_gcode
    assert "QtWidgets.QFileDialog.getOpenFileName" in browse_gcode

    select_npz = src.split("    def _on_select_npz", 1)[1].split(
        "    def _on_clear_npz", 1
    )[0]
    assert "QtWidgets.QFileDialog.getOpenFileName" in select_npz
    assert "getExistingDirectory" not in select_npz
    assert "_DEFAULT_NPZ_OUTPUT_DIR" in select_npz
    assert "_normalize_npz_launch_path" in select_npz


def test_formal_print_layer_progress_uses_low_priority_ui_status_path():
    ui_src = _source()
    traj_msg = TRAJECTORY_MSG.read_text(encoding="utf-8")
    startup_src = STARTUP_LAUNCH.read_text(encoding="utf-8")
    system_src = SYSTEM_MANAGER.read_text(encoding="utf-8")
    queue_src = QUEUE_MANAGER.read_text(encoding="utf-8")
    loader_hpp = NPZ_LOADER_HPP.read_text(encoding="utf-8")
    loader_cpp = NPZ_LOADER_CPP.read_text(encoding="utf-8")
    exporter_src = NPZ_EXPORTER.read_text(encoding="utf-8")

    assert "uint32 layer_index" in traj_msg
    assert "uint32 total_layers" in traj_msg
    assert "layer_index" in loader_hpp
    assert "total_layers" in loader_hpp
    assert 'npz.count("layer_index")' in loader_cpp
    assert 'npz.count("total_layers")' in loader_cpp
    assert "tp.layer_index = row.layer_index" in queue_src
    assert "tp.total_layers = row.total_layers" in queue_src
    assert "layer_index=layer_index" in exporter_src
    assert "total_layers=total_layers_arr" in exporter_src

    assert '("ui_publish_period_ms", "100", "UI 状态发布周期（ms）", "系统管理器")' in ui_src
    ui_publish_section = startup_src.split(
        'DeclareLaunchArgument(\n            "ui_publish_period_ms"', 1
    )[1].split("        ),", 1)[0]
    assert 'default_value="100"' in ui_publish_section
    assert 'declare_parameter<int>("ui_publish_period_ms", 100)' in system_src

    assert "self._print_progress_bar" in ui_src
    assert "self._print_progress_label" in ui_src
    assert "def _update_print_progress" in ui_src
    assert "msg.current_traj.layer_index" in ui_src
    assert "msg.current_traj.total_layers" in ui_src
    assert "self._print_progress_widget.setVisible(mode == _MODE_PAGE_PRINT)" in ui_src


def test_offset_and_resin_z_spinboxes_ignore_mouse_wheel():
    src = _source()

    assert "class _NoWheelDoubleSpinBox(QtWidgets.QDoubleSpinBox):" in src
    no_wheel = src.split(
        "class _NoWheelDoubleSpinBox(QtWidgets.QDoubleSpinBox):", 1
    )[1].split("class _PanelDialog", 1)[0]
    assert "def wheelEvent(self, event):" in no_wheel
    assert "event.ignore()" in no_wheel

    resin_z_section = src.split("self._resin_z_print_comp_spin =", 1)[1].split(
        "self._resin_z_print_comp_spin.valueChanged", 1
    )[0]
    assert "_NoWheelDoubleSpinBox()" in resin_z_section

    offset_section = src.split("self._offset_spins = {}", 1)[1].split(
        "export_layout.addLayout(offset_grid)", 1
    )[0]
    assert "spin = _NoWheelDoubleSpinBox()" in offset_section
    assert "spin = QtWidgets.QDoubleSpinBox()" not in offset_section


def test_formal_print_offsets_show_resin_z_fiber_z_and_fiber_xy_only():
    src = _source()
    export_section = src.split("# ======== GCode Export 区域 ========", 1)[1].split(
        "# ======== Print Test 区域 ========", 1
    )[0]

    assert "树脂 Z 打印补偿 / 纤维 Z 偏置" in export_section
    assert "树脂 Z" in export_section
    assert "纤维 Z 偏置" in export_section
    assert "纤维头 XY 偏置" in export_section
    assert "self._fiber_z_print_comp_spin" in export_section
    assert 'for axis, default_val in [("X",' in export_section
    assert '("Z", offset_cfg["tool_offset_z"])' not in export_section


def test_formal_print_offsets_load_all_values_from_head_calibration_when_available():
    src = _source()
    export_section = src.split("# ======== GCode Export 区域 ========", 1)[1].split(
        "# ======== Print Test 区域 ========", 1
    )[0]

    assert "if DEFAULT_HEAD_CALIBRATION_PATH.is_file():" in export_section
    assert "resin_z_default = float(" in export_section
    assert "head_calibration.resin_z_print_compensation_mm" in export_section
    assert "fiber_x_default = float(" in export_section
    assert "head_calibration.fiber_x_print_compensation_mm" in export_section
    assert "fiber_y_default = float(" in export_section
    assert "head_calibration.fiber_y_print_compensation_mm" in export_section
    assert "fiber_z_default = float(" in export_section
    assert "head_calibration.fiber_z_print_compensation_mm" in export_section
    assert "self._resin_z_print_comp_spin.setValue(resin_z_default)" in export_section
    assert 'for axis, default_val in [("X", fiber_x_default),' in export_section
    assert '("Y", fiber_y_default)' in export_section


def test_formal_print_offsets_fallback_to_legacy_when_head_calibration_unreadable():
    src = _source()
    export_section = src.split("# ======== GCode Export 区域 ========", 1)[1].split(
        "# ======== Print Test 区域 ========", 1
    )[0]

    assert "json.loads(" in export_section
    assert 'DEFAULT_HEAD_CALIBRATION_PATH.read_text(encoding="utf-8")' in export_section
    fallback = export_section.split("except Exception:", 1)[1].split(
        "resin_z_row = QtWidgets.QHBoxLayout()", 1
    )[0]
    assert 'resin_z_default = offset_cfg["resin_z_print_compensation_mm"]' in fallback
    assert 'fiber_x_default = offset_cfg["tool_offset_x"]' in fallback
    assert 'fiber_y_default = offset_cfg["tool_offset_y"]' in fallback
    assert 'fiber_z_default = offset_cfg["tool_offset_z"]' in fallback


def test_split_export_is_not_enabled_by_default_for_formal_print():
    src = _source()

    planner_params = src.split("_PLANNER_PARAMS = [", 1)[1].split("]", 1)[0]
    assert '("split_by_layer_type", "false", "按层+类型拆分 NPZ")' in planner_params
    assert '("plot_layer_xy", "true", "每层生成 XY 路径图")' in planner_params


def test_stop_paths_send_current_tool_heat_off_before_shutdown():
    src = _source()

    assert "def current_tool_id(self):" in src
    assert "def _send_current_tool_heat_off(self):" in src
    assert "self._send_current_tool_heat_off()" in src.split(
        "    def _on_command_submit", 1)[1].split(
        "    def _on_uart_command_submit", 1)[0]
    assert "self._send_current_tool_heat_off()" in src.split(
        "    def _on_stop_launch", 1)[1].split(
        "    def _check_launch_process", 1)[0]
    assert "self._send_current_tool_heat_off()" in src.split("    def shutdown_plugin", 1)[1]
    assert "EV 0 heat_cf 0\\n" in src
    assert "EV 0 heat_resin 0\\n" in src


def test_extrusion_precision_defaults_preserve_4ms_e_values():
    ui_src = _source()
    startup_src = STARTUP_LAUNCH.read_text(encoding="utf-8")
    center_src = CENTER_NODE.read_text(encoding="utf-8")

    assert '("e_decimals", "6", "挤出小数精度", "中心节点")' in ui_src
    e_decimals_section = startup_src.split(
        "DeclareLaunchArgument(\n            \"e_decimals\"", 1)[1].split(
        "        ),", 1)[0]
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
    lever_section = startup_src.split(
        'DeclareLaunchArgument(\n            "robot_match_nozzle_lever_mm"',
        1)[1].split(
        "        ),",
        1)[0]
    assert 'default_value="401.68"' in lever_section
    assert 'TCP 姿态误差等效臂长' in lever_section


def test_test_mode_exposes_fiber_calibration_and_print_actions():
    src = _source()
    test_section = src.split("# ======== Print Test 区域 ========", 1)[1].split(
        "# ======== Launch Control 区域 ========", 1
    )[0]

    for text in (
        "打印测试",
        "全局测试参数",
        "树脂头参数",
        "纤维头参数",
        "进入树脂测试准备",
        "树脂头动作",
        "纤维头动作",
        "切换到纤维头",
        "Z +0.1",
        "Z -0.1",
        "确认打印高度",
        "开始树脂测试",
        "X 偏置",
        "Y 偏置",
        "Z 补偿",
        "锁定偏置输入",
        "确认偏置（叠加树脂Z并下发）",
        "确认偏置",
        "直接打印纤维",
        "剪切",
        "复合打印",
    ):
        assert text in test_section

    assert "打印测试（树脂）" not in test_section
    assert "全局动作" not in test_section
    assert "纤维头：继续调整" not in test_section
    assert "树脂头：" not in test_section
    assert "纤维头：" not in test_section

    for attr in (
        "_test_resin_z_comp_input",
        "_test_fiber_x_comp_input",
        "_test_fiber_y_comp_input",
        "_test_fiber_z_comp_input",
        "_test_fiber_temp_input",
        "_test_line_length_input",
        "_test_y_spacing_input",
        "_test_tool_change_safe_lift_input",
    ):
        assert attr in test_section

    for connection in (
        "self._btn_test_confirm_resin_height.clicked.connect("
        "self._on_print_test_confirm_resin_height)",
        "self._btn_test_continue_fiber.clicked.connect("
        "self._on_print_test_continue_fiber)",
        "self._btn_test_print_resin.clicked.connect(self._on_print_test_print_resin)",
        "self._btn_test_apply_fiber_offset.clicked.connect("
        "self._on_print_test_apply_fiber_offset)",
        "self._btn_test_confirm_fiber_offset.clicked.connect("
        "self._on_print_test_confirm_fiber_offset)",
        "self._btn_test_print_fiber.clicked.connect(self._on_print_test_print_fiber)",
        "self._btn_test_print_composite.clicked.connect("
        "self._on_print_test_print_composite)",
        "self._btn_test_cut.clicked.connect(self._on_print_test_cut)",
    ):
        assert connection in test_section



def test_print_test_prepare_switches_to_resin_and_starts_both_heat_fans():
    src = _source()
    prepare = src.split("    def _on_print_test_prepare", 1)[1].split(
        "    def _set_print_test_controls_enabled", 1
    )[0]

    assert 'self.uart_command_submit.emit("EV 0 tool_change_resin 2' in prepare
    assert 'self.uart_command_submit.emit("EV 0 fan_resin 1' in prepare
    assert 'self.uart_command_submit.emit("EV 0 fan_cf 1' in prepare
    assert "self.uart_command_submit.emit(f\"EV 0 heat_resin {params['resin']['temp']}\\n\")" in prepare
    assert "self.uart_command_submit.emit(f\"EV 0 heat_cf {params['fiber']['temp']}\\n\")" in prepare


def test_print_test_resin_print_always_prepares_head_before_job():
    src = _source()

    assert '_PRINT_TEST_FIBER_TOOL_ID = 1' in src
    assert '_PRINT_TEST_RESIN_TOOL_ID = 2' in src
    assert '_PRINT_TEST_TEMP_TOLERANCE_C = 20.0' in src

    print_resin = src.split("    def _on_print_test_print_resin", 1)[1].split(
        "    def _start_print_test_resin_matrix", 1
    )[0]
    assert "self._ensure_resin_tool_then_start_print_test_resin()" in print_resin
    assert "self._run_print_test_job" not in print_resin

    ensure_resin = src.split(
        "    def _ensure_resin_tool_then_start_print_test_resin", 1
    )[1].split("    def _request_print_test_resin_tool", 1)[0]
    assert "self._request_print_test_resin_tool()" in ensure_resin
    assert "current_tool = self.current_tool_id()" not in ensure_resin
    assert "self._start_print_test_resin_matrix()" not in ensure_resin

    request_resin = src.split("    def _request_print_test_resin_tool", 1)[1].split(
        "    def _on_print_test_apply_fiber_offset", 1
    )[0]
    assert 'self._print_test_waiting_for_tool = _PRINT_TEST_RESIN_TOOL_ID' in request_resin
    assert 'self._print_test_pending_after_tool_change = "print_resin_matrix"' in request_resin
    assert 'self._send_print_test_head_prepare("resin")' in request_resin

    helper = src.split("    def _send_print_test_head_prepare", 1)[1].split(
        "    def _print_test_head_ready", 1
    )[0]
    assert 'self.uart_command_submit.emit("EV 0 fan_resin 1' in helper
    assert "heat_resin" not in helper
    assert 'self.uart_command_submit.emit("EV 0 tool_change_resin 2' in helper

    update_ui = src.split("    def _update_ui(self, msg: UiStatus):", 1)[1].split(
        "        else:\n            self._current_tool_id = 0", 1
    )[0]
    assert "self._print_test_head_ready(" in update_ui
    assert 'pending_after_tool_change == "print_resin_matrix"' in update_ui
    assert "self._start_print_test_resin_matrix()" in update_ui


def test_print_test_fiber_actions_switch_to_fiber_tool_from_safe_position_before_job():
    src = _source()

    controls = src.split("    def _set_print_test_controls_enabled", 1)[1].split(
        "    def _on_current_correction", 1
    )[0]
    assert "fiber_action_ready = base_ready and self._print_test_fiber_confirmed" in controls
    assert "self._btn_test_print_fiber.setEnabled(fiber_action_ready)" in controls
    assert "self._btn_test_print_composite.setEnabled(fiber_action_ready)" in controls

    current_correction = src.split("    def _on_current_correction", 1)[1].split(
        "    def _on_print_test_z", 1
    )[0]
    assert 'self._print_test_pending_after_zero == "tool_change_cf"' in current_correction
    assert 'self._send_print_test_head_prepare("fiber")' in current_correction
    assert "if self._print_test_pending_after_tool_change is None:" in current_correction
    assert 'self._print_test_pending_after_tool_change = "adjust_fiber_offset"' in current_correction

    print_fiber = src.split("    def _on_print_test_print_fiber", 1)[1].split(
        "    def _start_print_test_fiber_matrix", 1
    )[0]
    assert "self._ensure_fiber_tool_then_continue(" in print_fiber
    assert '"print_fiber_matrix"' in print_fiber
    assert "self._run_print_test_job" not in print_fiber

    print_composite = src.split("    def _on_print_test_print_composite", 1)[1].split(
        "    def _start_print_test_composite_matrix", 1
    )[0]
    assert "self._ensure_fiber_tool_then_continue(" in print_composite
    assert '"print_composite_matrix"' in print_composite
    assert "self._run_print_test_job" not in print_composite

    ensure_fiber = src.split("    def _ensure_fiber_tool_then_continue", 1)[1].split(
        "    def _request_print_test_fiber_tool", 1
    )[0]
    assert "current_tool = self.current_tool_id()" in ensure_fiber
    assert "if current_tool == _PRINT_TEST_FIBER_TOOL_ID:" in ensure_fiber
    assert "self._request_print_test_fiber_tool(pending_after_tool_change)" in ensure_fiber
    assert "continuation()" not in ensure_fiber

    request_fiber = src.split(
        "    def _request_print_test_fiber_tool_from_safe_position", 1
    )[1].split("    def _print_test_matrix_target", 1)[0]
    assert 'self._print_test_pending_after_zero = "tool_change_cf"' in request_fiber
    assert "self._run_print_test_job(" in request_fiber
    assert "target_pose=_PRINT_TEST_ZERO_CORRECTION" in request_fiber

    helper = src.split("    def _send_print_test_head_prepare", 1)[1].split(
        "    def _print_test_head_ready", 1
    )[0]
    assert 'self.uart_command_submit.emit("EV 0 fan_cf 1' in helper
    assert "heat_cf" not in helper
    assert 'self.uart_command_submit.emit("EV 0 tool_change_cf 1' in helper

    update_ui = src.split("    def _update_ui(self, msg: UiStatus):", 1)[1].split(
        "        else:\n            self._current_tool_id = 0", 1
    )[0]
    assert "self._print_test_head_ready(" in update_ui
    assert 'pending_after_tool_change == "print_fiber_matrix"' in update_ui
    assert "self._start_print_test_fiber_matrix()" in update_ui
    assert 'pending_after_tool_change == "print_composite_matrix"' in update_ui
    assert "self._start_print_test_composite_matrix()" in update_ui


def test_print_test_resin_target_accounts_for_final_same_height_y_shift():
    src = _source()
    target_section = src.split("    def _print_test_matrix_target", 1)[1].split(
        "    def _on_print_test_cut", 1
    )[0]

    assert 'final_y_steps = line_count if head_key == "resin" else max(0, line_count - 1)' in target_section
    assert "base[1] + final_y_steps * y_spacing" in target_section


def test_print_test_composite_target_uses_resin_surface_height_not_head_offset():
    src = _source()
    target_section = src.split("    def _print_test_matrix_target", 1)[1].split(
        "    def _on_print_test_cut", 1
    )[0]

    assert 'resin_surface_height = float(resin_layers[0]) if resin_layers else 0.0' in target_section
    assert 'calibration_relative_offsets(' not in target_section
    assert "base[2] + resin_surface_height + last_layer_height + safe_lift" in target_section

def test_fiber_offset_apply_only_locks_inputs_for_micro_nudging():
    src = _source()
    apply_section = src.split("    def _on_print_test_apply_fiber_offset", 1)[1].split(
        "    def _on_print_test_nudge_fiber_offset", 1
    )[0]

    assert "self._print_test_fiber_offset_locked = True" in apply_section
    assert "save_head_calibration" not in apply_section
    assert "self._run_print_test_job" not in apply_section


def test_fiber_offset_micro_nudges_only_change_inputs_until_confirm():
    src = _source()

    assert "self._print_test_fiber_offset_locked = False" in src
    controls = src.split("    def _set_print_test_controls_enabled", 1)[1].split(
        "    def _on_current_correction", 1
    )[0]
    assert "fiber_input_ready = fiber_ready and not self._print_test_fiber_offset_locked" in controls
    assert "inp.setEnabled(fiber_input_ready)" in controls
    assert "self._btn_test_apply_fiber_offset.setEnabled(fiber_input_ready)" in controls
    assert "fiber_nudge_ready = fiber_ready and self._print_test_fiber_offset_locked" in controls
    assert "btn.setEnabled(fiber_nudge_ready)" in controls

    nudge_section = src.split("    def _on_print_test_nudge_fiber_offset", 1)[1].split(
        "    def _on_print_test_confirm_fiber_offset", 1
    )[0]
    assert "if not self._print_test_fiber_offset_locked:" in nudge_section
    assert "setText" in nudge_section
    assert "save_head_calibration" not in nudge_section
    assert "self._run_print_test_job" not in nudge_section


def test_fiber_offset_confirm_saves_and_downlinks_resin_z_adjusted_offset():
    src = _source()
    confirm_section = src.split("    def _on_print_test_confirm_fiber_offset", 1)[1].split(
        "    def _on_print_test_print_fiber", 1
    )[0]

    assert "start = self._print_test_current_correction" in confirm_section
    assert "start[0] + calibration.fiber_x_print_compensation_mm" in confirm_section
    assert "start[1] + calibration.fiber_y_print_compensation_mm" in confirm_section
    assert "calibration.resin_z_print_compensation_mm" in confirm_section
    assert "calibration.fiber_z_print_compensation_mm" in confirm_section
    assert "save_head_calibration(calibration" in confirm_section
    assert 'self._run_print_test_job("travel", start, target_pose=target)' in confirm_section


def test_scissor_button_checks_fiber_tool_before_uart_command():
    src = _source()
    cut_anchor = "    def _on_print_test_cut"
    assert cut_anchor in src

    tree = ast.parse(src)
    cut_fns = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_on_print_test_cut"
    ]
    assert cut_fns
    cut_fn = cut_fns[0]

    def is_current_tool_guard(if_node):
        test = if_node.test
        if not isinstance(test, ast.Compare):
            return False
        if len(test.ops) != 1 or not isinstance(test.ops[0], ast.NotEq):
            return False
        if len(test.comparators) != 1:
            return False

        call = test.left
        comparator = test.comparators[0]
        return (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == "current_tool_id"
            and isinstance(call.func.value, ast.Name)
            and call.func.value.id == "self"
            and not call.args
            and not call.keywords
            and isinstance(comparator, ast.Constant)
            and type(comparator.value) is int
            and comparator.value == 1
        )

    def is_cut_uart_emit(node):
        return (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "emit"
            and isinstance(node.func.value, ast.Attribute)
            and node.func.value.attr == "uart_command_submit"
            and isinstance(node.func.value.value, ast.Name)
            and node.func.value.value.id == "self"
            and len(node.args) == 1
            and not node.keywords
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "EV 0 cut_cf\n"
        )

    guard_ifs = [
        node
        for node in ast.walk(cut_fn)
        if isinstance(node, ast.If) and is_current_tool_guard(node)
    ]
    assert guard_ifs
    guard_if = guard_ifs[0]
    assert any(isinstance(stmt, ast.Return) for stmt in guard_if.body)

    uart_sends = sorted(
        (node for node in ast.walk(cut_fn) if is_cut_uart_emit(node)),
        key=lambda node: node.lineno,
    )
    assert uart_sends
    send_call = uart_sends[0]
    assert send_call.lineno > guard_if.end_lineno
