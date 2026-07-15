from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CENTER = ROOT / "control_center" / "src" / "center_node.cpp"
RSI = ROOT / "rsi_server" / "src" / "rsi_node.cpp"
UART = ROOT / "uart_bridge" / "src" / "uart_node.cpp"
TRAJ = ROOT / "my_project_interfaces" / "msg" / "TrajectoryPoint.msg"
UI = ROOT / "my_project_ui" / "my_project_ui" / "ui_panel.py"
LAUNCH = ROOT / "my_project_startup" / "launch" / "startup.launch.py"
NPZ_HEADER = ROOT / "control_center" / "include" / "control_center" / "npz_loader.hpp"
NPZ_LOADER_CPP = ROOT / "control_center" / "src" / "npz_loader.cpp"
QUEUE = ROOT / "control_center" / "src" / "queue_manager.cpp"


def _read(path):
    return path.read_text(encoding="utf-8")


def test_ui_pause_button_requests_staged_pause_without_redefining_immediate_pause():
    ui = _read(UI)
    assert 'self.command_submit.emit("REQUEST_PAUSE")' in ui
    assert 'cmd == "ABORT"' in ui


def test_uart_keeps_extrusion_forwarding_during_staged_pause_request():
    uart = _read(UART)
    assert 'cmd == "REQUEST_PAUSE"' not in uart.split('void on_system_command', 1)[1]
    assert 'cmd == "PAUSE"' in uart
    assert 'paused_.store(true)' in uart


def test_path_metadata_is_part_of_runtime_trajectory_contract():
    msg = _read(TRAJ)
    assert 'uint32 path_id' in msg
    assert 'bool path_end_flag' in msg
    header = _read(NPZ_HEADER)
    assert 'uint32_t path_id' in header
    assert 'bool path_end_flag' in header
    queue = _read(QUEUE)
    assert 'tp.path_id = row.path_id' in queue
    assert 'tp.path_end_flag = row.path_end_flag' in queue


def test_rsi_has_staged_pause_states_and_uses_path_end_flag():
    rsi = _read(RSI)
    assert 'REQUEST_PAUSE' in rsi
    assert 'PAUSE_ARMED' in rsi
    assert 'PAUSE_RETRACT' in rsi
    assert 'PAUSE_LIFT' in rsi
    assert 'PAUSE_RETURN' in rsi
    assert 'path_end_flag' in rsi
    assert 'pause_requested_tool_id_' in rsi
    assert 'cut' in rsi


def test_rsi_cut_event_is_nonblocking_and_continues_trajectory_same_cycle():
    rsi = _read(RSI)
    assert "bool is_nonblocking_event" in rsi
    event_block = rsi.split("if (should_enter_wait(next_seq_))", 1)[1].split("} else if (current_state == State::WAIT)", 1)[0]
    assert "if (is_nonblocking_event(*current_wait_))" in event_block
    assert "current_wait_.reset();" in event_block
    assert "state_.store(State::WAIT);" in event_block
    assert "if (!current_wait_)" in event_block


def test_center_reports_staged_pause_states_but_old_pause_remains_immediate():
    center = _read(CENTER)
    assert 'REQUEST_PAUSE' in center
    assert 'PAUSE_REQUESTED' in center
    assert 'PAUSE_READY' in center
    assert 'cmd == "PAUSE"' in center


def test_pause_safety_parameters_are_launch_configurable():
    ui = _read(UI)
    launch = _read(LAUNCH)
    for name in (
        "pause_safe_lift_mm",
        "pause_lift_speed_mm_s",
        "pause_retract_mm",
        "pause_retract_speed_mm_s",
    ):
        assert name in ui
        assert f'LaunchConfiguration("{name}")' in launch
        assert f'"{name}": {name}' in launch
        assert 'DeclareLaunchArgument(\n            "' + name + '"' in launch



def test_runtime_trajectory_contract_carries_optional_timing():
    msg = _read(TRAJ)
    assert "planned_time_s" in msg
    assert "planned_total_time_s" in msg
    assert "bool planned_time_valid" in msg
    header = _read(NPZ_HEADER)
    assert "planned_time_s" in header
    assert "planned_total_time_s" in header
    queue = _read(QUEUE)
    assert "planned_time_valid" in queue


def test_npz_loader_exposes_optional_timing_metadata():
    header = _read(NPZ_HEADER)
    loader_cpp = _read(NPZ_LOADER_CPP)
    assert "bool timing_valid() const" in header
    assert "double total_planned_time_s() const" in header
    assert 'npz.count("planned_time_s")' in loader_cpp
    assert "timing.json" in loader_cpp
