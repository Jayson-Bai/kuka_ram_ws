import re
from pathlib import Path


UART_NODE = Path(__file__).resolve().parents[1] / "src" / "uart_node.cpp"
STARTUP_LAUNCH = (
    Path(__file__).resolve().parents[2]
    / "my_project_startup"
    / "launch"
    / "startup.launch.py"
)


def _source():
    return UART_NODE.read_text(encoding="utf-8")


def _startup_source():
    return STARTUP_LAUNCH.read_text(encoding="utf-8")


def _between(source, start, end):
    assert start in source, f"missing start marker: {start!r}"
    after_start = source.split(start, 1)[1]
    assert end in after_start, f"missing end marker after {start!r}: {end!r}"
    return after_start.split(end, 1)[0]


def test_heartbeat_keeps_guards_and_commits_only_after_write_success():
    heartbeat = _between(
        _source(),
        "void on_heartbeat(const RsiHeartBeat & hb)",
        "void read_loop",
    )

    assert "paused_.load() || aborted_.load() || has_pending_event()" in heartbeat
    assert "hb.extrude_abs * scale" in heartbeat
    assert "std::lock_guard<std::mutex> lk(extrude_forward_mutex_)" in heartbeat
    assert "extrusion_forwarder_->prepare" in heartbeat
    assert "write_line(candidate.line)" in heartbeat
    commit_call = "extrusion_forwarder_->commit(candidate);"
    assert heartbeat.count(commit_call) == 1
    assert heartbeat.index(
        "std::lock_guard<std::mutex> lk(extrude_forward_mutex_)"
    ) < heartbeat.index(
        "extrusion_forwarder_->prepare"
    )
    assert heartbeat.index("extrusion_forwarder_->prepare") < heartbeat.index(
        "write_line(candidate.line)"
    )
    assert heartbeat.index("write_line(candidate.line)") < heartbeat.index(
        commit_call
    )
    assert re.search(
        r"if\s*\(\s*write_line\(candidate\.line\)\s*\)\s*\{\s*"
        r"extrusion_forwarder_->commit\(candidate\);\s*\}",
        heartbeat,
    )
    assert "static_cast<float>(scaled)" not in heartbeat
    assert "should_forward_extrude" not in heartbeat
    assert "send_extrude_command" not in heartbeat


def test_write_line_preserves_log_and_returns_complete_delivery_status():
    write_line = _between(
        _source(),
        "bool write_line(const std::string & line)",
        "void on_print_test_command",
    )

    assert "publish_uart_log(\"TX\", line)" in write_line
    assert "boost::asio::write" in write_line
    assert write_line.index("publish_uart_log(\"TX\", line)") < write_line.index(
        "boost::asio::write"
    )
    assert "TX_FAIL" not in write_line
    assert "ec || written != line.size()" in write_line
    assert "return false;" in write_line
    assert "return true;" in write_line


def test_node_defaults_to_canonical_and_parses_static_mode():
    source = _source()

    assert re.search(
        r'declare_parameter<std::string>\(\s*"extrusion_wire_mode",'
        r'\s*"canonical_v1"',
        source,
    )
    assert "uart_bridge::parse_extrusion_wire_mode" in source
    assert "std::make_unique<uart_bridge::ExtrusionForwarder>" in source
    callback = _between(source, "add_on_set_parameters_callback(", "//订阅心跳包")
    assert 'p.get_name() == "extrude_scale"' in callback
    assert "extrusion_wire_mode" not in callback


def test_extrusion_wire_mode_is_a_read_only_startup_parameter():
    source = _source()
    mode_setup = _between(
        source,
        'declare_parameter<double>("extrude_scale", 1.0)',
        "param_cb_ = add_on_set_parameters_callback(",
    )

    assert '#include <rcl_interfaces/msg/parameter_descriptor.hpp>' in source
    assert (
        "rcl_interfaces::msg::ParameterDescriptor extrusion_wire_mode_descriptor;"
        in mode_setup
    )
    assert "extrusion_wire_mode_descriptor.read_only = true;" in mode_setup
    assert re.search(
        r'declare_parameter<std::string>\(\s*"extrusion_wire_mode",'
        r'\s*"canonical_v1",\s*extrusion_wire_mode_descriptor\s*\)',
        mode_setup,
    )


def test_startup_launch_passes_extrusion_wire_mode_only_to_uart_node():
    source = _startup_source()
    launch_configuration = (
        'extrusion_wire_mode = LaunchConfiguration("extrusion_wire_mode")'
    )
    uart_parameter = '"extrusion_wire_mode": extrusion_wire_mode,'
    launch_argument_name = '"extrusion_wire_mode",'

    assert source.count(launch_configuration) == 1

    rsi_node = _between(source, "    rsi_node = Node(", "\n    uart_node = Node(")
    uart_node = _between(source, "    uart_node = Node(", "\n    center_node = Node(")
    assert "extrusion_wire_mode" not in rsi_node
    assert uart_node.count(uart_parameter) == 1

    extrude_to_abort_arguments = _between(
        source,
        '        DeclareLaunchArgument(\n            "extrude_scale",',
        '        DeclareLaunchArgument(\n            "abort_lift_mm",',
    )
    assert extrude_to_abort_arguments.count("DeclareLaunchArgument(") == 1
    assert re.search(
        r'DeclareLaunchArgument\(\s*"extrusion_wire_mode",\s*'
        r'default_value="canonical_v1",\s*'
        r'description="[^"]*canonical_v1/legacy_v1[^"]*",\s*\)',
        extrude_to_abort_arguments,
    )

    remainder = source
    for expected in (launch_configuration, uart_parameter, launch_argument_name):
        assert remainder.count(expected) == 1
        remainder = remainder.replace(expected, "", 1)
    assert "extrusion_wire_mode" not in remainder


def test_reset_path_delegates_to_pure_forwarder():
    reset = _between(
        _source(),
        "void reset_extrude_forward_state()",
        "void on_heartbeat",
    )

    assert "std::lock_guard<std::mutex> lk(extrude_forward_mutex_)" in reset
    assert "extrusion_forwarder_->reset()" in reset
