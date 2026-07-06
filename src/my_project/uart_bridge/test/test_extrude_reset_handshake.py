from pathlib import Path


UART_NODE = Path(__file__).resolve().parents[1] / "src" / "uart_node.cpp"


def _source():
    return UART_NODE.read_text(encoding="utf-8")


def test_extrude_reset_can_complete_from_stat_clear_without_ack_done():
    src = _source()

    assert "handle_event_response(line, false)" in src
    assert "handle_event_response(line, true)" in src
    assert "last_e_seq" in src
    assert "last_e_abs" in src
    assert "last_e_us" in src
    assert "snapshot.last_e_seq == -1" in src
    assert "std::abs(snapshot.last_e_abs)" in src
    assert "snapshot.last_e_us == 0" in src
    reset_block = src.split('else if (ev.event_type == "extrude_reset")', 1)[1].split("}", 1)[0]
    assert "done = stat_cleared" in reset_block
    assert "current_event_ack_received_ && current_event_done_received_" not in reset_block


def test_print_test_reset_does_not_fake_event_completion():
    src = _source()

    assert 'print_test_cmd_sub_' in src
    assert '"/print_test/rsi_command"' in src
    assert 'on_print_test_command(msg->data)' in src
    assert 'void on_print_test_command(const std::string & cmd)' in src
    reset_block = src.split('void on_print_test_command(const std::string & cmd)', 1)[1].split('void on_system_command', 1)[0]
    assert 'cmd == "RESET"' in reset_block
    assert 'current_event_ack_received_ = false' in reset_block
    assert 'current_event_done_received_ = false' in reset_block
    assert 'set_ready_state(true' not in reset_block
    assert 'publish_ready_state("print_test_reset")' not in reset_block


def test_triggered_event_sends_firmware_compatible_ev_zero_before_marking_not_ready():
    src = _source()

    handler = src.split("void on_triggered_event(const PlannedEvent & ev)", 1)[1].split(
        "void on_heartbeat", 1
    )[0]
    assert 'oss << "EV 0 " << ev.event_type << " " << arg << "\\n";' in handler
    assert 'write_line(oss.str())' in handler
    assert 'set_ready_state(false, ev.trigger_seq, ev.event_type)' in handler
    assert handler.index('write_line(oss.str())') < handler.index(
        'set_ready_state(false, ev.trigger_seq, ev.event_type)'
    )


def test_cut_event_uses_trigger_seq_and_waits_for_ack_done():
    src = _source()

    handler = src.split("void on_triggered_event(const PlannedEvent & ev)", 1)[1].split(
        "void on_heartbeat", 1
    )[0]
    assert 'if (ev.event_type == "cut")' in handler
    assert 'oss << "EV " << ev.trigger_seq << " cut " << arg << "\\n";' in handler

    completion = src.split("void check_event_completion()", 1)[1].split(
        "void send_extrude_command", 1
    )[0]
    assert 'else if (ev.event_type == "cut")' in completion
    cut_block = completion.split('else if (ev.event_type == "cut")', 1)[1].split("}", 1)[0]
    assert "current_event_ack_received_ && current_event_done_received_" in cut_block


def test_heartbeat_does_not_forward_e_while_event_is_pending():
    src = _source()

    heartbeat = src.split("void on_heartbeat(const RsiHeartBeat & hb)", 1)[1].split(
        "void read_loop", 1
    )[0]
    assert "has_pending_event()" in heartbeat
    assert "return;" in heartbeat.split("has_pending_event()", 1)[1].split("}", 1)[0]
    assert "bool has_pending_event() const" in src



def test_heartbeat_skips_unchanged_extrusion_values():
    src = _source()

    heartbeat = src.split("void on_heartbeat(const RsiHeartBeat & hb)", 1)[1].split(
        "void read_loop", 1
    )[0]
    assert "should_forward_extrude" in heartbeat
    assert "send_extrude_command" in heartbeat
    assert heartbeat.index("should_forward_extrude") < heartbeat.index("send_extrude_command")
    assert "last_sent_e_valid_" in src
    assert "last_sent_e_abs_" in src
