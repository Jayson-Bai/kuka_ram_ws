from pathlib import Path


UART_NODE = Path(__file__).resolve().parents[1] / "src" / "uart_node.cpp"


def _source():
    return UART_NODE.read_text(encoding="utf-8")


def test_extrude_reset_waits_for_firmware_ack_done_and_stat_clear():
    src = _source()

    assert "handle_event_response(line, false)" in src
    assert "handle_event_response(line, true)" in src
    assert "current_event_ack_received_" in src
    assert "current_event_done_received_" in src
    assert "last_e_seq" in src
    assert "last_e_abs" in src
    assert "last_e_us" in src
    assert "snapshot.last_e_seq == -1" in src
    assert "std::abs(snapshot.last_e_abs)" in src
    assert "snapshot.last_e_us == 0" in src
    reset_block = src.split('else if (ev.event_type == "extrude_reset")', 1)[1].split("}", 1)[0]
    assert "current_event_ack_received_" in reset_block
    assert "current_event_done_received_" in reset_block
    assert "done = true" not in reset_block
