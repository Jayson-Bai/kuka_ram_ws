from pathlib import Path


UART_NODE = Path(__file__).resolve().parents[1] / "src" / "uart_node.cpp"


def _source():
    return UART_NODE.read_text(encoding="utf-8")


def _between(source, start, end):
    return source.split(start, 1)[1].split(end, 1)[0]


def test_reset_requires_wire_ack_and_done():
    src = _source()
    reset_block = _between(
        src,
        '} else if (ev.planned.event_type == "extrude_reset") {',
        '} else if (ev.planned.event_type == "heat_cf") {',
    )
    assert "done = ev.ack_received && ev.done_received;" in reset_block
    assert "snapshot.last_e_seq" not in reset_block
    assert "snapshot.last_e_abs" not in reset_block
    assert "snapshot.last_e_us" not in reset_block


def test_print_test_reset_clears_state_without_claiming_completion():
    src = _source()
    reset_block = _between(
        src,
        'void on_print_test_command(const std::string & cmd)',
        "void on_system_command",
    )
    assert 'cmd == "RESET"' in reset_block
    assert "current_event_.reset();" in reset_block
    assert "pending_cut_.reset();" in reset_block
    assert "event_fault_latched_ = false;" in reset_block
    assert "set_ready_state(true" not in reset_block
    assert 'publish_ready_state("print_test_reset")' not in reset_block


def test_wire_request_ids_are_nonzero_unique_and_not_rsi_trigger_seq():
    src = _source()
    assert "allocate_wire_request_id()" in src
    assert "next_wire_request_id_.fetch_add" in src
    handler = _between(
        src,
        "void on_triggered_event(const PlannedEvent & ev)",
        "void has_pending_event",
    )
    assert "const uint32_t wire_request_id = allocate_wire_request_id();" in handler
    assert 'oss << "EV " << trigger_seq_or_zero' not in handler
    assert 'EV " << wire_request_id' in handler


def test_cut_declares_two_sync_members_and_nonblocking_ready_does_not_forget_cut():
    src = _source()
    handler = _between(
        src,
        "void on_triggered_event(const PlannedEvent & ev)",
        "void has_pending_event",
    )
    cut_block = _between(
        handler,
        'if (ev.event_type == "cut") {',
        '} else if (ev.event_type == "extrude_sync") {',
    )
    assert "parse_cut_sync_contract" in cut_block
    assert "expected_sync_count" in cut_block
    assert "if (current_event_ || pending_cut_)" in cut_block
    assert "pending_cut_ = PendingCut{" in cut_block
    assert 'set_ready_state(true, ev.trigger_seq, ev.event_type);' in handler
    assert "pending_cut_.reset();" not in handler.split(
        'set_ready_state(true, ev.trigger_seq, ev.event_type);', 1
    )[1].split("return;", 1)[0]


def test_two_syncs_share_one_cut_owner_and_release_after_second():
    src = _source()
    admission = _between(
        src,
        '} else if (ev.event_type == "extrude_sync") {',
        '} else {\n        if (current_event_)',
    )
    assert "expected_sync_count == 0U" in admission
    assert "sync_done_count >= pending_cut_->expected_sync_count" in admission
    assert "pending_cut_->wire_request_id" in admission

    completion = _between(
        src,
        "void check_event_completion()",
        "bool write_line",
    )
    assert "++pending_cut_->sync_done_count;" in completion
    assert "pending_cut_->sync_done_count >= pending_cut_->expected_sync_count" in completion
    assert "const bool final_sync =" not in completion
    sync_block = _between(
        completion,
        'if (ev.planned.event_type == "extrude_sync") {',
        '} else if (ev.planned.event_type == "extrude_reset") {',
    )
    done_expression = sync_block.split(
        "done = ev.ack_received && ev.done_received", 1
    )[1].split(";", 1)[0]
    assert "pending_cut_->ack_received" in done_expression
    assert "pending_cut_->done_received" in done_expression
    release = completion.split(
        "++pending_cut_->sync_done_count;", 1
    )[1].split("pending_cut_.reset();", 1)[0]
    assert "expected_sync_count" in release
    assert "pending_cut_->done_received" in release


def test_first_sync_waits_for_both_cutter_strokes_before_the_retract():
    src = _source()
    completion = _between(
        src,
        "void check_event_completion()",
        "bool write_line",
    )
    sync_block = _between(
        completion,
        'if (ev.planned.event_type == "extrude_sync") {',
        '} else if (ev.planned.event_type == "extrude_reset") {',
    )
    # The safety retract sits between the two syncs, so the CUT DONE gate may
    # not be deferred to the last sync: the first sync has to hold RSI in WAIT
    # until the firmware reports both cutter strokes finished.
    assert "final_sync" not in sync_block
    done_expression = sync_block.split(
        "done = ev.ack_received && ev.done_received", 1
    )[1].split(";", 1)[0]
    assert "pending_cut_->ack_received" in done_expression
    assert "pending_cut_->done_received" in done_expression


def test_incomplete_cut_holds_last_sync_without_generating_a_fault():
    src = _source()
    completion = _between(
        src,
        "void check_event_completion()",
        "bool write_line",
    )
    sync_block = _between(
        completion,
        'if (ev.planned.event_type == "extrude_sync") {',
        '} else if (ev.planned.event_type == "extrude_reset") {',
    )
    assert "pending_cut_->done_received == false" not in sync_block
    assert "completion_fault = \"cut_owner_invalid\";" in sync_block
    assert sync_block.index("if (!pending_cut_ || pending_cut_->failed") < sync_block.index(
        "done = ev.ack_received && ev.done_received"
    )

    timeout = _between(
        src,
        "void check_event_timeouts()",
        "void parse_stat",
    )
    assert "completed_sync_waiting_for_cut" in timeout
    assert "!completed_sync_waiting_for_cut" in timeout


def test_sync_done_validates_target_tool_and_cut_request_id():
    src = _source()
    response = _between(
        src,
        "void handle_event_response(",
        "void handle_event_failure",
    )
    sync_done = _between(
        response,
        'event_type == "extrude_sync") {',
        "            } else {\n              current_event_->done_received = true;",
    )
    assert 'parse_target_um(tokens, reported_target)' in sync_done
    assert 'parse_integer_field(tokens, "tool=", reported_tool)' in sync_done
    assert (
        'parse_integer_field(tokens, "cut_request_id=", reported_cut_request)'
        in sync_done
    )
    assert "reported_target != *current_event_->expected_target_um" in sync_done
    assert "reported_tool != *current_event_->expected_tool" in sync_done
    assert (
        "reported_cut_request != static_cast<int64_t>(*current_event_->cut_request_id)"
        in sync_done
    )


def test_unowned_or_malformed_responses_fail_closed():
    src = _source()
    response = _between(
        src,
        "void handle_event_response(",
        "void handle_event_failure",
    )
    assert 'mark_event_fault("malformed_response", 0, "uart")' in response
    assert 'mark_event_fault("invalid_response_seq", 0, "uart")' in response
    assert 'mark_event_fault("unexpected_response", event_seq, event_type)' in response
    assert 'mark_event_fault("sync_identity_mismatch", event_seq, event_type)' in response

    failure = _between(
        src,
        "void handle_event_failure(",
        "void check_event_timeouts",
    )
    assert "mark_event_fault(reason, event_seq, event_type);" in failure


def test_timeouts_cover_unfinished_cut_and_orphaned_sync_pair():
    src = _source()
    timeout = _between(
        src,
        "void check_event_timeouts()",
        "void parse_stat",
    )
    assert "now - current_event_->sent_at > current_event_->timeout" in timeout
    assert "!pending_cut_->done_received" in timeout
    assert "reason = \"cut_timeout\";" in timeout
    assert "sync_done_count < pending_cut_->expected_sync_count" in timeout
    assert "reason = \"cut_sync_incomplete\";" in timeout


def test_heartbeat_is_serialized_against_event_barrier_and_retries_after_done():
    src = _source()
    heartbeat = _between(
        src,
        "void on_heartbeat(const RsiHeartBeat & hb)",
        "void read_loop",
    )
    assert "std::lock_guard<std::mutex> event_lk(event_mutex_);" in heartbeat
    assert "event_fault_latched_ || current_event_" in heartbeat
    assert "extrusion_forwarder_->prepare" in heartbeat
    assert "write_line(candidate.line)" in heartbeat
    assert "extrusion_forwarder_->commit(candidate)" in heartbeat

    reset = _between(
        src,
        "void reset_extrude_forward_state()",
        "void on_heartbeat",
    )
    assert "extrusion_forwarder_->reset()" in reset
