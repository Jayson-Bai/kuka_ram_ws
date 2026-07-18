from pathlib import Path


CENTER_NODE = Path(__file__).resolve().parents[1] / "src" / "center_node.cpp"


def _source():
    return CENTER_NODE.read_text(encoding="utf-8")


def test_center_auto_aborts_only_after_final_published_seq_is_used_by_rsi():
    src = _source()

    assert "cmd_pub_" in src
    assert "final_traj_seq_" in src
    assert "final_seq_heartbeat_count_" in src
    assert "auto_abort_sent_" in src
    assert "check_print_complete(*msg)" in src

    prefill_block = src.split("void initial_prefill", 1)[1].split("void load_print_test_npz", 1)[0]
    publish_block = src.split("void publish_from_queue", 1)[1].split("void remember_if_final_traj_locked", 1)[0]
    assert "remember_if_final_traj_locked(tp.seq)" in prefill_block
    assert "remember_if_final_traj_locked(tp.seq)" in publish_block

    remember_block = src.split("void remember_if_final_traj_locked", 1)[1].split("void check_print_complete", 1)[0]
    assert "final_traj_seq_ = seq" in remember_block
    assert "final_seq_heartbeat_count_ = 0" in remember_block
    assert "auto_abort_sent_.store(false)" in remember_block

    complete_block = src.split("void check_print_complete", 1)[1].split("void on_system_command", 1)[0]
    assert "msg.seq_used != *final_seq" in complete_block
    assert "publish_system_command(\"ABORT\")" in complete_block
    assert "path_end_flag" not in complete_block


def test_center_does_not_auto_abort_while_test_mode_is_active():
    src = _source()

    assert "std::atomic<bool> print_test_mode_active_" in src
    reset_block = src.split(
        "void on_print_test_command(const std::string & cmd)", 1
    )[1].split("void load_print_test_npz", 1)[0]
    load_block = src.split("void load_print_test_npz", 1)[1].split(
        "void publish_from_queue", 1
    )[0]
    complete_block = src.split("void check_print_complete", 1)[1].split(
        "void on_system_command", 1
    )[0]

    assert "print_test_mode_active_.store(true)" in reset_block
    assert "print_test_mode_active_.store(true)" in load_block
    assert "if (print_test_mode_active_.load())" in complete_block
    assert complete_block.index("if (print_test_mode_active_.load())") < complete_block.index(
        "std::optional<uint32_t> final_seq"
    )


def test_manual_abort_exits_test_mode_without_removing_abort_cleanup():
    src = _source()
    command_block = src.split(
        "void on_system_command(const std::string & cmd)", 1
    )[1].split("};", 1)[0]

    assert "cmd == \"ABORT\"" in command_block
    assert "print_test_mode_active_.store(false)" in command_block
    assert "aborted_.store(true)" in command_block
    assert "queue_manager_->clear()" in command_block
