from pathlib import Path


CENTER_NODE = Path(__file__).resolve().parents[1] / "src" / "center_node.cpp"


def _source():
    return CENTER_NODE.read_text(encoding="utf-8")


def test_print_test_reset_clears_center_loader_before_next_dynamic_npz():
    src = _source()

    assert "print_test_cmd_sub_" in src
    assert '"/print_test/rsi_command"' in src
    assert "on_print_test_command(msg->data)" in src
    assert "void on_print_test_command(const std::string & cmd)" in src

    reset_block = src.split(
        "void on_print_test_command(const std::string & cmd)", 1
    )[1].split("void load_print_test_npz", 1)[0]
    assert 'cmd == "RESET"' in reset_block
    assert "queue_manager_->clear()" in reset_block
    assert "npz_loader_.reset()" in reset_block
    assert "queue_manager_.reset()" in reset_block
    assert "last_published_traj_seq_.reset()" in reset_block
