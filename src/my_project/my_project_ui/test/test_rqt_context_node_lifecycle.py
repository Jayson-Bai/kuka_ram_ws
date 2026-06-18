from pathlib import Path


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"


def _source():
    return UI_PANEL.read_text(encoding="utf-8")


def test_shutdown_does_not_destroy_rqt_context_node():
    src = _source()

    assert "self._node = context.node" in src
    shutdown_body = src.split("    def shutdown_plugin(self):", 1)[1]
    assert "destroy_node(" not in shutdown_body
