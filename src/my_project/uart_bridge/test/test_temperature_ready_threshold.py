from pathlib import Path


UART_NODE = Path(__file__).resolve().parents[1] / "src" / "uart_node.cpp"


def test_heat_event_ready_threshold_is_five_celsius():
    src = UART_NODE.read_text(encoding="utf-8")

    assert "constexpr float TEMP_EPS = 5.0f;" in src
    assert "constexpr float TEMP_EPS = 2.0f;" not in src
