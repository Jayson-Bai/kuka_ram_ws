from pathlib import Path


UART_NODE = Path(__file__).resolve().parents[1] / "src" / "uart_node.cpp"


def test_heat_event_ready_threshold_is_twenty_celsius_and_symmetric():
    src = UART_NODE.read_text(encoding="utf-8")

    assert "constexpr float TEMP_EPS = 20.0f;" in src
    assert "std::abs(snapshot.current_temp_cf - snapshot.target_temp_cf) <= TEMP_EPS" in src
    assert "std::abs(snapshot.current_temp_resin - snapshot.target_temp_resin) <= TEMP_EPS" in src
    assert "constexpr float TEMP_EPS = 5.0f;" not in src
