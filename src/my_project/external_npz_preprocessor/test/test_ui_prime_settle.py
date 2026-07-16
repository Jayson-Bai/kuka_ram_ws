from pathlib import Path


UI = Path(__file__).resolve().parents[1] / "external_npz_preprocessor" / "ui.py"


def _source() -> str:
    return UI.read_text(encoding="utf-8")


def test_prime_settle_control_defaults_to_nonnegative_half_second():
    src = _source()
    build_ui = src.split("    def _build_ui(self):", 1)[1].split(
        "    def _refresh_offset_info", 1
    )[0]

    assert "self.prime_settle_s = self._spin(0.5)" in build_ui
    assert (
        '("预挤出稳定等待 s", self.prime_settle_s, "空走速度 mm/s", self.travel_feed)'
        in build_ui
    )
    assert "def _spin(self, value, minimum=0.0, maximum=100000.0):" in src


def test_loaded_prime_settle_is_applied_to_the_control():
    src = _source()
    load_params = src.split("    def _load_saved_params(self):", 1)[1].split(
        "    def _apply_params", 1
    )[0]
    apply_params = src.split("    def _apply_params", 1)[1].split(
        "    def _params", 1
    )[0]

    assert "self._apply_params(load_print_params(path))" in load_params
    assert "self.prime_settle_s.setValue(params.prime_settle_s)" in apply_params


def test_params_reconstructs_prime_settle_from_the_control():
    src = _source()
    params = src.split("    def _params(self) -> ProcessParams:", 1)[1].split(
        "    def _save_params", 1
    )[0]

    assert "prime_settle_s=self.prime_settle_s.value()" in params


def test_save_persists_reconstructed_process_params():
    src = _source()
    save_params = src.split("    def _save_params(self):", 1)[1].split(
        "    def _validate_source", 1
    )[0]

    assert "save_print_params(self._params())" in save_params
