from pathlib import Path

import pytest

from external_npz_preprocessor.param_config import (
    default_print_params_path,
    load_print_params,
    save_print_params,
)
from external_npz_preprocessor.process_params import FiberProcessParams, ProcessParams, ResinProcessParams


def test_save_and_load_print_params_round_trip(tmp_path):
    path = tmp_path / "print_params.json"
    params = ProcessParams(
        resin=ResinProcessParams(
            layer_height_mm=0.6,
            extrusion_scale=1.2,
            feed_mm_s=11.0,
            temperature_c=245.0,
            fan_enabled=False,
            prime_length_mm=19.0,
            prime_speed_mm_s=16.0,
            retract_length_mm=14.0,
            retract_speed_mm_s=28.0,
        ),
        fiber=FiberProcessParams(
            layer_height_mm=0.12,
            extrusion_scale=1.3,
            feed_mm_s=9.0,
            temperature_c=255.0,
            fan_enabled=False,
            prime_length_mm=13.0,
            prime_speed_mm_s=6.0,
            retract_length_mm=9.0,
            retract_speed_mm_s=4.0,
        ),
        travel_feed_mm_s=12.0,
        default_a=1.0,
        default_b=2.0,
        default_c=3.0,
    )

    save_print_params(params, path)
    loaded = load_print_params(path)

    assert loaded == params


def test_default_print_params_path_uses_data_directory(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "ignored_config"))
    monkeypatch.setenv("APPDATA", str(tmp_path / "ignored_appdata"))

    assert default_print_params_path(data_root=tmp_path) == (
        tmp_path / "external_npz_preprocessor" / "print_params.json"
    )


def test_legacy_fiber_geometry_fields_are_ignored_on_load(tmp_path):
    path = tmp_path / "legacy.json"
    path.write_text(
        '{"params":{"fiber":{"diameter_mm":0.6,"line_width_mm":1.0,"layer_height_mm":0.11}}}',
        encoding="utf-8",
    )

    params = load_print_params(path)

    assert params.fiber.layer_height_mm == 0.11
    assert not hasattr(params.fiber, "diameter_mm")
    assert not hasattr(params.fiber, "line_width_mm")


def test_legacy_resin_bead_width_is_ignored_on_load(tmp_path):
    path = tmp_path / "legacy_resin.json"
    path.write_text(
        '{"params":{"resin":{"bead_width_mm":99.0,"layer_height_mm":0.4,"extrusion_scale":2.0}}}',
        encoding="utf-8",
    )

    params = load_print_params(path)

    assert not hasattr(params.resin, "bead_width_mm")
    assert params.resin.layer_height_mm == 0.4
    assert params.resin.e_per_mm() == pytest.approx(1.6)
