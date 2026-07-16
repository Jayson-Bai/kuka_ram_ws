import pytest

from external_npz_preprocessor.cli import build_parser, params_from_args


def test_fiber_geometry_options_are_not_exposed_in_cli_help():
    help_text = build_parser().format_help()

    assert "--fiber-layer-height-mm" in help_text
    assert "--fiber-diameter-mm" not in help_text
    assert "--fiber-line-width-mm" not in help_text


def test_resin_line_width_option_is_not_exposed_in_cli_help():
    help_text = build_parser().format_help()

    assert "--resin-bead-width-mm" not in help_text


def test_height_accumulation_switch_is_not_exposed_in_cli_help():
    help_text = build_parser().format_help()

    assert "--accumulate-fiber-height-for-resin-layers" not in help_text


def test_fiber_start_acceleration_option_is_exposed_in_cli_help():
    help_text = build_parser().format_help()

    assert "--fiber-start-accel-s" in help_text


def test_prime_settle_s_defaults_to_half_second():
    args = build_parser().parse_args(["--source", "input.npz"])

    assert params_from_args(args).prime_settle_s == pytest.approx(0.5)


def test_prime_settle_s_cli_override_is_used():
    args = build_parser().parse_args(
        ["--source", "input.npz", "--prime-settle-s", "0.25"]
    )

    assert params_from_args(args).prime_settle_s == pytest.approx(0.25)
