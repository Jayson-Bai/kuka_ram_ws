from external_npz_preprocessor.cli import build_parser


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
