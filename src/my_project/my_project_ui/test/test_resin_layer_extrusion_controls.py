"""Exercise UI parsing/persistence without starting ROS or hardware nodes."""
import ast
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import pytest


UI_PANEL = Path(__file__).resolve().parents[1] / "my_project_ui" / "ui_panel.py"
KEYS = ("resin_layer_end_retract_mm", "resin_layer_start_prime_mm")


def _functions(tmp_path):
    tree = ast.parse(UI_PANEL.read_text(encoding="utf-8"))
    names = {"_optional_resin_extrusion_mm", "_load_core_resin_extrusion_config",
             "_save_core_resin_extrusion_config", "_local_injection_values"}
    functions = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                 and node.name in names]
    ns = {"math": math, "json": json, "os": os,
          "_OFFSET_CONFIG_DIR": str(tmp_path),
          "_CORE_RESIN_EXTRUSION_CONFIG_PATH": str(tmp_path / "settings.json"),
          "_CORE_RESIN_EXTRUSION_DEFAULTS": dict.fromkeys(KEYS)}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(UI_PANEL), "exec"), ns)
    return ns


def test_empty_zero_and_explicit_values_are_distinct_and_persist(tmp_path):
    ns = _functions(tmp_path)
    assert ns["_load_core_resin_extrusion_config"]() == dict.fromkeys(KEYS)
    values = {KEYS[0]: "0", KEYS[1]: " 3.125 "}
    ns["_save_core_resin_extrusion_config"](values)
    assert ns["_load_core_resin_extrusion_config"]() == {KEYS[0]: 0, KEYS[1]: 3.125}
    values[KEYS[1]] = " "
    ns["_save_core_resin_extrusion_config"](values)
    assert ns["_load_core_resin_extrusion_config"]() == {KEYS[0]: 0, KEYS[1]: None}
    assert not (tmp_path / "cut_params.json").exists()


@pytest.mark.parametrize("text", ["-1", "nan", "inf", "invalid"])
def test_invalid_input_rejected(tmp_path, text):
    ns = _functions(tmp_path)
    with pytest.raises(ValueError):
        ns["_save_core_resin_extrusion_config"]({KEYS[0]: text})
    assert not (tmp_path / "settings.json").exists()


def test_value_reader_keeps_existing_controls_and_optional_semantics(tmp_path):
    ns = _functions(tmp_path)
    panel = SimpleNamespace(
        get_tool_offset=lambda: (1, 2, 3),
        current_resin_z_print_compensation=lambda: 0.2,
        _local_injection_inputs={
            "cut_wait_s": SimpleNamespace(value=lambda: 15),
            "cut_settle_s": SimpleNamespace(value=lambda: 2.5),
            KEYS[0]: SimpleNamespace(text=lambda: "0"),
            KEYS[1]: SimpleNamespace(text=lambda: ""),
        },
    )
    values = ns["_local_injection_values"](panel)
    assert values == {"tool_offset": (1, 2, 3), "resin_z_print_compensation_mm": 0.2,
                      "cut_wait_s": 15, "cut_settle_s": 2.5,
                      KEYS[0]: 0, KEYS[1]: None}


def test_all_three_final_injection_calls_receive_optional_values():
    tree = ast.parse(UI_PANEL.read_text(encoding="utf-8"))
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == "inject_npz"]
    assert len(calls) == 3
    for call in calls:
        for key in KEYS:
            keyword = next(item for item in call.keywords if item.arg == key)
            assert isinstance(keyword.value, ast.Call)
            assert keyword.value.func.attr == "get"
            assert keyword.value.args[0].value == key


def test_all_three_final_injection_calls_receive_cut_settle():
    tree = ast.parse(UI_PANEL.read_text(encoding="utf-8"))
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == "inject_npz"]
    assert len(calls) == 3
    for call in calls:
        keyword = next(item for item in call.keywords if item.arg == "cut_settle_s")
        assert isinstance(keyword.value, ast.Subscript)
        assert keyword.value.slice.value == "cut_settle_s"
