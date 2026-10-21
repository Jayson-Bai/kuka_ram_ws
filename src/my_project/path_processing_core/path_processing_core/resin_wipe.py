"""Geometry shared by full export and local resin-to-fiber tool changes."""

from .types import GlobalCurveCommand, MoveCommand, ToolChangeCommand


RESIN_WIPE_POLICY = "nearest_y_boundary_5mm_v1"
RESIN_WIPE_MM = 5.0


def wipe_target_y(start_y, specification):
    """Keep the full 5 mm motion; a boundary tie deterministically uses +Y."""
    low = float(specification["y_min"])
    high = float(specification["y_max"])
    direction = -1.0 if abs(float(start_y) - low) < abs(high - float(start_y)) else 1.0
    return float(start_y) + direction * RESIN_WIPE_MM


def command_resin_wipes(commands, initial_tool, map_tool):
    """Locate only resin deposition followed by a switch to the fiber head."""
    tool = int(initial_tool)
    bounds = {}
    last_layer = None
    resin_since_switch = False
    result = {}
    for index, command in enumerate(commands):
        if isinstance(command, ToolChangeCommand):
            target = map_tool(command.tool)
            if target != tool:
                if tool == 2 and target == 1 and resin_since_switch:
                    low, high = bounds[last_layer]
                    result[index] = {
                        "policy": RESIN_WIPE_POLICY,
                        "y_min": low, "y_max": high, "layer": last_layer,
                    }
                resin_since_switch = False
                tool = target
            continue
        if tool != 2 or not isinstance(command, (MoveCommand, GlobalCurveCommand)):
            continue
        if command.type not in ("PRINT", "PRINT_FIT"):
            continue
        originals = getattr(command, "original_moves", ()) or ()
        if any("primeline" in (getattr(item, "raw", "") or "").lower()
               for item in (command, *originals)):
            continue
        poses = [command.start_pos]
        if isinstance(command, MoveCommand):
            poses.append(command.pos)
        else:
            poses.extend(command.control_points)
        if not any(abs(p.x - poses[0].x) + abs(p.y - poses[0].y)
                   + abs(p.z - poses[0].z) > 1e-9 for p in poses[1:]):
            continue
        layer = max(0, int(getattr(command, "layer", 0) or 0))
        low, high = bounds.get(layer, (float("inf"), float("-inf")))
        bounds[layer] = (min(low, *(p.y for p in poses)), max(high, *(p.y for p in poses)))
        last_layer = layer
        resin_since_switch = True
    return result
