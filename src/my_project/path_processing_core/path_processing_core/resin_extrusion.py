"""Explicit entity-layer prime/retract locations; primeline is never an entity."""

from .types import ExtrudeWait, GlobalCurveCommand, MoveCommand, ToolChangeCommand


BOUNDARY_FORMAT = "resin_entity_layer_boundaries_v1"
BOUNDARY_PARAMS = {
    "resin_layer_start_prime_mm": ("resin_layer_start_prime", 1),
    "resin_layer_end_retract_mm": ("resin_layer_end_retract", -1),
}


def command_resin_boundaries(commands, initial_tool, map_tool):
    tool = int(initial_tool)
    tools, spatial, entities = [], set(), {}
    for index, command in enumerate(commands):
        if isinstance(command, ToolChangeCommand):
            tool = map_tool(command.tool)
        tools.append(tool)
        if not isinstance(command, (MoveCommand, GlobalCurveCommand)):
            continue
        if command.type not in ("PRINT", "PRINT_FIT"):
            continue
        poses = [command.start_pos]
        poses.extend([command.pos] if isinstance(command, MoveCommand) else command.control_points)
        if not any(abs(p.x - poses[0].x) + abs(p.y - poses[0].y)
                   + abs(p.z - poses[0].z) > 1e-9 for p in poses[1:]):
            continue
        spatial.add(index)
        originals = getattr(command, "original_moves", ()) or ()
        primeline = any("primeline" in (getattr(item, "raw", "") or "").lower()
                        for item in (command, *originals))
        if tool == 2 and not primeline:
            layer = max(0, int(getattr(command, "layer", 0) or 0))
            entities.setdefault(layer, []).append(index)

    entries, by_command = [], {}
    for layer, prints in sorted(entities.items()):
        for parameter, (role, sign) in BOUNDARY_PARAMS.items():
            anchor = prints[0] if sign > 0 else prints[-1]
            scan = range(anchor - 1, -1, -1) if sign > 0 else range(anchor + 1, len(commands))
            selected = None
            for index in scan:
                if index in spatial or tools[index] != 2 or isinstance(commands[index], ToolChangeCommand):
                    break
                command = commands[index]
                if isinstance(command, ExtrudeWait) and command.delta_e * sign > 1e-9:
                    selected = index
                    break
            entry = {"layer": layer, "parameter": parameter, "role": role,
                     "available": selected is not None}
            if selected is not None:
                wait = commands[selected]
                duration = float(wait.wait_sec)
                speed = abs(float(wait.delta_e)) / duration if duration > 0 else float(wait.feedrate) / 60.0
                entry.update(
                    wait_layer=int(getattr(wait, "layer", 0) or 0),
                    source_line=int(wait.line), distance_mm=abs(float(wait.delta_e)),
                    speed_mm_s=speed,
                )
                by_command[selected] = entry
            entries.append(entry)
    return by_command, {"format": BOUNDARY_FORMAT, "entries": entries}
