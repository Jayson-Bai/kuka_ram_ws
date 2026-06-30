# Parameter Model

## Principle

The source NPZ provides geometry only. Extrusion is derived from material process parameters during conversion.

## Current Defaults

- Resin line width: fixed `2.0 mm` and not user configurable
- Resin layer height: `0.5 mm`
- Resin extrusion scale: `1.0`
- Resin print speed: `10 mm/s`
- Resin temperature: `250 C`
- Resin prime: `18 mm @ 15 mm/s`
- Resin retract: `15 mm @ 30 mm/s`
- Fiber layer height: `0.1 mm`
- Fiber extrusion scale: `1.0`
- Fiber print speed: `10 mm/s`
- Fiber temperature: `250 C`
- Fiber prime: `12 mm @ 5 mm/s`
- Fiber retract: `10 mm @ 5 mm/s`
- Resin and fiber fans: always on by default
- Travel speed: `10 mm/s`
- Default ABC: `0, 0, 0`

## Z Handling

The source NPZ must contain Z in every path point. The preprocessor treats source Z as trajectory geometry and never replaces it with UI layer-height values. This keeps curved-surface slicing intact.

UI layer-height fields are process references:

- Resin layer height participates in resin extrusion calculation.
- Fiber layer height is retained as a fiber process parameter, but it does not change trajectory Z or fiber `E` in the current speed-matched model.

Shared head offsets and tool-switch compensation are not duplicated here; they remain in `npz_exporter` through `tool_offset` and `resin_z_print_compensation_mm`.

## Resin

Resin equivalent extrusion per millimeter is derived as:

```text
resin_e_per_mm = 2.0 * layer_height_mm * extrusion_scale
```

The old pump-volume-to-E ratio is intentionally removed from the UI and CLI. Resin line width is also fixed in code at `2.0 mm`, so the user-facing controls are `layer_height_mm` and `extrusion_scale`.

## Fiber

Fiber does not expose diameter, printed line width, or a direct `E/mm` field. The active model forces fiber feed to match TCP movement speed by default:

```text
fiber_e_per_mm = fiber_extrusion_scale
```

With the default `fiber_extrusion_scale = 1.0`, a 10 mm path produces 10 mm of fiber feed. Fiber layer height remains in the process parameters as a process reference, but it does not change source Z or `E` in the current speed-matched fiber model.

## Shared Parameters

- `travel_feed_mm_s`: feed speed for non-print travel moves.
- `default_a/default_b/default_c`: pose values appended to Nx3 source paths.
- `dt`: sample period forwarded to `npz_exporter`.

## Prime and Retract

Prime/retract defaults mirror the existing test-mode UI:

- Resin: prime before each resin path, retract after each resin path.
- Fiber: retract before each fiber path, then prime before each fiber path. This follows the existing fiber matrix behavior where fiber extrusion is path-length based.

The converter represents these as `ExtrudeWait` commands so `npz_exporter` remains the only system NPZ writer.

## Persistent Parameter JSON

The UI has a `保存打印参数json文件` button. It writes the current print parameters under the project data directory, matching the existing test/formal-mode convention:

```text
data/external_npz_preprocessor/print_params.json
```

The directory is created automatically before saving. When the UI starts, it automatically reads this file if it exists and applies the latest saved print parameters. Resin line width is not stored as a parameter; legacy JSON files containing `bead_width_mm` are accepted but that value is ignored.

## Shared Head Offsets

External NPZ conversion uses the same head-offset data source as test mode and formal printing:

```text
data/head_calibration_offsets/head_offsets.json
```

The converter reads this through `gcode_planner.head_calibration.load_head_calibration()`. During export it mirrors the formal-print path:

```text
tool_offset = (fiber_x_print_compensation_mm,
               fiber_y_print_compensation_mm,
               fiber_z_print_compensation_mm)
resin_z_print_compensation_mm = resin.z_print_compensation_mm
```

These values are passed directly to `npz_exporter.export_npz()`, so tool switching and resin-Z compensation stay centralized in the existing exporter logic.
