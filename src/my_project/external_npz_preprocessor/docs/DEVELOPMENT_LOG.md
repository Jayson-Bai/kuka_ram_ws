# Development Log

## 2026-06-29 Initial Framework

- Created `external_npz_preprocessor` as a standalone `ament_python` package under `src/my_project`.
- Added source NPZ loading, process parameters, conversion to `path_processing_core.types`, CLI, Qt UI, and tests.
- Kept system NPZ writing centralized through `path_processing_core.npz_exporter.export_npz()`.

## 2026-06-29 Parameter Defaults and Persistence

- Added source/output path selectors and default output path handling under `data/output_npz`.
- Added resin/fiber process parameters, prime/retract defaults from test mode, and JSON persistence under `data/external_npz_preprocessor/print_params.json`.
- Removed resin pump-volume-to-E ratio and fiber direct `E/mm`; resin uses extrusion scale and fiber feed is path-length matched by default.
- Fixed resin line width in code at `2.0 mm`; it is not user configurable.

## 2026-06-29 Shared Head Offset Integration

- External NPZ conversion reads the same head-offset data source as test mode and formal print: `data/head_calibration_offsets/head_offsets.json`.
- Conversion passes `tool_offset=(fiber_x, fiber_y, fiber_z)` and `resin_z_print_compensation_mm` into `npz_exporter.export_npz()`.
- UI displays the offset source and values used by conversion.

## 2026-06-29 Template and Output Defaults

- Added a generated two-layer external source NPZ template.
- Added default source template directory under `data/external_npz_preprocessor/source_npz_templates`.
- Empty output path resolves to `data/output_npz/<source_stem>/<source_stem>.npz`, matching the formal-print named-output convention.

## 2026-06-30 Required Source Z

- Finalized the source NPZ contract as Z-bearing paths only: Nx3 `[x, y, z]` or Nx6 `[x, y, z, a, b, c]`.
- Removed preprocessor-generated Z and the `忽略纤维高度累计` UI/CLI parameter.
- Re-scoped resin/fiber layer-height UI values as process/extrusion references, not trajectory-height generators.
- Updated the generated template to numeric XYZ arrays so it remains readable with `allow_pickle=False`.


## 2026-07-01 Core Extraction and Dual Input

- Moved shared exporter, command types, head calibration, B-spline fitting, polynomial interpolation, and B-spline base modules into `path_processing_core`.
- Kept `gcode_planner` as the GCode input adapter, preview helper owner, and legacy import compatibility package.
- Kept `external_npz_preprocessor` as the agreed source NPZ adapter; it converts source NPZ paths into `path_processing_core.types` commands and exports through `path_processing_core.npz_exporter`.
- Formal-print compatible input now supports `.gcode/.gc/.g` through `gcode_planner` and agreed `.npz` through `external_npz_preprocessor`; both paths converge in `path_processing_core`.

## 2026-07-01 Event Ordering Update

- Source NPZ conversion now starts with both-head fan and heat events.
- ResetE is inserted immediately after tool changes, matching the established GCode placement.
- This update temporarily used paired retract/prime waits around print paths; the current rule is documented in the 2026-07-10 update below.
- Shared head-offset behavior was left unchanged.

## 2026-07-10 Prime/Retract Ordering Update

- External NPZ conversion inserts one initial retract before the whole part's first print path, then primes before printing.
- Later resin/fiber paths insert only prime immediately before printing; resin paths insert only retract after printing.
- Fiber paths emit `CUT`, and the exporter performs the cut lift feed plus equal safety retract.
- Travel segments do not carry prime or retract waits; preparation waits run after travel reaches the next print start pose.

Verification:

```bash
python3 -m pytest src/my_project/external_npz_preprocessor/test -q
python3 -m py_compile src/my_project/external_npz_preprocessor/external_npz_preprocessor/*.py
colcon build --packages-select external_npz_preprocessor
```
