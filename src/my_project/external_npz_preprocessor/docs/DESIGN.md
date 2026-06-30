# External NPZ Preprocessor Design

## Goal

This package converts an external geometry-only NPZ into the system NPZ format consumed by `control_center`, while always going through `gcode_planner.npz_exporter.export_npz()`.

## Package Boundary

- `source_npz.py`: reads and validates the external source NPZ contract.
- `process_params.py`: defines material process parameters and derives extrusion.
- `converter.py`: converts layer/material paths into `gcode_planner.types` parsed commands.
- `export_runner.py`: connects reader, converter, and `npz_exporter`.
- `cli.py`: batch conversion entry point.
- `ui.py` / `app.py`: simple desktop UI for selecting paths and parameters.

The package is intentionally separate from `gcode_planner` and `my_project_ui`. It depends on `gcode_planner`, but does not change the system NPZ exporter.

## Data Flow

```text
external source NPZ
-> load_source_npz()
-> SourceJob / LayerPaths / MaterialPath
-> source_job_to_parsed_commands()
-> MoveCommand / ToolChangeCommand / ResetECommand / MCommand
-> gcode_planner.npz_exporter.export_npz()
-> system-compatible NPZ
```

## Current Scope

The package supports:

- `layer_0000_R` / `layer_0000_F` source keys.
- Per-key numeric `float32` arrays shaped `[path_count, max_points, columns]`; short paths use all-NaN padding rows.
- Nx3 `[x, y, z]` source paths as the required base format.
- Nx6 `[x, y, z, a, b, c]` source paths when the source needs explicit pose values.
- Source-side Z is trajectory geometry and is never overwritten by UI layer-height values.
- Resin extrusion derived from fixed `2.0 mm` resin line width, resin layer height, and resin extrusion scale.
- Fiber extrusion matched to path length by default through fiber extrusion scale.
- A basic Qt UI and CLI.

