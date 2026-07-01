# Source NPZ Format

## Contract

The external source file is a standard `.npz` archive with one key per layer and material:

```text
meta
layer_0000_R
layer_0000_F
layer_0001_R
layer_0001_F
```

`R` means resin. `F` means fiber. The layer index describes print order and grouping. Every path point must contain source-side Z; the preprocessor does not generate or overwrite trajectory Z.

## Path Arrays

Each `layer_xxxx_R/F` value should be a numeric `float32` 3D array:

```text
[path_count, max_points_per_path, columns]
```

Shorter paths are padded with rows where every column is `NaN`. This keeps the file readable with `np.load(..., allow_pickle=False)` and avoids serialized Python objects.

Required point format:

```text
Nx3: [x, y, z]
```

Optional pose format:

```text
Nx6: [x, y, z, a, b, c]
```

Nx3 input is normalized to Nx6 by appending the configured default `a/b/c`. Nx2/Nx5 paths are invalid because the source file must carry Z.

## Z Ownership

Source Z is the geometry truth. This is required for curved-surface slicing where Z can vary within one layer path. UI layer-height values are process references only: resin layer height participates in resin extrusion calculation, and fiber layer height is retained as a fiber process parameter. Neither value changes the imported trajectory Z.

Shared head offsets and tool-change compensation are applied by `path_processing_core.npz_exporter` during system NPZ export.

## Meta

`meta` is optional and should be a JSON string:

```json
{
  "format": "external_layer_paths_v1",
  "unit": "mm",
  "point_columns": ["x", "y", "z"],
  "materials": {
    "R": "resin",
    "F": "fiber"
  }
}
```

## Template File

A two-layer template is generated under the project data directory:

```text
data/external_npz_preprocessor/source_npz_templates/two_layer_rf_template.npz
```

The preprocessor source-file dialog opens this folder by default. The file contains `layer_0000_R`, `layer_0000_F`, `layer_0001_R`, and `layer_0001_F`; each layer has both resin and fiber XYZ paths.
