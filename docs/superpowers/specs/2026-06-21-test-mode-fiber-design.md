# Test Mode Fiber Printing Design

Date: 2026-06-21

## Scope

This design adds carbon fiber test printing to the existing test mode and updates the formal-print offset UI layout. Formal-print trajectory compensation is not implemented in this scope.

In scope:

- Test-mode resin Z height calibration with both existing Z jog buttons and direct signed numeric input.
- Test-mode fiber X/Y/Z calibration with signed numeric inputs and an apply action.
- Persistent head calibration file shared by test mode and the formal-print UI.
- Test-mode resin-only, fiber-only, and resin-plus-fiber composite test print actions.
- Test-mode scissor button that only sends a reserved UART command when the fiber head is active.
- Test-mode tool-change safety moves for generated test NPZ jobs.
- Formal-print UI re-layout for resin Z, fiber Z, and fiber X/Y offsets only.

Out of scope:

- Applying the new compensation model to formal-print GCode/NPZ export.
- Completing a partially printed resin matrix before fiber printing.
- Runtime insertion of compensation inside `center_node`, `rsi_node`, or `uart_node`.
- Final scissor UART protocol fields beyond a reserved command string.

## Existing System

The test mode is implemented mostly in `my_project_ui/ui_panel.py`. It currently generates temporary GCode and NPZ files under `/home/jayson/kuka_ram_ws/data/print_test`, publishes a reset command to `/print_test/rsi_command`, and publishes the NPZ path to `/print_test/load_npz`. `center_node` dynamically loads that NPZ and pre-fills the existing RSI trajectory queue.

Manual test-mode Z adjustment already uses the same temporary NPZ path. The UI observes `/rsi/current_correction` and treats a test action as complete when the current correction is close to the target pose.

The current resin test matrix generator lives in `gcode_planner/print_test_generator.py`. It creates a serpentine multi-line matrix with fixed default line length and Y spacing.

## Calibration Data

Calibration is stored in one current-state JSON file:

`/home/jayson/kuka_ram_ws/data/head_calibration_offsets/head_offsets.json`

The file is overwritten whenever the user starts a confirmed test print action or confirms a calibration step. It includes an update timestamp:

```json
{
  "updated_at": "2026-06-21T15:30:00+08:00",
  "resin": {
    "z_print_compensation_mm": -20.0
  },
  "fiber": {
    "x_print_compensation_mm": 5.0,
    "y_print_compensation_mm": 4.0,
    "z_print_compensation_mm": -25.0
  }
}
```

All values are signed and entered as-is. The UI does not invert signs.

The calibration values have two separate meanings:

- During calibration, each value is local to the current print head and is applied independently.
- During generated composite test jobs, relative compensation is computed from the confirmed values:
  - `R = (0, 0, resin_z_print_compensation_mm)`
  - `F = (fiber_x_print_compensation_mm, fiber_y_print_compensation_mm, fiber_z_print_compensation_mm)`
  - Resin to fiber compensation: `F - R`
  - Fiber to resin compensation: `R - F`

Example: resin Z is `-20`, fiber is `(5, 4, -25)`. Resin to fiber compensation is `(5, 4, -5)`.

## Test Mode UI

The current `打印测试（树脂）` area becomes a combined test-mode panel. It is organized into:

- Global test parameters:
  - Test speed.
  - Line length.
  - Y spacing.
  - Tool-change safe lift, default `10mm`.
- Resin parameters:
  - Resin target temperature.
  - Resin layer height range.
  - Resin extrusion scale range.
  - Resin prime length and speed.
  - Resin retract length and speed.
  - Resin Z print compensation input.
- Fiber parameters:
  - Fiber target temperature.
  - Fiber layer height range.
  - Fiber extrusion scale range.
  - Fiber prime length and speed.
  - Fiber retract length and speed.
  - Fiber X/Y/Z print compensation inputs.
- Actions:
  - `进入测试准备`
  - `确认树脂打印高度`
  - `继续调整纤维头`
  - `开始测试树脂打印`
  - `应用纤维偏置`
  - `确认纤维头偏置`
  - `直接打印纤维`
  - `复合打印`
  - `剪切`

The scissor button is separate from the calibration flow. It only checks the current tool before publishing a UART command.

## Test Mode Flow

### Preparation

When the user clicks `进入测试准备`:

1. The UI loads the current calibration JSON if it exists.
2. The UI sends the resin fan and resin heat commands using the resin target temperature.
3. The UI ensures the resin head is selected. If `PrintHeadStatus.current_tool == 2`, no tool switch is sent. Otherwise the UI must first generate a temporary travel NPZ back to RSI zero correction, wait for arrival, send `EV 0 tool_change_resin 2\n`, and wait until `PrintHeadStatus.current_tool == 2`.
4. The UI sends `RESET` to `/print_test/rsi_command`.
5. Resin Z controls become available after `/rsi/current_correction` has been received.

### Resin Height Calibration

The user can adjust resin Z by:

- Existing Z jog buttons.
- Direct signed resin Z input plus an apply action.

Both routes generate a temporary travel NPZ from the current correction to the requested target correction. When the user clicks `确认树脂打印高度`, the resin Z value is saved.

After resin height confirmation, two paths are available:

- `开始测试树脂打印`: generate and run only the resin test matrix.
- `继续调整纤维头`: move into fiber calibration.

### Fiber Calibration Entry

When the user clicks `继续调整纤维头`:

1. The UI warns that the system will return to the RSI zero correction before switching to the fiber head.
2. If the user confirms, the UI generates a temporary travel NPZ from the current correction to `(0, 0, 0, 0, 0, 0)`.
3. Only after arrival at zero correction, the UI sends `EV 0 tool_change_cf 1\n`.
4. The UI waits until `PrintHeadStatus.current_tool == 1`.
5. Fiber X/Y/Z inputs and the fiber apply/confirm actions become available.

This zero-return rule is required only for manual calibration entry, where the fiber offset may not be known yet.

### Fiber Offset Calibration

Fiber X/Y/Z inputs are signed values local to the fiber head. Clicking `应用纤维偏置` generates a temporary travel NPZ from the current correction to the entered fiber X/Y/Z target while keeping A/B/C unchanged or at the current correction values.

Clicking `确认纤维头偏置` saves the fiber X/Y/Z values to the calibration JSON. After confirmation:

- `直接打印纤维` is enabled.
- `复合打印` is enabled.

### Resin-Only Test Print

`开始测试树脂打印` generates a resin test matrix using the resin parameter set and the current resin correction. It preserves the existing behavior of the prior resin-only test logic.

### Fiber-Only Test Print

`直接打印纤维` generates a fiber test matrix using the fiber parameter set and the current fiber correction. It mirrors the resin-only matrix behavior:

- Same serpentine matrix structure.
- Same configurable line length and Y spacing.
- Fiber tool id `1`.
- Fiber target temperature and fiber extrusion parameters.

It does not print resin first.

### Composite Test Print

`复合打印` generates one temporary test job:

1. Resin matrix using resin parameters and resin tool id `2`.
2. Tool-change safety sequence from resin to fiber:
   - Insert safe lift of `tool_change_safe_lift_mm`, default `10mm`.
   - Insert compensation move `F - R`.
   - Insert `tool_change_cf`.
3. Fiber matrix using fiber parameters and fiber tool id `1`, with the same geometric matrix pattern as the resin matrix.

The composite job is generated as GCode and exported to NPZ before loading. No runtime queue mutation is added.

## Tool-Change Safety

For manual calibration entry:

- Return to RSI zero correction first.
- Then execute the tool-change UART event.
- Then wait for `PrintHeadStatus.current_tool` to match the requested tool.

For generated test print jobs after calibration:

- Do not return to zero before each tool change.
- Insert a safe lift of `10mm`.
- Insert the signed relative compensation move.
- Then emit the tool-change event.

The generated job approach keeps the real-time communication path unchanged.

## Scissor Button

`剪切` is an external test-mode button. It does not depend on calibration completion.

On click:

1. Check `PrintHeadStatus.current_tool == 1`.
2. If false, show a status error and send nothing.
3. If true, publish the reserved UART command `EV 0 cut_cf\n`.

The command string is intentionally isolated so the final protocol fields can be replaced later.

## Formal-Print UI Layout

Formal-print behavior is not changed in this scope. Only the offset layout changes:

- The old resin Z print compensation area becomes a Z compensation area with two inputs:
  - Resin Z print compensation.
  - Fiber Z print compensation, placed to the right of resin Z.
- The old tool offset area becomes fiber XY offset:
  - Fiber X print compensation.
  - Fiber Y print compensation.
- Existing wheel-event protection remains for these numeric offset controls.
- Values are loaded from and saved to `head_offsets.json`.

Formal-print export still uses the current existing offset behavior until a later implementation explicitly connects the new compensation model.

## Testing

Focused tests should cover:

- Calibration JSON load/save path, schema, signed values, and timestamp.
- Test-mode UI contains separate resin/fiber/global parameter groups and the requested buttons.
- Resin-only path still generates the existing resin matrix behavior.
- Fiber-only path generates a fiber matrix with tool id `1`.
- Composite path emits resin matrix, safe lift, compensation move `F - R`, `tool_change_cf`, and fiber matrix in order.
- Manual calibration entry requires zero correction before tool change and waits for current tool status.
- Scissor button refuses to send unless current tool is fiber.
- Formal-print UI layout exposes resin Z, fiber Z, fiber X, and fiber Y in the requested locations.

## Open Decisions

No open decisions remain for this implementation scope.
