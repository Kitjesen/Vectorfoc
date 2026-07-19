# Architecture seams

## Runtime settings

`Src/UI/parameter/` owns parameter validation, persistence orchestration,
immutable metadata and Flash images. It does not own concrete runtime RAM
targets or apply PID, protocol, CAN watchdog or encoder-offset effects
directly.

The only runtime seam is:

```c
void Param_SetRuntimeApplyCallback(ParamRuntimeApplyCallback callback,
                                   void *context);
void Param_ApplyRuntimeState(void);
```

- A successful single write publishes its `ParamIndex` after leaving the
  parameter critical section.
- A Flash restore, default restore or explicit replay publishes one
  `PARAM_RUNTIME_APPLY_ALL` notification.
- The callback must not recursively write parameters.

`Src/APP/settings/runtime_settings.c` is the production adapter. `App_Init`
installs it only after the encoder, protocol and motor runtime are ready, then
replays all settings. The early encoder-offset restore also goes through this
module so the hardware action has one owner.

Host tests provide the second adapter: a recording callback for the parameter
module and a fake parameter registration point for `RuntimeSettings`. This
makes the seam real rather than test-only.

## Parameter target binding

`param_table.c` owns immutable parameter metadata only.  It has no direct
dependency on motor state, safety configuration, board configuration or
encoder headers.  `ParamTable_SetBindingAdapter()` validates that an APP
adapter resolves every entry to the matching type, a non-null RAM target and a
valid default before that adapter becomes active.

`Src/APP/settings/parameter_bindings_settings.c` is the production adapter.
It owns the concrete `motor_data`, legacy `g_*` configuration fields and the
`Detection_GetConfig()` target lookup.  `App_Init` installs it after
`Detection_Init(NULL)` and before `Param_SystemInitOnce()`.  Binding has no
runtime side effect: early Flash recovery still changes RAM only, and the
existing late runtime replay remains the sole place that applies PID, protocol
or hardware behavior.

## Communication bootstrap

`App_Init` remains the owner of whole-device startup sequencing, but
`Src/APP/init/app_comm_bootstrap.c` owns the communication composition that
must stay contiguous: BSP CAN initialization, transport initialization,
transport registration, protocol initialization, then safety fault-callback
registration.

It sanitizes a persisted protocol value before protocol initialization and
bridges safety fault reports to the protocol manager. It runs after early
parameter recovery (so persisted CAN and protocol settings are visible) and
before runtime settings are replayed. The bootstrap module does not move CAN
or Flash work between tasks; it only makes this startup boundary explicit.

## PositionSensor boundary

`Src/HAL/position_sensor/` is the hardware boundary for position feedback.
The public API is a small set of typed functions in `position_sensor.h`:

- `PositionSensor_UpdateAndRead()` publishes one coherent 20 kHz sample:
  multi-turn position, wrapped mechanical angle, velocity, electrical angle
  and native raw count. Publication and task-side copies use a short platform
  critical section, so readers cannot observe a partially assigned frame.
- `PositionSensor_GetHealth()` reports valid/diagnostic/failure counters
  without exposing concrete driver status enums.
- `PositionSensor_CaptureCalibration()`,
  `PositionSensor_RestoreCalibration()` and
  `PositionSensor_ClearCalibration()` own portable calibration persistence.
- `PositionSensor_RawCalibration*()` exposes direction, pole-pair and LUT
  calibration only when the selected adapter reports the matching capability.

The concrete adapter table is private in `position_sensor_internal.h`.
Public callers do not receive a vtable or a concrete encoder pointer.
`position_sensor_selection.c` is the only place that maps
`HW_POSITION_SENSOR_MODE` to MT6816, TMR3109, Hall or ABZ. The compatibility
adapter `position_sensor_motor_hal.c` keeps the existing
`Motor_HAL_EncoderInterface_t` shape for code that has not moved to the new
API yet.

Current rule: ALGO, APP and SAFE may depend on `position_sensor.h`, but must
not include concrete encoder headers, concrete encoder globals or
`HW_POSITION_SENSOR_MODE` branches. `test_position_sensor_architecture`
enforces this rule so adding another sensor does not require edits in control,
calibration persistence, fault detection or startup readiness code.

Adding a sensor should be local:

1. Add the low-level driver.
2. Add one `position_sensor_<name>.c` adapter that implements the private
   runtime, calibration and raw-calibration ops it actually supports.
3. Register the adapter in `position_sensor_internal.h` and
   `position_sensor_selection.c`.
4. Extend board/CMake validation for the legal board + sensor combination.
5. Add selection/runtime tests and keep `test_position_sensor_architecture`
   green.

## Encoder calibration persistence

`param_encoder_calibration` is a deep module for the encoder-calibration
portion of `FlashParamData`. Its public interface is only the portable
snapshot adapter registration; Flash field mapping and reserved-byte validation
remain inside the parameter module.

`EncoderCalibrationSettings_InstallAdapter()` runs before
the selected sensor is initialized, and `Param_SystemInitOnce()` runs only
after that initialization succeeds. The production adapter delegates to
`PositionSensor_CaptureCalibration()`, `PositionSensor_RestoreCalibration()`
and `PositionSensor_ClearCalibration()`. Consequently, a validated Flash
snapshot is restored directly into initialized driver state; the module does
not keep a second pending LUT copy in SRAM.

- Save captures `valid + offset_lut` from the registered adapter.
- A validated image restores that snapshot through the adapter.
- Invalid metadata is rejected before the adapter is called.
- Rollback to defaults clears the adapter state.

This fixes the startup-order trap where restoring calibration before concrete
driver initialization allowed the driver's default state to overwrite it.

## MCU replacement seam status

The position-sensor boundary is complete for the current four supported
sensors. The MCU replacement boundary is not complete yet.

Known STM32G4 coupling that remains by design in this slice:

- ADC injected callbacks and PWM timing are still APP/HAL specific.
- CAN, USB CDC, Flash, reset and Bootloader entry still depend on STM32G4 BSP
  services.
- `board_config.h` still selects board-level compile-time features.
- Several communication/UI paths still call platform HAL services directly.

The next architecture slice should introduce an MCU kit/platform-service
boundary for clocks, GPIO, ADC/PWM ISR entry, transport setup, Flash, timebase,
reset and Bootloader handoff. Keep the 20 kHz path static and measured; do not
add a dynamic dispatch layer to the ISR without size and timing evidence.

## Dependency direction

For this slice the allowed direction is:

```text
COMM / UI callers -> parameter module -> parameter runtime seam <- APP adapter
APP adapter -> motor, control, protocol, detection, encoder HAL

parameter module -> encoder-calibration module -> calibration seam <- APP adapter
APP calibration adapter -> PositionSensor module -> selected concrete adapter

parameter table -> target-binding seam <- APP target adapter
APP target adapter -> motor_data / g_* configuration / DetectionConfig

App_Init -> communication-bootstrap module -> CAN transport / protocol / safety bridge

ALGO calibration / SAFE fault checks -> PositionSensor public API
PositionSensor compatibility adapter -> legacy Motor_HAL_EncoderInterface_t
```

Do not add direct `motor.h`, `manager.h`, `control/control.h`,
`fault_detection.h`, `hal_encoder.h`, `config.h`, or concrete encoder headers
back to `param_access.c` or `param_table.c`.

Do not add concrete encoder headers or `HW_POSITION_SENSOR_MODE` branches to
`Src/ALGO`, `Src/APP` or `Src/SAFE`.
