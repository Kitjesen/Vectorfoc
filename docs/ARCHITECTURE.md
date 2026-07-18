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

## Encoder calibration persistence

`param_encoder_calibration` is a deep module for the encoder-calibration
portion of `FlashParamData`. Its public interface is only the portable
snapshot adapter registration; Flash field mapping and reserved-byte validation
remain inside the parameter module.

`EncoderCalibrationSettings_InstallAdapter()` runs before
`Param_SystemInitOnce()`. That preserves the existing early restore point that
Hall/ABZ initialization expects, while moving concrete encoder globals and
compile-time sensor branches out of `param_access.c`.

- Save captures `valid + offset_lut` from the registered adapter.
- A validated image restores that snapshot through the adapter.
- Invalid metadata is rejected before the adapter is called.
- Rollback to defaults clears the adapter state.

## Dependency direction

For this slice the allowed direction is:

```text
COMM / UI callers -> parameter module -> parameter runtime seam <- APP adapter
APP adapter -> motor, control, protocol, detection, encoder HAL

parameter module -> encoder-calibration module -> calibration seam <- APP adapter
APP calibration adapter -> Hall / ABZ / MT6816 / TMR3109 state

parameter table -> target-binding seam <- APP target adapter
APP target adapter -> motor_data / g_* configuration / DetectionConfig

App_Init -> communication-bootstrap module -> CAN transport / protocol / safety bridge
```

Do not add direct `motor.h`, `manager.h`, `control/control.h`,
`fault_detection.h`, `hal_encoder.h`, `config.h`, or concrete encoder headers
back to `param_access.c` or `param_table.c`.
