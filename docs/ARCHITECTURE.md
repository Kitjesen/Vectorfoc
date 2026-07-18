# Architecture seams

## Runtime settings

`Src/UI/parameter/` owns parameter validation, RAM values and Flash images.
It does not apply PID, protocol, CAN watchdog or encoder-offset effects
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

## Dependency direction

For this slice the allowed direction is:

```text
COMM / UI callers -> parameter module -> parameter runtime seam <- APP adapter
APP adapter -> motor, control, protocol, detection, encoder HAL
```

Do not add direct `motor.h`, `manager.h`, `control/control.h`,
`fault_detection.h` or `hal_encoder.h` includes back to `param_access.c`.
