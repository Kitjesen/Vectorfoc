# Control trace support

`control_probe.c` includes `src/foc/control_dispatcher.c` and exposes a
read-only snapshot of its private scheduling state. The control-mode test uses
that snapshot to record deterministic per-cycle state and PWM writes without
adding test APIs to the firmware.

`run_differential.py` compares the current tree with a saved source ZIP. Both
trees must use the current shallow layout:

```text
algorithm/
src/
tests/
```

The runner extracts only those directories, configures each tree through its
own `tests/CMakeLists.txt`, builds the selected test executables, runs their
`--trace` mode, and compares the output byte for byte. It supports GCC and
Clang on Windows and Linux and requires CMake and Ninja.

From any working directory:

```text
python <firmware>/tests/control_support/run_differential.py --baseline <source-before.zip> --compiler clang --build-dir <firmware>/build/control-differential --suite all
```

`--compiler` accepts `GCC`, `Clang`, a compiler command, or an executable path.
`--suite control` compares the control dispatcher in its default,
`TORQUE_AND_CURRENT=1`, and `TORQUE_ADJUST=1` variants plus feedforward/cogging.
`--suite calibration` compares electrical R/L calibration. `--suite all` runs
both groups.

The build directory contains `validation.json`, `validation.log`, the extracted
baseline, isolated CMake build trees, and full trace files. The console prints
only one summary line per variant; trace contents stay in the build directory.

An exact match proves that the recorded host inputs produced identical named
float bits, integer state, and hardware-boundary events. It does not validate
interrupt latency, ADC timing, physical gate levels, or behavior on a powered
motor.
