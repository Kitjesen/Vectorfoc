# Build Guide

## Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| [xpack arm-none-eabi-gcc](https://xpack.github.io/arm-none-eabi-gcc/) | ≥ 12.x | Cross-compiler for STM32 |
| [CMake](https://cmake.org/download/) | ≥ 3.22 | Build system |
| make / ninja | any | Build backend |
| [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) | any | For SWD flashing |
| Python 3.x | ≥ 3.8 | For OTA upload and utility scripts |

## Firmware Build

Run the commands below from the `firmware/` directory. Keep generated files
under `build/`: ARM firmware in `build/arm`, host tests in `build/host`, and
separately configured bootloader builds in `build/boot`.

First-party C sources are explicitly listed by module in the root
`CMakeLists.txt`; register new files there. Tests use `tests/CMakeLists.txt`.
The editor fallback include paths match the current module directories and do
not recursively search archived builds or test mocks. If you configure a
compilation database, regenerate it after moving source files.

### Linux / macOS

```bash
# Configure
cmake -S . -B build/arm \
  --toolchain cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build/arm --parallel 4
```

### Windows (PowerShell, MinGW64)

```powershell
# Local installation examples; adjust both paths for your machine.
$env:Path = "D:\msys64\mingw64\bin;D:\Program Files\armcc\xpack-arm-none-eabi-gcc\bin;" + $env:Path

# Configure using the same generator as the local migration verification.
cmake -S . -B build/arm -G Ninja `
  --toolchain cmake/gcc-arm-none-eabi.cmake `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY

# Build
cmake --build build/arm --parallel 4
```

### Clean rebuild

```bash
cmake --build build/arm --clean-first --parallel 4
```

When changing generator or toolchain, configure a fresh directory under
`build/`, such as `build/arm-release`, instead of reusing an incompatible cache.

## Host-Side Unit Tests (no hardware required)

### Linux / macOS

```bash
cmake -S tests -B build/host -G Ninja
cmake --build build/host --parallel 4
ctest --test-dir build/host --output-on-failure
```

### Windows (PowerShell, MinGW64)

```powershell
# Local MinGW64 installation; adjust the path for your machine.
$env:Path = "D:\msys64\mingw64\bin;" + $env:Path
cmake -S tests -B build/host -G Ninja -DCMAKE_C_COMPILER=D:/msys64/mingw64/bin/gcc.exe
cmake --build build/host --parallel 4
ctest --test-dir build/host --output-on-failure
```

The suite builds and runs 18 executables covering algorithms, control modes,
CAN/task integration, startup safety, electrical calibration and parameter
compatibility, plus a CubeMX/board PWM configuration check (19 CTest entries).
Release and RelWithDebInfo retain test assertions. Configuration
also compiles one valid non-default timing setup and rejects 15 invalid setups
(zero, faster than the PWM base, or non-divisible frequencies).

The obsolete `test_runner_foc_state` smoke test and its unused mock/plant files
were removed. Startup and fault checks link the production protection, FSM, ADC
startup and PWM code in `test_runner_startup`; only peripheral I/O is mocked.

## 最新验证

2026-10-07，本轮整理基于本地 `4520d1c`，发布到 `codex/vector-control-cleanup`。

| 检查 | 结果 |
| --- | --- |
| Release 主机测试 | 19 / 19 通过，测试断言保持开启 |
| 频率配置编译检查 | 1 组合合法配置通过，15 组非法配置按预期被拒绝 |
| 逐周期对照 | 默认控制、`TORQUE_AND_CURRENT`、`TORQUE_ADJUST`、前馈、R/L 标定共 5 组输出完全一致；对照本轮开始时保存的源码 |
| 默认传感器 MT6816 ARM 固件 | 构建通过；RAM 28,400 B / 32,752 B，Flash 97,284 B / 224 KiB |
| TMR3109 ARM 固件 | 构建通过；RAM 28,408 B / 32,752 B，Flash 97,468 B / 224 KiB |
| Keil 引用 | XML 中 100 个源文件路径存在，未执行 Keil 编译 |
| Bootloader | 编译完成、链接失败；20,032 B 超过 16 KiB，仍超出 3,648 B |

本轮删除未使用的 CMSIS-DSP 预编译库和头文件（3,035,549 字节），
并同步移除 Keil 引用。纯算法仍由 CMake 独立构建；构建结果不表示已完成硬件验证。

## Flashing

The bootloader overflow recorded on 2026-09-27 was reproduced on 2026-10-07:
20032 B exceeds
its 16 KiB Flash region by 3648 B. The paths below describe the intended outputs;
there is no validated bootloader image from this check. Resolve that blocker
and confirm the board, linker layout, and hardware conditions before flashing.

### First-time via SWD

These examples use the default application layout in
`platform/stm32g431xx_app.ld`, whose application origin is `0x08004000`.
The linker layout and address must match the hardware adapter and bootloader
that are actually installed.

```bash
# Flash bootloader at 0x08000000
st-flash write build/boot/VectorFoc_Bootloader.bin 0x08000000

# Flash application at 0x08004000
st-flash write build/arm/VectorFoc.bin 0x08004000
```

Or use STM32CubeProgrammer GUI with the same addresses.

### OTA update via USB

After the bootloader is installed, subsequent updates can be done over USB:

```bash
python scripts/ota_upload.py build/arm/VectorFoc.bin --port /dev/ttyUSB0
# Windows: --port COM3
```

## OTA Bootloader

See [OTA_BOOTLOADER.md](OTA_BOOTLOADER.md) for the separate `cmake/bootloader/` entry and the existing bootloader design. Current validation limits are recorded in [Vector debugging notes](../../docs/DEBUGGING.md).
