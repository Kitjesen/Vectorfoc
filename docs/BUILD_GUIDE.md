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

The suite builds and runs C regression executables covering algorithms, control modes,
CAN/task integration, startup safety, electrical calibration and parameter
compatibility, ADC/encoder guards, watchdog supervision and sensor health, plus tooling and CubeMX/PWM configuration checks.
Release and RelWithDebInfo retain test assertions. Configuration
also compiles one valid non-default timing setup and rejects 15 invalid setups
(zero, faster than the PWM base, or non-divisible frequencies).

The obsolete `test_runner_foc_state` smoke test and its unused mock/plant files
were removed. Startup and fault checks link the production protection, FSM, ADC
startup and PWM code in `test_runner_startup`; only peripheral I/O is mocked.

## 最新验证

2026-10-07，将整理分支 `da2b068` 与主线 `a756902` 合并。测试和工程引用已适配当前目录；纯算法继续独立编译。

| 检查 | 结果 |
| --- | --- |
| Release 主机回归 | 29 / 29 通过；Release 保留断言，包含 13 组真实启动/FSM/保护/PWM 链路检查 |
| 频率配置编译检查 | 1 组合法配置通过，15 组非法配置按预期拒绝 |
| MT6816 ARM 应用 | 构建、链接和镜像 CRC 校验通过；RAM 20,264 / 22,512 B，CCM 9,280 / 10,240 B，Flash 102,076 / 110,592 B |
| TMR3109 ARM 应用 | 构建、链接和镜像 CRC 校验通过；RAM 20,272 / 22,512 B，CCM 9,280 / 10,240 B，Flash 102,160 / 110,592 B |
| Bootloader | 构建、链接通过；Flash 13,704 / 16,384 B，原有超限已解决 |
| Keil 引用 | XML 中 115 个源文件路径存在；只核对引用，没有执行 Keil 编译 |
| Python 工具 | OTA 镜像头长度/CRC 回归及受影响脚本的 ruff 检查通过 |

STM32G431CB 的物理分区为 128 KiB Flash、22 KiB 普通 SRAM 和 10 KiB CCM；普通 SRAM 顶部 16 B 保留为启动标志。应用 Flash 为 108 KiB，Bootloader 为 16 KiB，参数为 4 KiB。旧记录中使用的连续 32 KiB RAM 和 224 KiB 应用 Flash 不适用于当前目标；本次采用主线修正后的链接与启动配置。

默认应用普通 RAM 剩余 2,248 B，CCM 剩余 960 B。增加缓冲区或任务栈前需要重新检查两块内存；静态链接用量不能代替运行时栈峰值和执行时序测量。

本次合并包括非有限值保护、前馈单位和参数边界修复，不沿用上一轮“与旧源码逐周期完全一致”的结论。门极电平、采样连续性、被动零偏、真实电机和 USB OTA 跳转仍需上板验证；本轮未烧录。GitHub CI 分别运行 Release 主机测试、两种位置传感器应用和 Bootloader 构建。

## Flashing

The bootloader now fits its 16 KiB region after integrating the upstream size and memory-layout fixes. Build success is a software check; validate the board and actual upgrade/jump path before flashing. This merge did not flash hardware.

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
