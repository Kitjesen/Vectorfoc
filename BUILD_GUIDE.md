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

### Linux / macOS

```bash
# Configure
cmake -S . -B build \
  --toolchain cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build -j$(nproc)
```

### Windows (PowerShell, MinGW64)

```powershell
# Add MinGW64 to PATH for this session
$env:Path = "C:\msys64\mingw64\bin;" + $env:Path

# Configure (adjust ARM compiler path as needed)
cmake -S . -B build -G "MinGW Makefiles" `
  --toolchain cmake/gcc-arm-none-eabi.cmake `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY

# Build
cmake --build build -j8
```

### X-STAR-S Board

Add `-DBOARD_XSTAR=1` to any of the configure commands above.

### Clean rebuild

```bash
cmake --build build --clean-first
# or simply delete the build directory:
rm -rf build && cmake -S . -B build ...
```

## Host-Side Unit Tests (no hardware required)

```bash
cmake -S test -B build_test
cmake --build build_test -j$(nproc)
ctest --test-dir build_test -V
```

## Flashing

### First-time via SWD

```bash
# Flash bootloader at 0x08000000
st-flash write build_boot/VectorFoc_Bootloader.bin 0x08000000

# Flash application at 0x08004000
st-flash write build/VectorFoc.bin 0x08004000
```

Or use STM32CubeProgrammer GUI with the same addresses.

### OTA update via USB

After the bootloader is installed, subsequent updates can be done over USB:

```bash
python scripts/ota_upload.py build/VectorFoc.bin --port /dev/ttyUSB0
# Windows: --port COM3
```

## OTA Bootloader

See [docs/OTA_BOOTLOADER.md](docs/OTA_BOOTLOADER.md) for full details on the bootloader design and upgrade procedure.
