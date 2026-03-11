# VectorFOC

![VectorFOC](fig/foc.png)

VectorFOC is a STM32G431-based field-oriented motor controller for robotics joints, servo drives, and compact actuator modules.

## Highlights

- 20 kHz current loop with Clarke, Park, inverse Park, and SVPWM.
- Control modes for open voltage, torque/current, velocity, position, ramped velocity, ramped position, and MIT impedance control.
- Multi-protocol communication stack with Inovxio private CAN, CANopen DS402 subset, and MIT-compatible control parsing.
- Calibration flow for current offset, Rs/Ls, flux, and encoder-related setup.
- Fault handling for voltage, current, temperature, stall, encoder, and communication errors.
- Parameter table with flash persistence and host-side read/write access.
- VOFA tooling support, including bus discovery support in the current runtime.

## Status

- Firmware build is supported through CMake and Keil MDK.
- Host-side tests are built from `test/` and executed through `ctest`.
- Current host test coverage includes math core, PID, LADRC, trajectory, integration, protocol parsing, protocol manager, executor, and VOFA bus scan helpers.

## Hardware Target

| Item | Value |
| --- | --- |
| MCU | STM32G431 |
| Core | Cortex-M4F @ 170 MHz |
| Encoder | MT6816 14-bit SPI magnetic encoder |
| Current loop | 20 kHz |
| CAN | Classical CAN |
| Typical bus rate | 1 Mbps |

## Build Firmware

```bash
git clone https://github.com/Kitjesen/Vectorfoc.git
cd Vectorfoc

cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build -j
```

Build artifacts are generated in `build/`:

- `VectorFoc.elf`
- `VectorFoc.bin`
- `VectorFoc.hex`

Keil users can also open `MDK-ARM/VectorFOC.uvprojx`.

## Run Host Tests

```bash
cmake -S test -B test/build-ci
cmake --build test/build-ci -j
ctest --test-dir test/build-ci --output-on-failure
```

The GitHub Actions workflow uses `ctest` instead of hard-coded runner names, so CI stays aligned with the actual test targets defined in `test/CMakeLists.txt`.

## Protocol Support

| Protocol | Current Support | Notes |
| --- | --- | --- |
| Inovxio private CAN | Main command and response path implemented | Includes motor control, enable/stop, zeroing, calibration, parameter read/write, save, version, fault query, report control, baudrate update, protocol switch, and bootloader request |
| Inovxio discovery | Implemented in `Src/COMM/manager/manager.c` | `GET_ID` discovery is handled by the protocol manager rather than `inovxio_protocol.c` |
| CANopen DS402 | Implemented subset | Supports NMT, RPDO1 position/velocity/torque path, selected SDO objects, TPDO1 feedback, EMCY, and heartbeat |
| MIT | Partial implementation | Current parser supports the 8-byte impedance control frame and the 0-byte RTR query path |

## Test Coverage Notes

- Inovxio protocol command parsing and frame builders are covered by host tests.
- CANopen and MIT parser/builder paths are covered by host tests.
- Protocol manager routing, including `GET_ID` handling, is covered by host tests.
- The MIT header still lists extra command enums beyond the parser implementation; those enums are not yet implemented as standalone command paths.
- Hardware validation is still required for real CAN bus timing, bootloader jump behavior, encoder chain validation, and protection recovery on target boards.

## Repository Layout

```text
Vectorfoc/
|-- Src/
|   |-- APP/
|   |-- ALGO/
|   |-- COMM/
|   |-- HAL/
|   `-- UI/
|-- Lib/
|-- MDK-ARM/
|-- cmake/
|-- fig/
`-- test/
```

## License

MIT License
