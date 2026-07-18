# VectorFOC

VectorFOC 是面向 STM32G431 电机控制板的开源 FOC 固件。项目包含 20 kHz 电流环、速度/位置控制、DS402 状态机、CAN/USB 通信、参数存储、Bootloader OTA 升级流程，以及可在主机运行的算法与安全回归测试。

[![CI](https://github.com/kitjesen/vectorfoc/actions/workflows/vectorfoc-ci.yml/badge.svg)](https://github.com/kitjesen/vectorfoc/actions/workflows/vectorfoc-ci.yml)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

![FOC Architecture](fig/foc.png)

## 当前能力

- 20 kHz FOC 电流环：Clarke/Park、SVPWM、d/q 轴 PI、限幅与异常输入保护。
- 控制模式：力矩、速度、位置、轨迹、开环、MIT 阻抗接口。
- 状态机：DS402 风格状态切换，空闲/故障态保持去使能，运行/标定态显式上电。
- 位置传感器：VectorFOC 板支持 MT6816、TMR3109；X-STAR-S 支持 Hall、ABZ。
- 通信：CAN 上的 Vector/Inovxio、MIT、CANopen 风格帧；USB CDC 调试与 VOFA+/VectorStudio 命令。
- 参数与安全边界：Flash 参数页带 CRC/提交标记；过压、欠压、过流、过温、堵转、CAN 超时、ADC/编码器异常等保护逻辑已有主机测试覆盖。
- OTA：VectorFOC 默认应用镜像带 App Header 和 CRC32，配合 16 KiB USB Bootloader 升级。

## 支持硬件

| 板型 | MCU | 位置传感器 | 镜像布局 | 说明 |
|---|---|---|---|---|
| VectorFOC G431 | STM32G431CBU6 | `MT6816`, `TMR3109` | Bootloader `0x08000000`，App `0x08004000` | 默认板型；App 构建会写入 Header/CRC |
| X-STAR-S | STM32G431RBT6 | `HALL`, `ABZ` | 独立 App `0x08000000` | `-DBOARD_XSTAR=ON`；当前不使用 VectorFOC USB Bootloader |

CMake 会拒绝不兼容组合：VectorFOC 仅接受 `AUTO`、`MT6816`、`TMR3109`；X-STAR-S 仅接受 `AUTO`、`HALL`、`ABZ`。

## 快速开始

### 工具

- CMake 3.20+（Bootloader/App），主机测试最低 3.10。
- xPack GNU Arm Embedded GCC。本仓库已用 `arm-none-eabi-gcc 13.3.1` 验证固件构建。
- Ninja 或 Make。
- Python 3.8+。OTA 上传需要 `pyserial`，本轮环境已检测到 `pyserial 3.5`。
- 首次烧录需要 SWD 工具，例如 STM32CubeProgrammer 或 `st-flash`。

### 构建 VectorFOC App

```bash
cmake -S . -B build-vector -G Ninja \
  --toolchain cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DPOSITION_SENSOR=MT6816
cmake --build build-vector --parallel
```

本轮验证会构建 VectorFOC/MT6816，并校验生成的 `build-vector/VectorFoc.bin` App Header。

### 构建 Bootloader

```bash
cmake -S . -B build-boot -G Ninja \
  --toolchain cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOTLOADER_BUILD=ON
cmake --build build-boot --parallel
```

验证结果：Bootloader 构建通过，生成 `build-boot/VectorFoc_Bootloader.bin`；当前 Flash 占用 13,756 B，低于 16 KiB Bootloader 分区。

### 构建 X-STAR-S

```bash
cmake -S . -B build-xstar-hall -G Ninja \
  --toolchain cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOARD_XSTAR=ON \
  -DPOSITION_SENSOR=HALL
cmake --build build-xstar-hall --parallel
```

验证结果：X-STAR-S/HALL 独立 App 构建通过，生成 `build-xstar-hall/VectorFoc.bin`。

### 运行主机测试

```bash
cmake -S test -B build-test
cmake --build build-test --parallel
ctest --test-dir build-test --output-on-failure
```

当前 CTest 注册 36 个自动测试，包含算法、通信、参数存储、Bootloader 协议、App Header 工具、安全保护、ADC/编码器保护，以及 VectorFOC/X-STAR-S 通信分支和 `test_runner_integration` 闭环仿真测试。若使用 Visual Studio 多配置生成器运行测试，需要加配置参数，例如：

```powershell
cmake --build build-test --config Debug --parallel
ctest --test-dir build-test -C Debug --output-on-failure
```

本轮将以重新配置后的构建目录执行完整 CTest；具体结果以提交前的验证记录为准。

### 运行闭环仿真测试

```bash
ctest --test-dir build-test -R test_runner_integration --output-on-failure
```

`test_runner_integration` 会生成 `foc_setpoint_switch.csv`，可用分析脚本绘制响应曲线：

```bash
python ../test/analyze_results.py foc_setpoint_switch.csv
```

本轮在新建主机构建目录中复跑该测试通过；分析脚本输出最终速度 19.75 rad/s、目标 20.0 rad/s、误差 0.2450 rad/s，并生成 `foc_setpoint_switch.png`。这些数值是当前实现的一次核验结果，不是固定的公开 API 或跨平台指标承诺。

CSV 和 PNG 是构建目录中的可再生输出，未提交到版本库；提交中保留了命令、分析脚本和本次结果，重新运行上述命令即可复现。

## 烧录与 OTA

首次部署 VectorFOC 板需要通过 SWD 烧录 Bootloader 和 App：

```bash
st-flash write build-boot/VectorFoc_Bootloader.bin 0x08000000
st-flash write build-vector/VectorFoc.bin 0x08004000
```

安装 Bootloader 后，可通过 USB CDC 执行 OTA：

```bash
python scripts/ota_upload.py build-vector/VectorFoc.bin --port COM3
```

Linux/macOS 串口通常为 `/dev/ttyUSB0`、`/dev/ttyACM0` 或同类设备名。

本轮只验证到固件镜像生成和 OTA 脚本依赖；`st-flash` 未在当前环境中发现，SWD 烧录和真实 USB OTA 需要连接硬件后再验证。

## OTA 与安全限制

当前 Bootloader/OTA 只提供镜像 Header、大小、保留字段和 CRC32 完整性校验；它不验证数字签名，不认证上传端，也不提供 secure boot。不要把它作为安全 OTA 使用。

生产部署建议：

- 只在可信物理环境或受控 USB 通道中使用当前 OTA。
- 不要把 USB CDC 或 CAN 控制面暴露给不可信用户或网络。
- 需要安全 OTA 时，应增加签名 manifest、公钥验签、版本/回滚策略，并结合 STM32 RDP/WRP 等读写保护；这些能力当前尚未实现。
- CAN 命令当前无总线级认证；同一 CAN 网络上的节点可以发送控制帧。
- 不需要现场调试时，生产固件应关闭或限制 USB CDC 参数写入能力。

## 项目结构

```text
VectorFOC/
├── Src/
│   ├── ALGO/           # FOC、控制器、轨迹、标定、状态机
│   ├── APP/            # 初始化、RTOS 任务、ISR 保护
│   ├── BOOT/           # USB Bootloader、Flash 操作、OTA 协议
│   ├── COMM/           # CAN/USB 通信协议与命令执行
│   ├── HAL/            # STM32G4 外设、编码器、板级抽象
│   ├── SAFE/           # 安全保护与看门狗监督
│   ├── UI/             # 参数表、VOFA+/VectorStudio、LED、错误上报
│   └── config/         # 板型与传感器配置
├── Lib/                # STM32 HAL、CMSIS、FreeRTOS、链接脚本
├── test/               # 主机单元测试与仿真测试
├── scripts/            # 构建、OTA、App Header 工具
├── docs/               # 设计、Bootloader、项目说明
└── cmake/              # ARM 工具链文件
```

更多说明见 [docs/PROJECT_OVERVIEW.md](docs/PROJECT_OVERVIEW.md)、[docs/SIMULATION.md](docs/SIMULATION.md)、[docs/OTA_BOOTLOADER.md](docs/OTA_BOOTLOADER.md) 和 [BUILD_GUIDE.md](BUILD_GUIDE.md)。

## 许可证

Copyright 2024-2026 VectorFOC Contributors.
Licensed under the [Apache License, Version 2.0](LICENSE).
