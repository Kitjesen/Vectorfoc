# VectorFOC

VectorFOC 是面向 STM32G431 电机控制板的开源 FOC 固件。项目包含 20 kHz 电流环、速度/位置控制、DS402 状态机、CAN/USB 通信、参数存储、Bootloader OTA 升级流程，以及可在主机运行的算法、安全和控制链路回归测试。

[![CI](https://github.com/kitjesen/vectorfoc/actions/workflows/vectorfoc-ci.yml/badge.svg)](https://github.com/kitjesen/vectorfoc/actions/workflows/vectorfoc-ci.yml)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

![FOC Architecture](fig/foc.png)

## 当前能力

- 20 kHz FOC 电流环：Clarke/Park、SVPWM、d/q 轴 PI、限幅与异常输入保护。
- 控制模式：力矩、速度、位置、轨迹、开环、MIT 阻抗接口。
- 状态机：DS402 风格状态切换，空闲/故障态保持去使能，运行/标定态显式上电。
- 启动保护：ADC 注入转换虽可在外设启动后立即触发，但 FOC ISR 会在电机、传感器和安全对象全部初始化完成前保持惰性，避免启动窗口访问半初始化状态。
- 位置传感器：VectorFOC 板支持 MT6816、TMR3109；X-STAR-S 支持 Hall、ABZ。
- 通信：CAN 上的 Vector/Inovxio、MIT、CANopen 风格帧；USB CDC 调试与 VOFA+/VectorStudio 命令。CAN 的 `RESET`/`BOOTLOADER` ACK 必须由 FDCAN Tx event 确认完成后才执行动作；等待期间功率桥保持去使能，超时会取消待发请求、报告通信超时且不复位。
- 参数与安全边界：Flash 参数页带 CRC/提交标记；显式保存和外部命令触发的持久化参数修改会先取得安全维护租约，运行/标定/其他维护占用时明确返回 `busy`，而不是留下无期限的 `queued` 保存。
- 故障处理：过压、欠压、过流、过温、堵转、CAN 超时、ADC/编码器异常等保护逻辑有主机回归；故障反应态未完成时拒绝清故障，避免对外报告“已清除”而 FSM 仍处于故障态。
- 资源观测：安全任务每秒输出 default/guard/comm 三个 FreeRTOS 任务的剩余栈高水位（单位为 word）；这用于上板压力测试，不代表已经完成硬件栈裕量认证。
- OTA：VectorFOC 默认应用镜像带 App Header 和 CRC32，配合 16 KiB USB Bootloader 升级；上传工具会先读取设备布局并在擦除前校验镜像范围。

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

提交前至少应构建 VectorFOC 的 `MT6816`、`TMR3109` 与 X-STAR-S 的 `HALL`、`ABZ` 组合，并校验 VectorFOC App 的 `VectorFoc.bin` Header。

### 构建 Bootloader

```bash
cmake -S . -B build-boot -G Ninja \
  --toolchain cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOTLOADER_BUILD=ON
cmake --build build-boot --parallel
```

验证时应确认 `VectorFoc_Bootloader.bin` 小于 16 KiB Bootloader 分区；提交前验证记录会给出当前工具链下的实际占用。

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

当前 CTest 注册 41 个自动测试，包含算法、通信、参数存储、Bootloader 协议、App Header 工具、安全保护、ADC/编码器保护，以及 VectorFOC/X-STAR-S 通信分支。新增的实际路径回归覆盖 ADC ISR 启动门控、`MotorStateTask`、闭环 plant、确定性 ADC 噪声、故障清除状态契约、保存维护租约、FreeRTOS 栈水位采样和 CAN 受确认的重启/Bootloader ACK。若使用 Visual Studio 多配置生成器运行测试，需要加配置参数，例如：

```powershell
cmake --build build-test --config Debug --parallel
ctest --test-dir build-test -C Debug --output-on-failure
```

本轮将以重新配置后的构建目录执行完整 CTest；具体结果以提交前的验证记录为准。

### 运行闭环仿真测试

```bash
ctest --test-dir build-test -R test_runner_integration --output-on-failure
```

`test_runner_integration` 会生成 `foc_setpoint_switch.csv`，可用分析脚本绘制速度设定切换响应：

```bash
python ../test/analyze_results.py foc_setpoint_switch.csv
```

该 harness 还覆盖电流/速度/负载扰动、位置设定跟踪和一组模型参数扫掠。它是可重复的主机级回归，不是电机台架性能承诺；详细范围、已覆盖故障和未覆盖硬件场景见 [docs/SIMULATION.md](docs/SIMULATION.md)。

CSV 和 PNG 是构建目录中的可再生输出，未提交到版本库；提交中保留了命令、分析脚本和本次结果，重新运行上述命令即可复现。

### 内存预算与上板门槛

本轮 ARM Release 构建通过，但 VectorFOC/MT6816 的静态占用为 RAM `20,952 / 22,512 B`（93.07%）、CCMRAM `9,024 / 10,240 B`（88.12%）、Flash `100,220 / 110,592 B`（90.62%）；TMR3109 的 RAM/CCMRAM/Flash 为 93.11% / 88.12% / 90.72%。这不是“可以忽略”的余量。

`task_guard` 每秒会通过现有日志输出 `stack_free_w=default/guard/comm`，数值是自启动以来最小的剩余栈空间，单位为 FreeRTOS word。结合 `Protocol_GetStats()` 的 `rx_queue_peak`，应在真实硬件执行 USB 流、CAN burst、标定、故障/清故障、CAN reset/boot 和最大 PWM/ADC 负载后记录数据；在此之前不要根据猜测缩小任务栈或通信队列。详见 [docs/SIMULATION.md](docs/SIMULATION.md)。

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

正常启动路径不会初始化 USB；只有进入升级模式才初始化 USB CDC。上传脚本在 `boot_erase` 前发送 `boot_info`，确认设备应用起始地址、应用区域大小和 8 字节对齐后的镜像长度。提交前只验证到固件镜像生成、工具协议和脚本依赖；`st-flash` 未在当前环境中发现，SWD 烧录和真实 USB OTA 仍需连接硬件后验证。

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
