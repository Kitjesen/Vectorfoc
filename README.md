# VectorFOC

VectorFOC 是面向 STM32G431 电机控制板的开源 FOC 固件。项目包含 20 kHz 电流环、速度/位置控制、DS402 状态机、CAN/USB 通信、参数存储、Bootloader OTA 升级流程，以及可在主机运行的算法、安全和控制链路回归测试。

[![CI](https://github.com/kitjesen/vectorfoc/actions/workflows/vectorfoc-ci.yml/badge.svg)](https://github.com/kitjesen/vectorfoc/actions/workflows/vectorfoc-ci.yml)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

![FOC Architecture](fig/foc.png)

## 当前能力

- 20 kHz FOC 电流环：Clarke/Park、SVPWM、d/q 轴 PI、限幅与异常输入保护。
- 控制模式：力矩、速度、位置、轨迹、开环、MIT 阻抗接口。
- 状态机：DS402 风格状态切换，空闲/故障态保持去使能，运行/标定态显式上电。逻辑状态更新保持短 critical section；常规 PWM HAL 调用在锁外执行，过渡期间发生故障会直接关闭 TIM1 桥臂输出并阻止重入。
- 启动保护：ADC 注入转换虽可在外设启动后立即触发，但 FOC ISR 会在电机、传感器和安全对象全部初始化完成前保持惰性，避免启动窗口访问半初始化状态。X-STAR-S 还要求 ADC1 与 ADC2 注入序列均完成且无错误才接受一个 FOC 样本，并在接受后清除 ADC2 完成标记，避免复用陈旧样本。
- 位置传感器：VectorFOC 板支持 MT6816、TMR3109；X-STAR-S 支持 Hall、ABZ。四种传感器统一经过 `Src/HAL/position_sensor/` 的 `PositionSensor` 模块发布位置、机械角、电角度、速度、健康状态和校准快照；ALGO/APP/SAFE 不再直接依赖具体传感器型号。
- 通信：CAN 上的 Vector、MIT、CANopen 风格帧；USB CDC 调试与 VOFA+/VectorStudio 命令。CAN 链路按 protocol manager -> generic transport -> BSP CAN -> STM32 HAL 分层，接收回调按 STM32 HAL/FDCAN ISR -> BSP CAN -> CAN transport -> protocol manager 反向上送。Vector `GET_ID` 快路径只接受 29-bit 扩展帧；CANopen stopped 状态只处理 NMT，忽略 RPDO/SDO。CAN 的 `RESET`/`BOOTLOADER` ACK 必须由 FDCAN Tx event 确认完成后才执行动作；等待期间功率桥保持去使能，超时会取消待发请求、报告通信超时且不复位。
- 参数与安全边界：Flash 参数页带 CRC/提交标记；显式保存和外部命令触发的持久化参数修改会先取得安全维护租约，运行/标定/其他维护占用时明确返回 `busy`。异步保存连续三次失败进入终态时，会丢弃对应保存代际并重新加载最后有效提交镜像；没有可加载镜像时恢复默认参数并清除编码器标定有效状态。
- 故障处理：过压、欠压、过流、过温、堵转、CAN 超时、ADC/编码器异常等保护逻辑有主机回归；CAN timeout 默认 1000 ms，只有有效 CAN 通信已喂狗且处于 `STATE_MODE_RUNNING` 时才触发，设为 `0` 可禁用。故障反应态未完成时拒绝清故障，避免对外报告“已清除”而 FSM 仍处于故障态。
- 资源观测：安全任务每秒输出 default/guard/comm 三个 FreeRTOS 任务的剩余栈高水位（单位为 word）；这用于上板压力测试，不代表已经完成硬件栈裕量认证。
- OTA：VectorFOC 默认应用镜像带 App Header 和 CRC32，配合 16 KiB USB Bootloader 升级；上传工具会先读取设备布局并在擦除前校验镜像范围。

## 支持硬件

| 板型 | MCU | 位置传感器 | 镜像布局 | 说明 |
|---|---|---|---|---|
| VectorFOC G431 | STM32G431CBU6 | `MT6816`, `TMR3109` | Bootloader `0x08000000`，App `0x08004000` | 默认板型；App 构建会写入 Header/CRC |
| X-STAR-S | STM32G431RBT6 | `HALL`, `ABZ` | 独立 App `0x08000000` | `-DBOARD_XSTAR=ON`；当前不使用 VectorFOC USB Bootloader |

CMake 会拒绝不兼容组合：VectorFOC 仅接受 `AUTO`、`MT6816`、`TMR3109`；X-STAR-S 仅接受 `AUTO`、`HALL`、`ABZ`。

## 分层架构与替换边界

当前依赖方向是：`APP` 负责启动编排和任务，`COMM/UI` 负责协议与命令，`ALGO` 负责控制算法与状态机，`SAFE` 负责故障策略，`HAL`/`Src/config` 承接板级外设和传感器差异。上层代码应依赖公开 seam，不应重新包含具体板卡、具体编码器或 STM32 HAL 细节。

主控/板级快速替换 seam 已收束到两个入口，但还不是完全无痛 MCU 迁移：

- 运行时硬件入口是 `Motor_HAL_Handle_t`，由 `pwm`、`adc`、`encoder` 三个接口组成；当前实现为 `g431_hal_handle` 和 `xstar_hal_handle`。
- 编译期板级入口是 `Src/config/board_config.h` 与 CMake 选项：默认 VectorFOC，`-DBOARD_XSTAR=ON` 切到 X-STAR-S；所有源文件只应包含 `board_config.h`，不直接包含 `Src/config/boards/board_*.h`。
- 迁移到新主控/新功率板时，先新增 `board_<name>.h`、对应 `Motor_HAL_Handle_t` 实现和 CMake 合法组合，再补齐 ADC/PWM/encoder/CAN/Flash/复位/Bootloader 服务测试。
- 仍未完成的 MCU seam：时钟、GPIO、ADC/PWM ISR、USB、Flash、复位和 Bootloader handoff 仍有 STM32G4 耦合；不要把当前板级抽象解读成已完成跨 MCU 移植。

位置传感器替换 seam 已落地：

- 上层只调用 `PositionSensor_*` 函数；旧代码可经 `position_sensor_motor_hal.c` 暴露的 `Motor_HAL_EncoderInterface_t` 兼容桥继续工作。
- `PositionSensorAdapter_t` 是私有 adapter 表；MT6816、TMR3109、Hall、ABZ 的具体驱动只在 `Src/HAL/position_sensor/position_sensor_*.c` 和 `position_sensor_selection.c` 里出现。
- `HW_POSITION_SENSOR_MODE` 只应在板级配置、selection 和底层驱动中使用；ALGO/APP/SAFE 不应按 MT6816/TMR3109/Hall/ABZ 分支。
- 校准持久化通过 `PositionSensorCalibrationSnapshot_t` 保存/恢复；启动时先初始化具体驱动，再从 Flash 恢复校准，避免驱动默认值覆盖已恢复数据。
- 方向、极对数、线性 LUT 标定走 typed raw-calibration API；Hall/ABZ 只声明并暴露实际支持的能力。

新增板/传感器清单：

1. 新板：新增 `Src/config/boards/board_<name>.h`，保持宏名与现有板一致；新增或复用 `Motor_HAL_Handle_t` 实现；在 `CMakeLists.txt` 增加板型选项、链接脚本和合法传感器矩阵；补充 HAL port binding、PWM/ADC phase mapping、CAN/BSP 和启动测试。
2. 新传感器：新增底层驱动；新增一个 `position_sensor_<name>.c` 实现私有 `PositionSensorAdapter_t`；在 `position_sensor_internal.h` 和 `position_sensor_selection.c` 注册；在 `board_config`/CMake 中加入枚举和合法组合；补充 selection/runtime/architecture 测试。
3. 相位映射：当前板级定义为 A-U、B-V、C-W，且电流通道保持 Ia/Ib/Ic 对齐。任何换板、换功率级或线束调整后，都必须低压限流核验三相 PWM、电流采样极性、编码器方向和电角度偏移，并重新做电流 offset、相序/方向和编码器零位标定。

启动顺序遵循 fail-closed 原则：`App_Init` 清 readiness，初始化 BSP/日志/错误管理/状态机/安全，再绑定 ADC/PWM、安装参数 target binding、初始化传感器、恢复参数、启动 ADC 与 PWM 采样触发、做电流 offset、禁用桥臂、恢复编码器 offset、采集首帧反馈并做初始安全扫描，然后执行通信 bootstrap，最后初始化电机运行时、重放运行时参数并发布 FOC readiness。任一关键步骤失败会 `Error_Handler()` 或触发故障；20 kHz ISR 在 readiness 发布前不得访问半初始化对象，功率桥默认保持关闭。

CAN 启动边界由 `AppComm_Bootstrap()` 维护：先 `CAN_Transport_Init()` 注册 BSP RX callback，再 `Protocol_RegisterTransport()`，然后 `Protocol_Init()` 和 `Safety_RegisterFaultCallback()`，最后校验持久化波特率（非法值先归一化为 1 Mbps）并只调用一次 `BSP_CAN_Init()`；过滤器、启动或通知配置失败会直接上报，不能被波特率回退掩盖。发送方向是 protocol manager -> `TransportInterface` -> CAN transport -> BSP CAN -> STM32 HAL/FDCAN；接收方向由 STM32 HAL/FDCAN 中断反向回调到 BSP CAN、CAN transport，再进入 protocol manager。

## 快速开始

### 工具

- CMake 3.20+（Bootloader/App），主机测试最低 3.10。
- xPack GNU Arm Embedded GCC。本仓库已用 `arm-none-eabi-gcc 13.3.1` 验证固件构建。
- Ninja 或 Make。
- ARM 交叉构建不支持 Visual Studio generator；请使用 Ninja 或 Makefiles。
- Python 3.8+。OTA 上传需要 `pyserial`，本轮环境已检测到 `pyserial 3.5`。
- 首次烧录需要 SWD 工具，例如 STM32CubeProgrammer 或 `st-flash`。

### 构建 VectorFOC App

```bash
cmake -S . -B build-vector -G Ninja \
  "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPOSITION_SENSOR=MT6816
cmake --build build-vector --parallel
```

提交前至少应构建 VectorFOC 的 `MT6816`、`TMR3109` 与 X-STAR-S 的 `HALL`、`ABZ` 组合，并校验 VectorFOC App 的 `VectorFoc.bin` Header。

### 构建 Bootloader

```bash
cmake -S . -B build-boot -G Ninja \
  "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOTLOADER_BUILD=ON
cmake --build build-boot --parallel
```

验证时应确认 `VectorFoc_Bootloader.bin` 小于 16 KiB Bootloader 分区；提交前验证记录会给出当前工具链下的实际占用。

### 构建 X-STAR-S

```bash
cmake -S . -B build-xstar-hall -G Ninja \
  "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake" \
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

测试数量和通过结果以当前 `ctest --test-dir <build-dir> --output-on-failure` 输出为准。覆盖范围包括算法、通信、参数存储、运行时设置、参数目标绑定、PositionSensor selection/runtime/persistence/architecture、`Motor_HAL` 兼容层、encoder-calibration 适配器、Bootloader 协议、App Header 工具、安全保护、ADC/编码器保护，以及 VectorFOC/X-STAR-S 通信分支。实际路径回归覆盖 MT6816、TMR3109、Hall、ABZ 和非法模式的初始化分发、通信启动的 CAN/transport/protocol/safety 调用顺序与故障回调失败传播、ADC shared IRQ 分发、独立注册的 X-STAR 双 ADC 新鲜度门控、FSM 过渡期桥臂关断、保存终态回滚、ADC ISR 启动门控、`MotorStateTask`、闭环 plant、确定性 ADC 噪声、故障清除状态契约、保存维护租约、FreeRTOS 栈水位采样和 CAN 受确认的重启/Bootloader ACK。若使用 Visual Studio 多配置生成器运行测试，需要加配置参数，例如：

```powershell
cmake --build build-test --config Debug --parallel
ctest --test-dir build-test -C Debug --output-on-failure
```

提交前仍应以本地 `ctest --test-dir <build-dir> --output-on-failure` 的实际输出为准。

### 运行闭环仿真测试

```bash
ctest --test-dir build-test -R test_runner_integration --output-on-failure
```

`test_runner_integration` 会生成 `foc_setpoint_switch.csv`，可用分析脚本绘制速度设定切换响应：

```bash
python ../test/analyze_results.py foc_setpoint_switch.csv
```

该 harness 还覆盖电流/速度/负载扰动、位置设定跟踪和一组模型参数扫掠。最近一次主机模型运行的 20 rad/s 速度设定切换结果为：最终速度 19.75 rad/s、稳态误差 0.2450 rad/s、模型超调 30.75 rad/s。它是可重复的主机级回归，不是电机台架性能承诺；详细范围、已覆盖故障和未覆盖硬件场景见 [docs/SIMULATION.md](docs/SIMULATION.md)。

CSV 和 PNG 是构建目录中的可再生输出，未提交到版本库；提交中保留了命令、分析脚本和本次结果，重新运行上述命令即可复现。

### 内存预算与上板门槛

当前 ARM Release 矩阵的 `VectorFoc.elf` 已用 `arm-none-eabi-size` 读取。下表是 text/data/bss 静态段大小，不是任务栈、ISR 栈或运行时峰值证明；最终发布仍要以完整链接输出和硬件压力日志复核。`BOOTFLAG` 的 `16 / 16 B` 是链接脚本刻意保留的交接区，不应与一般 RAM 余量混为一谈。

| 镜像 | text | data | bss |
|---|---:|---:|---:|
| VectorFOC / MT6816 | 104,052 B | 1,812 B | 28,524 B |
| VectorFOC / TMR3109 | 104,148 B | 1,820 B | 28,524 B |
| X-STAR-S / HALL | 84,000 B | 1,092 B | 14,252 B |
| X-STAR-S / ABZ | 83,844 B | 1,116 B | 14,212 B |

最新链接输出中，VectorFOC/MT6816 使用 Flash 95.76%、RAM 93.46%、CCMRAM 90.62%；TMR3109 使用 Flash 95.86%、RAM 93.50%、CCMRAM 90.62%。两个组合的余量仍偏紧，不能据此推断运行时栈安全；发布前必须复核同一工具链下的实际链接输出。

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
│   ├── HAL/            # STM32G4 外设、编码器底层驱动、PositionSensor 传感器边界、板级抽象
│   ├── SAFE/           # 安全保护与看门狗监督
│   ├── UI/             # 参数表、VOFA+/VectorStudio、LED、错误上报
│   └── config/         # 板型与传感器配置
├── Lib/                # STM32 HAL、CMSIS、FreeRTOS、链接脚本
├── test/               # 主机单元测试与仿真测试
├── scripts/            # 构建、OTA、App Header 工具
├── docs/               # 设计、Bootloader、项目说明
└── cmake/              # ARM 工具链文件
```

更多说明见 [docs/PROJECT_OVERVIEW.md](docs/PROJECT_OVERVIEW.md)、[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)、[docs/SIMULATION.md](docs/SIMULATION.md)、[docs/RUST_MODULARIZATION.md](docs/RUST_MODULARIZATION.md)、[docs/OTA_BOOTLOADER.md](docs/OTA_BOOTLOADER.md) 和 [BUILD_GUIDE.md](BUILD_GUIDE.md)。

## GitHub 上游合并说明

从 GitHub 上游合并时，优先保留当前分层 seam 和 fail-closed 行为：`Motor_HAL_Handle_t`/`board_config` 板级入口、`PositionSensorAdapter_t` 传感器入口、protocol manager -> generic transport -> BSP CAN -> STM32 HAL 通信链路、CAN ACK Tx event 确认、参数维护租约和启动 readiness gate 都不应被绕开。

拒绝把不安全的 `V/F`、`I/F`、固定电压/固定电流开环实现合入为默认控制路径；任何调试注入或开环实验都必须显式受状态机、安全限幅、低压验证和测试保护约束，不能替代闭环 FOC、故障处理、标定和硬件确认流程。

## 硬件验证状态

本仓库当前文档和主机测试只能证明代码路径、构建配置和仿真/单元测试行为；截至本次说明更新，未连接实体电机、真实线束、真实 CAN/USB、SWD 烧录或 USB OTA 做端到端硬件验证。上板前必须低压限流复核 A-U、B-V、C-W 相位映射、电流采样极性、编码器方向、故障输入、CAN ACK 和重新标定结果。

## 许可证

Copyright 2024-2026 VectorFOC Contributors.
Licensed under the [Apache License, Version 2.0](LICENSE).
