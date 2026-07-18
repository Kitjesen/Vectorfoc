# VectorFOC 项目介绍

本文档说明当前仓库实际提供的功能、构建入口、测试入口、OTA 能力和安全边界。内容以当前代码与提交前命令核验为准。

## 项目定位

VectorFOC 是基于 STM32G431 的无刷电机 FOC 固件。它面向电机控制板固件开发，而不是桌面应用或完整上位机系统。仓库包含 MCU 固件、Bootloader、板级配置、通信协议、参数存储、安全保护逻辑，以及不依赖硬件的主机测试。

## 功能模块

| 模块 | 当前位置 | 当前能力 |
|---|---|---|
| FOC 算法 | `Src/ALGO/foc/` | Clarke/Park、SVPWM、三角函数、FOC 电流环 |
| 控制器 | `Src/ALGO/control/`, `Src/ALGO/pid/`, `Src/ALGO/trajectory/` | PID、LADRC、前馈、弱磁、限幅、梯形轨迹、速率限制 |
| 电机状态与标定 | `Src/ALGO/motor/` | DS402 风格状态机、Rs/Ls/flux/编码器标定上下文；PWM HAL 操作在短 critical section 外执行，过渡期故障直接关闭桥臂输出 |
| 传感器与板级抽象 | `Src/HAL/`, `Src/config/` | VectorFOC 与 X-STAR-S 两套板级配置，MT6816/TMR3109/Hall/ABZ 路由 |
| 通信 | `Src/COMM/`, `Src/UI/vofa/` | Vector/Inovxio、MIT、CANopen 风格帧，USB CDC 文本命令；Vector GET_ID 仅接受扩展帧、CANopen STOP 只接受 NMT；控制指令经统一 executor 原子发布；重启/进入 Bootloader 仅在 CAN ACK 的 FDCAN Tx event 确认后执行 |
| 参数存储 | `Src/UI/parameter/`, `Src/APP/settings/`, `Src/HAL/bsp/bsp_flash.*` | Flash 参数页、CRC32、提交标记与回退测试；参数 module 只校验、保存、通知及 portable calibration snapshot 映射，APP adapters 分别在正确时序应用运行时设置和具体 encoder calibration；持久化修改通过维护租约串行化，scheduled save 终态失败会恢复最后有效镜像或默认参数 |
| 安全保护 | `Src/SAFE/`, `Src/APP/isr/` | 故障检测、ADC 样本保护、编码器失败计数、CAN timeout watchdog、看门狗监督；启动 ISR readiness gate、X-STAR ADC paired-sample gate、shared ADC IRQ 分发与状态感知的故障清除 |
| Bootloader/OTA | `Src/BOOT/`, `scripts/ota_upload.py`, `scripts/patch_app_header.py` | USB CDC OTA、App Header 生成/验证、Flash 擦写协议；设备布局预检在擦除前完成 |

## 板型和传感器矩阵

| 板型 | CMake 选项 | 默认传感器 | 可选传感器 | 链接脚本 |
|---|---|---|---|---|
| VectorFOC G431 | 默认，`BOARD_XSTAR=OFF` | `MT6816` | `MT6816`, `TMR3109` | `Lib/stm32g431xx_app.ld`，App 从 `0x08004000` 启动 |
| X-STAR-S | `-DBOARD_XSTAR=ON` | `HALL` | `HALL`, `ABZ` | `Lib/stm32g431xx_flash.ld`，独立镜像从 `0x08000000` 启动 |

`POSITION_SENSOR=AUTO` 会按板型选择默认传感器。CMake 会在配置阶段拒绝不兼容组合，例如 VectorFOC + `HALL` 或 X-STAR-S + `MT6816`。

## 构建入口

### VectorFOC App

```bash
cmake -S . -B build-vector -G Ninja \
  "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPOSITION_SENSOR=MT6816
cmake --build build-vector --parallel
```

已核验：该命令在 xPack GNU Arm Embedded GCC 13.3.1、CMake 4.2.1、Ninja 1.13.0 环境下通过，并生成带 App Header 的 `VectorFoc.bin`。

### Bootloader

```bash
cmake -S . -B build-boot -G Ninja \
  "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOTLOADER_BUILD=ON
cmake --build build-boot --parallel
```

已核验：该命令通过；每次发布都应以 `arm-none-eabi-size` 复核 `VectorFoc_Bootloader.elf` 低于 16 KiB 分区限制。

### X-STAR-S

```bash
cmake -S . -B build-xstar-hall -G Ninja \
  "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOARD_XSTAR=ON \
  -DPOSITION_SENSOR=HALL
cmake --build build-xstar-hall --parallel
```

已核验：该命令通过，使用独立 `0x08000000` App 布局。

## 测试和仿真

主机测试入口：

```bash
cmake -S test -B build-test
cmake --build build-test --parallel
ctest --test-dir build-test --output-on-failure
```

当前 `test/CMakeLists.txt` 注册 46 个自动 CTest 测试；最近一次干净 Clang/Ninja 主机构建通过 46/46。覆盖范围包括：

- FOC 基础算法、PID、LADRC、轨迹、速率限制、三角函数；
- 参数运行时通知 seam 与 APP `RuntimeSettings` 适配器的单参数映射、批量重放和编码器 offset 错误传播；
- 参数 encoder-calibration seam：快照保存/恢复、无效元数据拒绝、默认回退清空校准状态；
- `test_runner_integration` 简化 PMSM 闭环仿真：电流/速度/负载扰动、速度设定切换、位置设定和参数扫掠；
- 真实 `MotorStateTask` + mock HAL/motor plant 闭环回归，以及确定性 ADC 噪声稳定性回归；
- ADC ISR readiness gate：未完成初始化时 injected callback 不得读取传感器、状态机或 PWM 路径；X-STAR-S 还要求 ADC1/ADC2 注入完成标记配对且无错误；
- 故障清除与上报重试：快速故障的证据不会因清除/回调失败而丢失，且故障反应态不会被伪装为已恢复；
- CAN reset/bootloader 仅在 Tx event 完成后执行，超时取消；
- 持久化保存的维护租约、运行/标定 busy 响应、有界失败终态，以及最后有效镜像/默认参数回滚；
- SMO、参数存储、Bootloader 协议、App Header 工具；
- 通信协议、命令执行、协议管理器；
- ADC 样本保护、ADC shared IRQ、编码器失败保护、看门狗、CAN timeout arm/run-state 语义、FSM 输出安全、故障检测。
- FreeRTOS 任务栈高水位诊断：调度器和三个核心任务均就绪时采样 default/guard/comm 的剩余 stack word；未就绪时安全返回不可用。

Windows/MSVC 与 GCC/Ninja 都是 CI 的一等验证目标；提交前的干净构建记录与当前 CTest 数量应以 `ctest --test-dir <build-dir> --output-on-failure` 为准。仿真命令、模型假设和硬件验证缺口见 [SIMULATION.md](SIMULATION.md)。

### 当前内存预算与测量方法

最新 ARM Release 矩阵已在同一 xPack GNU Arm GCC 13.3.1 工具链下构建。`BOOTFLAG` 的 `16 / 16 B` 是链接脚本刻意保留的交接区，不计入一般 RAM 余量：

| 镜像 | RAM | CCMRAM | Flash |
|---|---:|---:|---:|
| VectorFOC / MT6816 | 20,960 / 22,512 B（93.11%） | 9,024 / 10,240 B（88.12%） | 101,384 / 110,592 B（91.67%） |
| VectorFOC / TMR3109 | 20,968 / 22,512 B（93.14%） | 9,024 / 10,240 B（88.12%） | 101,532 / 110,592 B（91.81%） |
| X-STAR-S / HALL | 14,216 / 22,512 B（63.15%） | 1,024 / 10,240 B（10.00%） | 79,424 / 126,976 B（62.55%） |
| X-STAR-S / ABZ | 14,208 / 22,512 B（63.11%） | 1,024 / 10,240 B（10.00%） | 79,136 / 126,976 B（62.32%） |
| Bootloader | 7,352 / 22,512 B（32.66%） | 0 / 10,240 B | 13,932 / 16,384 B（85.03%） |

这些数字是链接期静态占用，不是任务/中断栈安全证明，也没有为运行时峰值预留保证。

这些数值是链接期静态占用，不是任务/中断栈安全证明。`task_guard` 每秒输出 `stack_free_w=default/guard/comm`（FreeRTOS word），可与 `Protocol_GetStats()` 的 `rx_queue_peak` 一起记录。必须在硬件压力测试后再调整任务栈、VOFA/USB 队列或 CAN 队列；当前不得以主机仿真替代此结论。

## Bootloader 和 OTA

VectorFOC 的 Flash 布局来自 `Src/BOOT/boot_config.h`：

| 区域 | 地址范围 | 大小 |
|---|---:|---:|
| Bootloader | `0x08000000` - `0x08003FFF` | 16 KiB |
| Application | `0x08004000` - `0x0801EFFF` | 108 KiB |
| Config/Params | `0x0801F000` - `0x0801FFFF` | 4 KiB |

App Header 位于 `APP_ADDR_START + 0x200`，即 `0x08004200`。App 构建后会调用 `scripts/patch_app_header.py` 生成并校验 Header；缺失或损坏 Header 不再被当作可启动旧镜像。Bootloader 启动时检查栈指针、Reset Handler、Magic、Header 保留字段、payload size 和 CRC32。正常跳转 App 时 USB 不初始化，只有升级模式才启动 USB CDC。

OTA 协议命令由 `Src/BOOT/boot_protocol.c` 处理，包括：

| 命令 | 用途 |
|---|---|
| `boot_info` | 查询 App 起始地址和大小 |
| `boot_erase` | 擦除 App 区域 |
| `boot_write,addr,len` | 写入 8 字节对齐的数据块 |
| `boot_verify,crc,size` | 按 Header 和 payload CRC 校验镜像 |
| `boot_reboot` | 校验 App 有效后跳转 |

App 侧 VOFA/USB 命令 `boot_enter` 会先请求 DS402 状态机进入 `STATE_SWITCH_ON_DISABLED`，发送 ACK 后在任务上下文请求重启进入 Bootloader。上传脚本在擦除前查询 `boot_info`，检查设备应用起始地址、区域大小和对齐后的镜像长度。

CAN 的 `SAVE` 命令只有在取得维护租约后才返回 `queued`，运行/标定/已有维护时返回 `busy`。USB `save_flash=1` 同样先取得租约；接受后会报告 `queued`，失败重试状态以 `retrying` 表示，三次失败后报告终态 `failed`。外部命令触发的会持久化参数修改会在改动运行时值前预留租约，避免“返回失败但参数已改变”。校准完成后的内部保存请求会直接调度 `Param_ScheduleSave()`，命令服务随后在可取得维护窗口时处理；终态失败也会回滚到最后有效提交镜像，或在没有可加载镜像时恢复默认参数并清除编码器标定有效状态。USB RX 队列满时会累计计数并发送 `rx_overflow=1` 诊断；Bootloader 队列满时会中止当前事务并返回 `BOOT_ERR_RX_OVERFLOW`。CAN 的 `RESET` 与 `BOOTLOADER` 命令会先使功率桥去使能、请求 DS402 切换禁止，并等待对应 ACK 的 FDCAN Tx event；等待期间不会全局关中断，以保证发送事件与超时仍能推进。确认完成才执行重启；超时则取消发送、保持功率桥去使能并报告通信超时，不重启。X-STAR-S 为独立 App，`BOOTLOADER` 明确返回 `unsupported` 而不会假装进入不存在的 Bootloader。

本轮核验覆盖固件镜像生成、App Header 校验、Bootloader 编译和 OTA 脚本依赖检查；未连接 SWD/USB 硬件，未做实际烧录或真实 USB OTA 端到端验证。

## 当前安全边界

当前实现不能称为 secure boot 或安全 OTA。已实现的是完整性检查，不是身份认证：

- OTA 镜像使用 Header 和 CRC32 校验传输/写入完整性。
- Bootloader 不验证数字签名。
- Bootloader 不认证上传端。
- CAN 控制面没有总线级认证。
- USB CDC 调试与参数写入能力应视为受信任维护接口。

生产部署需要至少补齐以下能力后，才可以宣称安全 OTA：

- 签名 manifest，包含版本、大小、哈希、兼容性和防回滚信息；
- MCU 内置公钥或公钥哈希验签；
- 失败回滚与版本单调策略；
- STM32 RDP/WRP 等读写保护配置；
- 受控的调试口、USB 和 CAN 暴露面策略。

在这些能力完成前，只应在可信物理环境或受控维护通道中使用 OTA。
