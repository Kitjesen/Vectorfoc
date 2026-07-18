# VectorFOC 项目介绍

本文档说明当前仓库实际提供的功能、构建入口、测试入口、OTA 能力和安全边界。内容以当前代码与提交前命令核验为准。

## 项目定位

VectorFOC 是基于 STM32G431 的无刷电机 FOC 固件。它面向电机控制板固件开发，而不是桌面应用或完整上位机系统。仓库包含 MCU 固件、Bootloader、板级配置、通信协议、参数存储、安全保护逻辑，以及不依赖硬件的主机测试。

## 功能模块

| 模块 | 当前位置 | 当前能力 |
|---|---|---|
| FOC 算法 | `Src/ALGO/foc/` | Clarke/Park、SVPWM、三角函数、FOC 电流环 |
| 控制器 | `Src/ALGO/control/`, `Src/ALGO/pid/`, `Src/ALGO/trajectory/` | PID、LADRC、前馈、弱磁、限幅、梯形轨迹、速率限制 |
| 电机状态与标定 | `Src/ALGO/motor/` | DS402 风格状态机、Rs/Ls/flux/编码器标定上下文 |
| 传感器与板级抽象 | `Src/HAL/`, `Src/config/` | VectorFOC 与 X-STAR-S 两套板级配置，MT6816/TMR3109/Hall/ABZ 路由 |
| 通信 | `Src/COMM/`, `Src/UI/vofa/` | Vector/Inovxio、MIT、CANopen 风格帧，USB CDC 文本命令；控制指令经统一 executor 原子发布 |
| 参数存储 | `Src/UI/parameter/`, `Src/HAL/bsp/bsp_flash.*` | Flash 参数页、CRC32、提交标记与回退测试 |
| 安全保护 | `Src/SAFE/`, `Src/APP/isr/` | 故障检测、ADC 样本保护、编码器失败计数、看门狗监督 |
| Bootloader/OTA | `Src/BOOT/`, `scripts/ota_upload.py`, `scripts/patch_app_header.py` | USB CDC OTA、App Header 生成/验证、Flash 擦写协议 |

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
  --toolchain cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DPOSITION_SENSOR=MT6816
cmake --build build-vector --parallel
```

已核验：该命令在 xPack GNU Arm Embedded GCC 13.3.1、CMake 4.2.1、Ninja 1.13.0 环境下通过，并生成带 App Header 的 `VectorFoc.bin`。

### Bootloader

```bash
cmake -S . -B build-boot -G Ninja \
  --toolchain cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOTLOADER_BUILD=ON
cmake --build build-boot --parallel
```

已核验：该命令通过；当前 `VectorFoc_Bootloader.elf` Flash 占用 13,756 B，低于 16 KiB 分区限制。

### X-STAR-S

```bash
cmake -S . -B build-xstar-hall -G Ninja \
  --toolchain cmake/gcc-arm-none-eabi.cmake \
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

当前 `test/CMakeLists.txt` 注册 36 个自动 CTest 测试。覆盖范围包括：

- FOC 基础算法、PID、LADRC、轨迹、速率限制、三角函数；
- `test_runner_integration` 闭环仿真测试，包含速度设定切换场景；
- SMO、参数存储、Bootloader 协议、App Header 工具；
- 通信协议、命令执行、协议管理器；
- ADC 样本保护、编码器失败保护、看门狗、FSM 输出安全、故障检测。

`test/test_foc_closed_loop.c` 和 `test/motor_plant.c` 也包含闭环仿真代码，但 `test_foc_closed_loop.c` 当前不是默认 CTest 注册目标；不要把它写作默认测试结果。

本轮核验结果：

- 新建主机构建目录执行完整 CTest：Clang 与 Visual Studio/MSVC 均为 36/36 通过。
- `test_runner_integration`：1/1 通过。
- `python ../test/analyze_results.py foc_setpoint_switch.csv`：最终速度 19.75 rad/s，目标 20.0 rad/s，误差 0.2450 rad/s，生成 `foc_setpoint_switch.png`。
仿真命令和输出解释见 [SIMULATION.md](SIMULATION.md)。

## Bootloader 和 OTA

VectorFOC 的 Flash 布局来自 `Src/BOOT/boot_config.h`：

| 区域 | 地址范围 | 大小 |
|---|---:|---:|
| Bootloader | `0x08000000` - `0x08003FFF` | 16 KiB |
| Application | `0x08004000` - `0x0801EFFF` | 108 KiB |
| Config/Params | `0x0801F000` - `0x0801FFFF` | 4 KiB |

App Header 位于 `APP_ADDR_START + 0x200`，即 `0x08004200`。App 构建后会调用 `scripts/patch_app_header.py` 生成并校验 Header，Bootloader 启动时检查栈指针、Reset Handler、Magic、Header 保留字段、payload size 和 CRC32。

OTA 协议命令由 `Src/BOOT/boot_protocol.c` 处理，包括：

| 命令 | 用途 |
|---|---|
| `boot_info` | 查询 App 起始地址和大小 |
| `boot_erase` | 擦除 App 区域 |
| `boot_write,addr,len` | 写入 8 字节对齐的数据块 |
| `boot_verify,crc,size` | 按 Header 和 payload CRC 校验镜像 |
| `boot_reboot` | 校验 App 有效后跳转 |

App 侧 VOFA/USB 命令 `boot_enter` 会先请求 DS402 状态机进入 `STATE_SWITCH_ON_DISABLED`，发送 ACK 后在任务上下文请求重启进入 Bootloader。

CAN 的 `SAVE` 命令返回 `queued` 表示已安排写入；USB `save_flash=1` 先返回 `queued`，再由持有状态机维护租约的命令服务回报 `succeeded` 或 `retrying`。USB RX 队列满时会累计计数并发送 `rx_overflow=1` 诊断；Bootloader 队列满时会中止当前事务并返回 `BOOT_ERR_RX_OVERFLOW`。CAN 的 `RESET` 与 `BOOTLOADER` 命令会先把 ACK 放入 CAN 发送队列、执行安全停机，再由通信任务延后执行重启；X-STAR-S 为独立 App，`BOOTLOADER` 明确返回 `unsupported` 而不会假装进入不存在的 Bootloader。

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
