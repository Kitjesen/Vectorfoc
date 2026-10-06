<p align="center">
  <img src="docs/images/foc.png" alt="VectorFOC logo" width="360">
</p>

<h1 align="center">VectorFOC</h1>

<p align="center">
  <strong>Inovxio 电机控制算法与固件</strong><br>
  FOC · CAN / USB · 可替换硬件适配
</p>

<p align="center">
  <a href="https://github.com/Kitjesen/Vectorfoc/actions/workflows/vectorfoc-ci.yml"><img src="https://github.com/Kitjesen/vectorfoc/actions/workflows/vectorfoc-ci.yml/badge.svg" alt="CI"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache%202.0-blue.svg" alt="Apache License 2.0"></a>
</p>

VectorFOC 是 Inovxio 的电机控制算法与固件。纯算法在 `algorithm/`，固件负责采样、状态、保护、通信和硬件连接。

## 从这里读

| 想了解什么 | 入口 |
| --- | --- |
| 一次电流环怎么计算 | [`algorithm/foc_current_loop.c`](algorithm/foc_current_loop.c) |
| Clarke、Park、SVPWM | [`algorithm/foc_transforms.c`](algorithm/foc_transforms.c) |
| 主控制流程与模式 | [`src/foc/control_dispatcher.c`](src/foc/control_dispatcher.c) 的 `Control_RunCurrentCycle` |
| 采样与调度 | [`src/app/foc_isr.c`](src/app/foc_isr.c) |
| 启动、状态与故障 | [`application_init.c`](src/app/application_init.c)、[`motor_runtime.c`](src/motor/motor_runtime.c)、[`drive_state_machine.c`](src/motor/drive_state_machine.c) |

闭环控制从 ADC 中断开始。只有电机状态允许运行，才进入控制和输出：

```text
ADC / 编码器采样 → 快速保护 → 状态机与补偿调度
  → Motor_RunControlCycle（状态门控）
  → Control_RunCurrentCycle
      1. 命令斜坡
      2. 控制模式与参考值
      3. 位置环 → 速度环（各自到期执行）
      4. 弱磁 → FOC 电流环 → PWM
  → SMO 观测器 → 示波器采样
```

`OPEN`、`VF` 直接注入电压，绕过电流 PI。上电保持输出关闭；电流零偏就绪且收到显式使能后才能运行，故障清除不会自动恢复输出。

## 频率只在一处配置

[`motor_configuration.h`](src/config/motor_configuration.h) 从板级 PWM 频率派生采样、电流环和命令斜坡的时间步长。当前 PWM 为 20 kHz，每周期完成一次电流采样。

| 工作 | 当前频率 / 周期 | 做什么 |
| --- | --- | --- |
| 采样、电流环、命令斜坡 | 20 kHz / 50 μs | 采样反馈、产生电压和 PWM、平滑目标值 |
| 速度环 | 5 kHz / 200 μs | 速度 PID 或 LADRC 生成 `Iq_ref` |
| 位置环 | 1 kHz / 1 ms | 位置 PID 生成速度目标 |
| 状态机 | 1 kHz / 1 ms | 更新 DS402 状态 |
| 力矩补偿 | 5 kHz / 200 μs | 前馈和齿槽补偿；弱磁仍随电流环更新 |
| 示波器采样 | 1 kHz / 1 ms | 记录控制数据，由诊断任务发送 |

`COMMAND_RAMP_FREQUENCY_HZ` 表示命令处理频率；速度与位置 PID 分别由 `VELOCITY_LOOP_FREQUENCY_HZ` 和 `POSITION_LOOP_FREQUENCY_HZ` 设置。`*_PERIOD_S` 是计算步长，`*_DECIMATION` 是分频数。5 kHz 每 4 个快速周期执行一次，1 kHz 每 20 个周期执行一次；填入零值、超过基准或不能整除的频率会编译报错。

## 文件按功能分开

```text
algorithm/   FOC、变换、PID/LADRC、轨迹、限幅、补偿与观测器
src/
├── app/     初始化、中断和周期任务
├── motor/   状态、生命周期和标定
├── foc/     控制调度和算法适配
├── safety/  保护、故障与错误记录
├── config/  板型、参数和存储
├── comm/    协议、命令和遥测
├── hal/     ADC、PWM、编码器和板级驱动
└── boot/    Bootloader 与升级
platform/    芯片启动、外设和第三方源码
```

`algorithm/` 不依赖 STM32、HAL 或板型宏，输入数值和状态，输出控制量。硬件适配集中在 `src/hal` 和 `platform`；`platform/VectorFOC.ioc` 用于重新生成外设初始化，当前 CMake 不依赖 `.mxproject`。

## 构建与测试

从仓库根目录执行：

```bash
cmake -S tests -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/host --parallel 4
ctest --test-dir build/host --output-on-failure

cmake -S . -B build/arm --toolchain cmake/gcc-arm-none-eabi.cmake
cmake --build build/arm --parallel 4
```

主机测试在 Release 中也保留断言。当前 19 项检查覆盖算法、控制模式、通信、参数兼容、启动与故障，以及 CubeMX 与板级 PWM 配置的一致性；配置阶段另检查 16 组合法和非法频率。控制、前馈和标定的 5 组逐周期对照通过，默认传感器与 TMR3109 的 ARM 构建通过。详见[验证记录](docs/BUILD_GUIDE.md#最新验证)。

软件测试不代替门极电平、ADC 连续性、零偏、真实电机和 OTA 上板验证，本轮未烧录。

## 继续阅读

- [算法导读](algorithm/README.md) · [固件控制适配](src/foc/README.md)
- [构建与验证](docs/BUILD_GUIDE.md) · [协议字段](src/comm/PROTOCOL_CN.md)
- [与 GitHub main 的差异](docs/UPSTREAM_COMPARISON.md) · [文档索引](docs/README.md)

当前整理分支基于 `4520d1c`，未合并 `main` 上新增的安全、传感器和通信修复；结构简化不代表所有功能领先于主线。

Copyright 2024–2026 VectorFOC Contributors · [Apache License 2.0](LICENSE)
