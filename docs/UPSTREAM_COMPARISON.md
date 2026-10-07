# 主线合并记录

2026-10-07，将整理分支 `codex/vector-control-cleanup`（`da2b068`）与 GitHub `main`（`a756902`）合并。两边共同起点是 `4520d1c`；主线新增的 15 个提交保留在 Git 历史中。

## 保留的结构

- 纯算法位于 `algorithm/`，不依赖 STM32、HAL 或板型宏。
- 固件按 `app/boot/comm/config/foc/hal/motor/safety` 八个职责目录组织。
- 控制由 `Control_RunCurrentCycle` 完成模式选择、独立外环调度、弱磁和电流环装配；不恢复旧 `inner.c` 转发。
- 速度环 5 kHz，位置环 1 kHz，采样和电流环 20 kHz；非法频率在编译阶段被拒绝。
- 继续保持被动启动零偏采样、显式使能，以及清故障后输出关闭。保留已有 VF/IF 模式。

## 接入的主线修复

| 能力 | 当前位置 |
| --- | --- |
| ADC 新鲜度、编码器连续失败保护、看门狗监督 | `src/safety/`、`src/app/foc_isr.c`、`safety_task.c` |
| 状态与功率输出的两阶段使能、故障上报重试、Flash 维护互斥 | `src/motor/drive_state_machine.c`、`src/safety/safety_manager.c` |
| 位置传感器选择、健康状态、原子反馈快照和标定 | `src/hal/encoder/`、`src/config/encoder_calibration_*` |
| 类型安全参数访问、绑定校验、恢复与失败回滚 | `src/config/parameter_*`、`runtime_settings.*` |
| CAN 帧检查、发送完成票据、协议和遥测边界 | `src/hal/bsp/board_can.*`、`src/comm/`、`src/app/communication_task.c` |
| FOC 非有限值处理、电压限幅、补偿单位与命令历史 | `algorithm/`、`src/foc/` |
| 真实 Flash/SRAM/CCM 分区、启动清零和 OTA 镜像头校验 | `platform/`、`src/boot/`、`scripts/patch_app_header.py` |

位置传感器边界放在已有 `src/hal/encoder/`；参数适配放在 `src/config/`。CAN 继续直接使用现有 BSP，没有恢复通用 transport 层或多层 settings 目录。已按此前范围移除的 X-STAR-S 板型不恢复。

## 验证

实际构建、回归测试、内存用量与尚待上板的事项见 [构建验证记录](BUILD_GUIDE.md#最新验证)。合并保留两边的设计意图，不再用“目录更简单”推断功能领先。
