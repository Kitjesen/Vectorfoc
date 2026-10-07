# 固件控制适配

这里保留和固件运行时绑定的控制调度。纯算法已经移到 [`algorithm/`](../../algorithm/)，但执行顺序从这里开始看更清楚：先看中断调度，再看模式分发，最后进入纯电流环。

| 文件 | 职责 |
| --- | --- |
| [control_dispatcher.c](control_dispatcher.c) | 模式选择、限幅、外环分频、弱磁和电流环适配 |
| [outer_control.c](outer_control.c) | 接收 `OuterLoopInput` 快照，独立更新位置环（1 kHz）和速度环（5 kHz） |
| [runtime_compensation.c](runtime_compensation.c) | 前馈与弱磁的运行时适配 |
| [cogging_compensation.c](cogging_compensation.c) | 齿槽补偿 |
| [observer_adapter.c](observer_adapter.c) | 使用运行时电流和电压数据的观测器适配 |

主链路只看两个固件入口；`Control_RunCurrentCycle` 内部完成模式、外环、弱磁和电流环装配：

```text
HAL_ADCEx_InjectedConvCpltCallback (20 kHz)
  → 采样 / 保护
  → 前馈、齿槽补偿 (5 kHz)
  → Motor_RunControlCycle（状态门控）
  → Control_RunCurrentCycle（单个控制步）
  → FOC_Algorithm_CurrentLoop (20 kHz)
  → MHAL_PWM_SetDuty
  → SMO_Observer_Update
```

`control_dispatcher.c` 不实现 FOC 数学，只在 `Control_RunOuterLoop()` 处完成一次 `MOTOR_DATA` 到 `OuterLoopInput/Output` 的映射；位置环和速度环各自到期时分别调用 `Control_UpdatePositionLoop()` 与 `Control_UpdateVelocityLoop()`。电流单位为 A，电压为 V，电角度为 rad，采样周期为 s，占空比为 0～1。

`Control_RunCurrentCycle` 的阅读顺序固定为四步：命令斜坡 → 模式参考值 → 位置/速度环调度 → 弱磁、电流环和 PWM。参数同步、外环计数与 PWM 死区补偿是同一文件内的私有步骤，不增加公共接口或跨目录转发。

该入口返回本周期是否完成有效控制。状态机先授予运行权限，写入首个有效占空比后再开启三相输出；发生故障或停机时取消使能意图。前馈、齿槽补偿和弱磁施加到本周期参考值副本，用户命令保留原值。

所有周期和分频数来自 [`motor_configuration.h`](../config/motor_configuration.h)，不在 ISR 或算法适配中重复定义。模式需要时才运行外环；位置环以 1 ms 生成速度参考，速度环以 200 μs 生成电流参考，电流环在两次外环更新之间保持上次参考值。
