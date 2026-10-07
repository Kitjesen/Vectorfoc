# 算法核心

`algorithm/` 是可以脱离具体芯片编译的数值层。这里的文件不是按文件名顺序执行的；固件的执行顺序由 [`src/app/foc_isr.c`](../src/app/foc_isr.c) 和 [`src/foc/control_dispatcher.c`](../src/foc/control_dispatcher.c) 决定。

## 先看主逻辑

一次正常的闭环周期只保留主线；`Control_RunCurrentCycle` 内部再按控制模式选择外环和弱磁：

```text
ADC / 编码器采样 → 快速保护 → 前馈/齿槽补偿
  → Motor_RunControlCycle（状态门控）
  → Control_RunCurrentCycle（模式、外环、弱磁和电流环装配）
  → FOC_Algorithm_CurrentLoop
  → MHAL_PWM_SetDuty → SMO 观测器更新
```

`OPEN` 和 `VF` 模式会在模式分发处直接做电压注入，绕过电流 PI；故障、未使能和校准状态由 `Motor_RunControlCycle` 拦截，不会进入正常闭环。

如果只想理解主算法，按这个顺序读三个文件即可：

1. [`src/app/foc_isr.c`](../src/app/foc_isr.c)：采样、保护和周期调度。
2. [`src/foc/control_dispatcher.c`](../src/foc/control_dispatcher.c)：模式分发、外环触发、弱磁和 PWM 适配。
3. [`foc_current_loop.c`](foc_current_loop.c)：一次电流环内部的数学计算。

## 每个算法做什么

| 文件 | 所属环节 | 作用 | 是否每周期执行 |
| --- | --- | --- | --- |
| `foc_current_loop.c` | 电流环 | 把三相电流和角度变成 d/q 电压及三相占空比 | 是，20 kHz |
| `foc_transforms.c` | 电流环工具 | Clarke、Park、反 Park、SVPWM 和三角函数 | 被电流环调用 |
| `pid_controller.c` | 外环 / 电流 PI | 通用 PID；电流 PI 在 `foc_current_loop.c` 内，速度/位置 PID 在 `outer_control.c` | 由所在环节决定 |
| `ladrc_controller.c` | 速度外环 | PID 的可选替代，用于从速度误差生成电流参考 | 速度模式且启用时，5 kHz |
| `trapezoid_trajectory.c` | 位置斜坡 | 生成位置、速度、加速度轨迹 | 新目标时规划；20 kHz 求值和条件步进 |
| `command_limiter.c` | 指令入口 | 限制速度和力矩指令变化率 | 20 kHz |
| `torque_feedforward.c` | 力矩补偿 | 根据速度变化率和摩擦估计补偿力矩 | 5 kHz |
| `field_weakening.c` | 电压裕量 | 电压饱和时生成负 d 轴电流，给电流环让出电压空间 | 20 kHz |
| `sliding_mode_observer.c` | 状态观测 | 用电流、电压估算角度和速度；当前在 FOC 输出后更新 | 20 kHz |
| `src/foc/outer_control.c` | 固件外环适配 | 位置 PID 生成速度参考，速度 PID/LADRC 生成电流参考 | 位置 1 kHz，速度 5 kHz |
| `src/foc/cogging_compensation.c` | 固件补偿适配 | 查表补偿齿槽力矩，并管理标定与 Flash 数据 | 5 kHz |

齿槽补偿暂时没有放进这个目录，因为它还负责标定、Flash 数据和电机状态，运行时实现位于 [`src/foc/cogging_compensation.c`](../src/foc/cogging_compensation.c)。同理，`outer_control.c` 接收数值快照并使用固件的调度周期；`MOTOR_DATA` 到快照的映射在 `control_dispatcher.c` 中完成。

## 核心接口

电流环唯一的纯算法入口是：

```c
FOC_Algorithm_CurrentLoop(&input, &config, &state, &output);
```

调用者提供电流、电角度、母线电压、参考值和采样周期；算法返回中间量和三相占空比，不访问硬件。

`SVPWM_Modulate` 的 `Valpha/Vbeta` 与 `Vbus` 都使用伏特，调制时直接用
`Vphase / Vbus` 归一化。返回值为 `0`（线性区）、`1`（超出线性区后已缩放）或
`-1`（非有限值、无效母线或无效输出指针）；电流环据此记录 `overmodulation`。

## 单独验证

```bash
cmake -S tests -B build/host -G Ninja
cmake --build build/host --parallel 4
ctest --test-dir build/host --output-on-failure
```

具体芯片、传感器和功率级只在 `src/hal` 及其上层固件适配中出现。

速度、位置和轨迹使用圈与圈/秒；惯量补偿把加速度乘以 2π 后得到 Nm，再通过力矩常数换算为 A。轨迹模式已有惯量前馈，运行时补偿只追加摩擦项。补偿保存在独立状态中，不回写用户命令。
