# safety 模块

电机安全与故障检测。

- `fault_def.h`: 故障码与默认阈值
- `fault_detection.c/.h`: 物理量检测与故障判定
- `safety_control.c/.h`: 故障上报与状态机联动

建议在高频中断执行快速检测，在低频任务执行慢速检测。
