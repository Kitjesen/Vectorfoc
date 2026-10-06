# 电机状态与标定

- [motor_runtime.c](motor_runtime.c)：唯一的状态门控控制入口 `Motor_RunControlCycle`；[motor_defaults.c](motor_defaults.c) 集中保存初始参数。
- [drive_state_machine.c](drive_state_machine.c)：使能、校准、停机和故障转换；[safety](../safety) 提供保护检查。
- [current_calibration.c](current_calibration.c)：启动被动零偏累加，以及显式校准流程。
- [resistance_inductance_calibration.c](resistance_inductance_calibration.c)：电阻、电感及编码器校准的步骤编排，电阻/电感步骤为文件私有函数。
- [encoder_calibration.c](encoder_calibration.c)、[flux_calibration.c](flux_calibration.c)：编码器与磁链校准。
- [calibration_state.c](calibration_state.c)：各校准过程共享的状态。

上电只采样。零偏就绪、无活动故障且收到显式命令后才可使能；清故障后保持未使能。
`Brake` 会主动导通低侧，不能代替关闭输出。
控制入口与时序见 [固件阅读路线](../../README.md#从哪里读代码)。
