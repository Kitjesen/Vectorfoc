# CAN 与 USB 通信

[protocol_dispatcher.c](protocol_dispatcher.c) 持有有界接收队列、选择协议并统计收发；[command_executor.c](command_executor.c) 将解析后的命令交给电机状态机和参数接口。

| 实现 | 职责 |
| --- | --- |
| [protocol_vector.c](protocol_vector.c) | Vector 帧编解码；[协议字段](PROTOCOL_CN.md) |
| [protocol_canopen.c](protocol_canopen.c) | 现有 CANopen 编解码 |
| [protocol_mit.c](protocol_mit.c) | 现有 MIT 编解码 |
| [protocol_messages.h](protocol_messages.h) | 三种协议共享的内部命令、状态与帧类型 |

CAN ISR 只入队；[通信任务](../app/communication_task.c) 每 2 ms 消费队列、处理延迟保存与周期上报。
发送统一进入 `Protocol_SendFrame`，规范化经典 CAN 帧后调用 BSP。
USB/VOFA 在 [telemetry_vofa.c](telemetry_vofa.c) 保留独立的文本命令和波形接口，由 [调试任务](../app/diagnostics_task.c) 处理。FOC ISR 只将波形样本入队。

现有 GET_ID 两种回复、查询可能触发停机和参数类型转换差异仍保留，见 [调试记录](../../../docs/DEBUGGING.md)。
完整调用链见 [固件 README](../../README.md#从哪里读代码)。
