# 应用入口

完整阅读顺序与调用链见 [固件 README](../../README.md#从哪里读代码)。

| 入口 | 职责 |
| --- | --- |
| `application_init.c` | 初始化软件状态，武装 ADC，启动采样时基 |
| `foc_isr.c` | 20 kHz 采样、快速保护及控制；按分频更新 FSM |
| `safety_task.c` | 200 Hz 慢速保护、LED 与诊断心跳 |
| `communication_task.c` | 500 Hz CAN 收帧处理、延迟参数保存及周期上报 |
| `diagnostics_task.c` | USB/VOFA 调试输出 |
| `status_led.c` | 状态指示灯，由守护任务更新 |

通信任务直接拥有周期服务；没有额外的命令 service 转发层。
上电守护任务不请求运行，也不自动清故障。
