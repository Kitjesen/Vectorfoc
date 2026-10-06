# 配置与参数

- [board_configuration.h](board_configuration.h)、[boards/](boards/)：选择板型及引脚、ADC/PWM 硬件常量。
- [motor_configuration.h](motor_configuration.h)：控制频率、限幅、保护阈值及编译期选项。

- [parameter_registry.c](parameter_registry.c)：40 个现有参数的 ID、类型、范围、默认值和变量地址。
- [parameter_access.c](parameter_access.c)：访问校验、延迟保存，以及共用的 34 字段 Flash 映射表。
- [parameter_storage.c](parameter_storage.c)：现用镜像的保存、读取和 CRC；字节布局由 [parameter_storage.h](parameter_storage.h) 定义。

电机运行初值在 [motor_defaults.c](../motor/motor_defaults.c)。USB 命令属于 `comm`，错误记录属于 `safety`，指示灯属于 `app`。

参数表有 39 项标为 persistent，但现有镜像只保存 34 项，包含 CAN_TIMEOUT 的直接读写特例；本次没有扩大保存范围。
类型转换及 Flash 页几何的现存问题见 [调试记录](../../../docs/DEBUGGING.md)。
Flash 保存留在任务上下文；FOC ISR 只把波形样本放入队列。
