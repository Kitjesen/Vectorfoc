# APP 层任务架构

## 目录结构

```
APP/
├── init/                     # 初始化层
│   ├── app_init.c           # 统一初始化实现
│   └── app_init.h           # 初始化接口
│
├── rtos/                     # RTOS 任务层
│   ├── rtos_tasks.h         # 任务接口声明
│   ├── task_debug.c         # USB/VOFA调试 (1kHz)
│   ├── task_guard.c         # 安全监控 (200Hz)
│   ├── task_comm.c          # CAN通信 (500Hz)
│   ├── cmd_service.c        # 命令处理服务
│   └── cmd_service.h
│
├── isr/                      # 中断层
│   ├── isr_foc.c            # FOC核心 (20kHz)
│   └── isr_foc.h
│
├── robot.c                   # 兼容层 (调用 App_Init)
└── robot.h
```

---

## 任务列表

| 任务 | 文件 | 频率 | 功能 |
|------|------|------|------|
| defaultTask | `task_debug.c` | 1kHz | USB/VOFA调试输出 |
| guardTask | `task_guard.c` | 200Hz | 安全监控+LED |
| customTask | `task_comm.c` | 500Hz | CAN通信+状态上报 |

---

## 中断

| 中断源 | 文件 | 频率 | 功能 |
|--------|------|------|------|
| ADC1注入完成 | `isr_foc.c` | 20kHz | FOC实时控制 |

---

## 初始化流程

```
main() → App_Init()
           ├── BSP (DWT, Log, ADC, PWM)
           ├── Safety & Detection
           ├── Parameter System
           ├── CAN & Protocol
           ├── State Machine
           └── Motor
```

---

## 设计原则

1. **分层架构**：init / rtos / isr 明确分离
2. **硬实时分离**：FOC在ISR，通信/监控在RTOS
3. **单一职责**：每个文件只做一件事
