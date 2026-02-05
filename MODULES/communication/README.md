# Communication Module

**Status**: ✅ Active - Production Ready

## 概述

本模块实现VectorFOC电机控制器的通信协议,支持3种主流协议:**Inovxio私有协议**、**CANopen DS402**和**MIT Cheetah协议**。

## 目录结构

```
communication/
├── manager.h/c              # 协议管理器(路由分发)
├── protocol_types.h         # 共享数据类型定义
│
├── executor/                # 指令执行器 [NEW]
│   ├── executor.h
│   └── executor.c           # 业务逻辑(状态机/参数/目标)
│
├── inovxio/                 # Inovxio私有协议
│   ├── inovxio_protocol.h
│   └── inovxio_protocol.c
│
├── canopen/                 # CANopen DS402
│   ├── canopen_protocol.h
│   └── canopen_protocol.c
│
└── mit/                     # MIT Cheetah协议
    ├── mit_protocol.h
    └── mit_protocol.c
```

## 协议说明

### 1. Inovxio私有协议 (`inovxio/`)

**特点**: CAN 2.0 Extended Frame (29-bit ID),高性能运控

**命令列表**:
| Cmd | 功能 | 说明 |
|-----|------|------|
| 0 | 获取设备ID | 返回CAN ID和UUID |
| 1 | 电机控制 | 位置/速度/力矩混合控制 |
| 2 | 电机反馈 | 周期性或请求反馈 |
| 3 | 电机使能 | 进入Operation Enabled状态 |
| 4 | 电机停止 | 禁用电机 |
| 6 | 设置机械零点 | 当前位置设为零点 |
| 7 | 设置CAN ID | 更新节点ID |
| 8 | 触发校准 | Data[0]: 0=Full, 1=RL, 2=Enc |
| 11 | 系统复位 | 软复位MCU |
| 12 | 清除故障 | 恢复到IDLE状态 |
| 13 | 进入Bootloader | 跳转到系统Bootloader (待完善) |
| 17 | 读取参数 | 读单个参数 |
| 18 | 写入参数 | 写单个参数 |
| 21 | 故障反馈 | 返回故障和警告 |
| 22 | 保存参数 | 保存到Flash |
| 25 | 切换协议 | 切换到其他协议 |
| 26 | 获取版本 | 返回固件版本号 |

**ID结构** (29-bit):
```
Bit 28-24: Command Type
Bit 23-8:  Data Area 2 (Host ID/Sub-index)
Bit 7-0:   Target Node ID
```

### 2. CANopen DS402 (`canopen/`)

**特点**: 标准工业协议,兼容性强

**支持对象**:
- NMT (Network Management)
- SDO (Service Data Object)
- PDO (Process Data Object)
- Heartbeat
- Emergency

### 3. MIT Cheetah协议 (`mit/`)

**特点**: 轻量级,适用于机器人关节控制

**命令格式**: 紧凑型数据打包,12字节定长

## 使用方法

### 初始化

```c
#include "communication/manager.h"

void System_Init(void) {
    // 初始化为默认协议
    Protocol_Init(PROTOCOL_INOVXIO);
    
    // 或动态切换
    Protocol_SetType(PROTOCOL_CANOPEN);
}
```

### 接收处理

```c
void CAN_RxCallback(CAN_Frame *frame) {
    // 协议管理器自动路由到对应协议
    Protocol_ProcessRxFrame(frame);
}
```

### 发送反馈

```c
void SendFeedback(void) {
    MotorStatus status = {
        .position = motor.position,
        .velocity = motor.velocity,
        .torque = motor.torque
    };
    
    CAN_Frame frame;
    if (Protocol_BuildFeedback(&status, &frame)) {
        Protocol_SendFrame(&frame);
    }
}
```

## 协议管理器 (`manager.h/c`)

**职责**: 统一路由和协议切换

**核心API**:
```c
void Protocol_Init(ProtocolType type);
void Protocol_SetType(ProtocolType type);
ProtocolType Protocol_GetType(void);
ParseResult Protocol_ParseFrame(const CAN_Frame*, MotorCommand*);
bool Protocol_BuildFeedback(const MotorStatus*, CAN_Frame*);
```

## 共享类型 (`types.h`)

**定义的核心类型**:
```c
// 协议类型枚举
typedef enum {
    PROTOCOL_INOVXIO,    // Inovxio(穹沛)私有协议
    PROTOCOL_CANOPEN,    // CANopen DS402
    PROTOCOL_MIT         // MIT Cheetah
} ProtocolType;

// CAN帧
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
} CAN_Frame;

// 电机命令
typedef struct {
    float position_ref;
    float velocity_ref;
    float torque_feedforward;
    // ...
} MotorCommand;

// 电机状态
typedef struct {
    float position;
    float velocity;
    float torque;
    // ...
} MotorStatus;
```

## 添加新协议

### Step 1: 创建协议目录

```bash
mkdir modbus
```

### Step 2: 实现协议接口

```c
// modbus/modbus_protocol.h
#ifndef MODBUS_PROTOCOL_H
#define MODBUS_PROTOCOL_H

#include "../types.h"

ParseResult Modbus_ParseFrame(const CAN_Frame*, MotorCommand*);
bool Modbus_BuildResponse(const MotorStatus*, CAN_Frame*);

#endif
```

### Step 3: 注册到管理器

```c
// manager.c
#include "modbus/modbus_protocol.h"

// 在Protocol_ParseFrame中添加:
case PROTOCOL_MODBUS:
    return Modbus_ParseFrame(frame, cmd);
```

## 设计原则

### 分层架构

```
┌──────────────────────┐
│   Motor Control      │  FOC/FSM
└──────────────────────┘
          ↑
┌──────────────────────┐
│   Executor Layer     │  executor.c (执行业务逻辑)
└──────────────────────┘
          ↑
┌──────────────────────┐
│   Manager Layer      │  manager.c (路由与调度)
└──────────────────────┘
          ↑
┌──────────────────────┐
│   Protocol Layer     │  inovxio/ canopen/ mit/ (翻译字节流)
└──────────────────────┘
          ↑
┌──────────────────────┐
│   Transport Layer    │  BSP/can/bsp_can.c (硬件驱动)
└──────────────────────┘
```

**各层职责**:
- **Transport**: 硬件IO, 只负责收发字节
- **Protocol**: 翻译字节流为标准指令 (`MotorCommand`)
- **Manager**: 路由管理, 协议切换, 接收队列
- **Executor**: 执行指令 (状态机/参数/目标更新)

### 模块独立

- **inovxio/**: 独立实现,无依赖其他协议
- **canopen/**: 独立实现
- **mit/**: 独立实现
- **manager**: 只负责路由,不实现具体协议

### 命名规范

- **目录**: 小写协议名 (`mineru/`, `canopen/`)
- **文件**: `<协议>_protocol.c/h`
- **函数**: `<协议>_<动词>()` (如`MinerU_ParseFrame()`)

## 性能指标

| 指标 | 值 |
|------|-----|
| CAN速率 | 1Mbps |
| 反馈频率 | 1kHz (可配置) |
| 协议切换时间 | <1ms |
| 内存占用 | ~5KB (所有协议) |



