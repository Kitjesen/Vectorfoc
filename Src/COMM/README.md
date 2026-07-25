# Communication Module

**Status**: Active — host/source tested; HIL CAN-bus validation pending

## 概述

本模块实现 VectorFOC 电机控制器的通信协议，支持 3 种协议：**Vector 私有协议**、**CANopen DS402** 和 **MIT Cheetah 协议**。

## 目录结构

```
Src/COMM/
├── executor/                # 指令执行器
│   ├── executor.h
│   └── executor.c           # 状态机、参数和目标业务逻辑
├── manager/                 # 协议路由与接收队列
│   ├── manager.h/c
│   └── protocol_types.h     # portable 共享类型
├── protocol/
│   ├── vector/              # Vector 私有协议
│   ├── canopen/             # CANopen DS402
│   └── mit/                 # MIT Cheetah 协议
└── transport/               # 通用 CAN transport，向下适配 BSP
    ├── transport.h
    └── can_transport.h/c
```

## 协议说明

### 1. Vector 私有协议 (`protocol/vector/`)

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
| 13 | 进入Bootloader | VectorFOC 板在 ACK 的 Tx event 确认后请求进入 Bootloader；X-STAR-S 返回 `unsupported` |
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
- 标准 8 字节 expedited SDO download（`0x2F`/`0x2B`/`0x23`），成功响应
  `0x60`，错误响应 `0x80 + abort code`
- RPDO1: `controlword[0:1] + mode[2] + reserved[3] + target[4:7]`
- 位置、速度分别使用可配置的 units/rad；`6071h` 目标转矩使用转矩限制的千分比
- Heartbeat（仅 CANopen 模式发送）
- Emergency

NMT `STOP` 后节点只处理 NMT；RPDO1 与 SDO download 会被忽略，不会改变电机命令或重新上电。

### 3. MIT Cheetah协议 (`mit/`)

**特点**: 轻量级,适用于机器人关节控制

**命令格式**: 紧凑型数据打包,12字节定长

## 使用方法

### 初始化

```c
#include "manager.h"

void System_Init(void) {
    // 初始化为默认协议
    Protocol_Init(PROTOCOL_VECTOR);
    
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
    
    CAN_Frame frame = {0};
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

Vector `GET_ID` 快路径只接受 29-bit extended frame；标准帧、RTR 帧和不完整帧不会被该路径接受或用于喂 CAN watchdog。

## 共享类型 (`manager/protocol_types.h`)

**定义的核心类型**:
```c
// 协议类型枚举
typedef enum {
    PROTOCOL_VECTOR,     // Vector 私有协议
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
│   Executor Layer     │  executor/
└──────────────────────┘
          ↑
┌──────────────────────┐
│   Manager Layer      │  manager/
└──────────────────────┘
          ↑
┌──────────────────────┐
│   Protocol Layer     │  protocol/vector, canopen, mit
└──────────────────────┘
          ↑
┌──────────────────────┐
│   Transport Layer    │  transport/can_transport.c
└──────────────────────┘
          ↑
┌──────────────────────┐
│   BSP CAN            │  HAL/bsp/bsp_can.c
└──────────────────────┘
          ↑
       STM32 HAL/FDCAN
```

**各层职责**:
- **Transport**: 将 portable CAN frame 适配到 BSP CAN，并管理发送票据
- **BSP CAN**: 配置过滤器、中断回调和物理收发
- **Protocol**: 翻译字节流为标准指令 (`MotorCommand`)
- **Manager**: 路由管理, 协议切换, 接收队列
- **Executor**: 执行指令 (状态机/参数/目标更新)

### 模块独立

- **protocol/vector/**: 独立实现，无依赖其他协议
- **canopen/**: 独立实现
- **mit/**: 独立实现
- **manager**: 只负责路由,不实现具体协议

### 命名规范

- **目录**: 小写协议名 (`vector/`, `canopen/`)
- **文件**: `<协议>_protocol.c/h`
- **函数**: `Protocol<协议>_<动词>()`（如 `ProtocolVector_Parse()`）

## 性能指标

| 指标 | 值 |
|------|-----|
| CAN速率 | 1Mbps |
| 反馈频率 | 1kHz (可配置) |
| 协议切换时间 | <1ms |
| 内存占用 | 以每个 Release 链接输出为准 |



