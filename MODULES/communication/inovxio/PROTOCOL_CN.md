# Inovxio 私有协议文档

本文档描述了 FalconFoc 固件中使用的私有通信协议（Inovxio / Robstride 兼容）。

## 1. CAN 帧结构 (29-bit Extended Frame)

协议完全基于 CAN 2.0B 扩展帧格式。29 位 ID 被划分为三个主要部分。

### 全局 ID 字段定义
| Bit Range | Bin Width | Field Name | Description |
| :---: | :---: | :--- | :--- |
| **[28:24]** | 5 bits | **Command (CMD)** | 命令类型编号 (0~31) |
| **[23:8]** | 16 bits | **Data / Info** | 辅助数据区（如前馈力矩、状态位），若未定义则填 0 |
| **[7:0]** | 8 bits | **Target ID** | 目标设备 ID (1~127)，主机 ID 通常为 0xFD |

---

## 2. 命令列表

| CMD ID (Dec/Hex) | 宏定义名称 | 方向 |Payload DLC| 说明 |
| :--- | :--- | :--- | :--- | :--- |
| **0** (0x00) | `PRIVATE_CMD_GET_ID` | Host -> Motor | 0 | 获取设备 ID (支持广播) |
| **1** (0x01) | `PRIVATE_CMD_MOTOR_CTRL` | Host -> Motor | 8 | 运控命令 (MIT 模式) |
| **2** (0x02) | `PRIVATE_CMD_MOTOR_FEEDBACK` | Motor -> Host | 8 | 电机状态反馈 (100Hz 自动上报) |
| **3** (0x03) | `PRIVATE_CMD_MOTOR_ENABLE` | Host -> Motor | 0 | 电机闭环使能 |
| **4** (0x04) | `PRIVATE_CMD_MOTOR_STOP` | Host -> Motor | 0 | 电机停止/失能 |
| **6** (0x06) | `PRIVATE_CMD_SET_ZERO` | Host -> Motor | 0 | 设置当前位置为机械零点 |
| **7** (0x07) | `PRIVATE_CMD_SET_ID` | Host -> Motor | 4 | 修改 CAN ID |
| **8** (0x08) | `PRIVATE_CMD_CALIBRATE` | Host -> Motor | 1 | 触发电机校准 |
| **11** (0x0B) | `PRIVATE_CMD_RESET` | Host -> Motor | 0 | 系统软复位 |
| **12** (0x0C) | `PRIVATE_CMD_CLEAR_FAULT` | Host -> Motor | 0 | 清除故障状态 |
| **17** (0x11) | `PRIVATE_CMD_PARAM_READ` | Host -> Motor | 2 | 读取参数 |
| **18** (0x12) | `PRIVATE_CMD_PARAM_WRITE` | Host -> Motor | 8 | 写入参数 |
| **21** (0x15) | `PRIVATE_CMD_FAULT` | Motor -> Host | 4 | 严重故障主动上报 |
| **22** (0x16) | `PRIVATE_CMD_SAVE` | Host -> Motor | 0 | 保存当前参数到 Flash |
| **24** (0x18) | `PRIVATE_CMD_REPORT` | Host -> Motor | 1 | 控制自动反馈开关 (Start/Stop Report) |
| **25** (0x19) | `PRIVATE_CMD_SET_PROTOCOL` | Host -> Motor | 1 | 切换通信协议 |
| **26** (0x1A) | `PRIVATE_CMD_GET_VERSION` | Host -> Motor | 0 | 获取固件版本 |

---

## 3. 命令详细定义

### 3.0 获取 ID (CMD: 0)
获取设备 ID 和唯一标识符 (UUID)。支持广播查询。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x00`
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID` 或 `0x7F` (广播)

*   **数据域 (Data Payload)**: DLC 0 (无数据)

*   **响应 (Response)**: ID=`0x00...` DLC 8
    *   **Data Payload**:
    
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **UID[0]** | **UID[1]** | **UID[2]** | **UID[3]** | **UID[4]** | **UID[5]** | **UID[6]** | **UID[7]** |
| STM32 | Unique | ID | (Lower | 8 | Bytes) | | |

### 3.1 运动控制帧 (CMD: 1)
用于向电机发送控制指令（MIT 阻抗控制模式）。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x01`
    *   **[23:8] Info**: `Torque_FF` (Uint16) - 前馈力矩
        *   范围: `0 ~ 65535` 对应 `-120.0 ~ +120.0 Nm`
    *   **[7:0] Target**: `Motor_ID` (目标电机ID)

*   **数据域 (Data Payload)**: DLC 8 (Big Endian)

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| Pos H | Pos L | Vel H | Vel L | Kp H | Kp L | Kd H | Kd L |
| **Position** | **Velocity** | **Kp** | **Kd** |
| -12.57~+12.57 | -15~+15 | 0~500 | 0~100 |

*   **响应 (Response)**:
    *   无直接响应。触发内部状态更新。若开启 CMD 24，则通过 CMD 2 持续上报。

### 3.2 状态反馈帧 (CMD: 2)
电机启用自动上报后，周期性返回此帧。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x02`
    *   **[23:8] Info**: 状态标志位
        *   **[23:22] Mode**: 运行模式 (0:Reset, 2:Motor)
        *   **[21:16] Fault**: 故障位 (6 bits)
        *   **[15:8] SourceID**: 电机当前 ID
    *   **[7:0] Target**: `0xFD` (主机ID)

*   **数据域 (Data Payload)**: DLC 8 (Big Endian)

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| Pos H | Pos L | Vel H | Vel L | Tor H | Tor L | Temp H | Temp L |
| **Position** | **Velocity** | **Torque** | **Temp** (*10) |

*   **响应 (Response)**: N/A

### 3.3 电机使能 (CMD: 3)
进入闭环控制状态。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x03`
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 0 (无数据)

*   **响应 (Response)**: 无。

### 3.4 电机停止 (CMD: 4)
进入空闲/失能状态。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x04`
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 0 (无数据)

*   **响应 (Response)**: 无。

### 3.5 设置零点 (CMD: 6)
将当前机械位置设置为零点。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x06`
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 0 (无数据)

*   **响应 (Response)**: 无。

### 3.6 设置 ID (CMD: 7)
修改电机的 CAN 通信 ID。注意：ID 必须发给旧 ID 才能生效。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x07`
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Old_ID` (当前旧ID)

*   **数据域 (Data Payload)**: DLC 4

| Byte 0 | Byte 1 | Byte 2 | Byte 3 |
| :---: | :---: | :---: | :---: |
| **New ID** | Reserved | Reserved | Reserved |
| 新 ID (1-127) | 0x00 | 0x00 | 0x00 |

*   **响应 (Response)**: 无。

### 3.7 触发校准 (CMD: 8)
*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x08`
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 1

| Byte 0 |
| :---: |
| **Type** |
| 1:电阻电感, 2:编码器 |

*   **响应 (Response)**: 无。

### 3.8 参数读取 (CMD: 17)
请求读取内部参数。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x11` (17)
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 2

| Byte 0 | Byte 1 |
| :---: | :---: |
| **Idx Low** | **Idx High** |
| 索引低8位 | 索引高8位 |

*   **响应 (Response)**: 参数读取响应帧 (CMD 18 格式, 但 ID=`0x12`)
    *   **[28:24] CMD**: `0x12`
    *   **Data Payload**:
    
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| Idx L | Idx H | 0x00 | 0x00 | Val 0 | Val 1 | Val 2 | Val 3 |
| Index | Little | Endian | | Float | Value | | |

### 3.9 参数写入 (CMD: 18)
写入内部参数。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x12` (18)
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 8

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Idx Low** | **Idx High** | 0x00 | 0x00 | **Val 0** | **Val 1** | **Val 2** | **Val 3** |
| 参数索引 | (小端) | 保留 | 保留 | Float[0] | Float[1] | Float[2] | Float[3] |

*   **响应 (Response)**: 无。

### 3.10 控制自动上报 (CMD: 24)
开启或关闭 100Hz 的状态主动上报 (Feedback)。

*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x18` (24)
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 1

| Byte 0 |
| :---: |
| **Enable** |
| 0:关闭, 1:开启 |

*   **响应 (Response)**: 开启/关闭 CMD 2 的发送。

### 3.11 切换协议 (CMD: 25)
切换 CAN 通信协议 (Inovxio / CANopen / MIT)。
*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x19` (25)
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 1

| Byte 0 |
| :---: |
| **Protocol Type** |
| 0: Inovxio |
| 1: CANopen |
| 2: MIT Cheetah |

*   **响应 (Response)**: 无。协议切换后立即生效并保存配置。

### 3.12 获取版本 (CMD: 26)
*   **帧 ID (Arbitration ID)**:
    *   **[28:24] CMD**: `0x1A` (26)
    *   **[23:8] Info**: `0x0000`
    *   **[7:0] Target**: `Motor_ID`

*   **数据域 (Data Payload)**: DLC 0 (无数据)

*   **响应 (Response)**: 版本信息帧
    *   **[28:24] CMD**: `0x1A`
    *   **Data Payload**:
    
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Major** | **Minor** | **Patch** | 0x00 | 0x00 | 0x00 | 0x00 | 0x00 |
| 主版本 | 次版本 | 修补号 | | | | | |
