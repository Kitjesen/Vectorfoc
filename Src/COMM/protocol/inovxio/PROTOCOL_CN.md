# Inovxio 私有协议文档

本文档按当前仓库实现对齐，覆盖以下代码路径：

- `Src/COMM/protocol/inovxio/inovxio_protocol.c`
- `Src/COMM/manager/manager.c`

如果代码和文档冲突，以代码实现为准；本文件的目标就是描述当前实际行为。

## 1. 基本约定

### 1.1 帧格式

Inovxio 协议使用 CAN 2.0B 29-bit 扩展帧：

| Bit Range | Width | Field | 说明 |
| --- | --- | --- | --- |
| `[28:24]` | 5 bits | `CMD` | 指令号 |
| `[23:8]` | 16 bits | `Info` | 附加信息字段 |
| `[7:0]` | 8 bits | `Target ID` | 目标节点 ID |

通用拼包方式：

```c
can_id = (cmd << 24) | (info << 8) | target_id;
```

### 1.2 节点约定

- 电机节点 ID：`1 ~ 127`
- 发现广播地址：`0x7F`
- 主机接收地址：`0xFD`
- `GET_ID` 响应地址：`0xFE`

### 1.3 当前实现约束

- `GET_ID` 不在 `inovxio_protocol.c` 中处理，而在 `manager.c` 中作为特殊发现帧处理。
- 除 `GET_ID` 外，当前实现只处理 `Target ID == 本机 CAN ID` 的 Inovxio 命令。
- `MOTOR_CTRL` 使用大端 `uint16` 编码。
- 参数索引在 `PARAM_READ` / `PARAM_WRITE` 中使用小端编码。
- 参数值在 `PARAM_WRITE` / 参数响应中按原始 `float` 内存字节拷贝。

## 2. 指令总表

| CMD | Hex | 名称 | 方向 | DLC | 说明 |
| ---: | ---: | --- | --- | ---: | --- |
| 0 | `0x00` | `PRIVATE_CMD_GET_ID` | Host -> Motor | 0 | 设备发现，支持广播 |
| 1 | `0x01` | `PRIVATE_CMD_MOTOR_CTRL` | Host -> Motor | 8 | MIT 风格控制帧 |
| 2 | `0x02` | `PRIVATE_CMD_MOTOR_FEEDBACK` | Motor -> Host | 8 | 状态反馈 |
| 3 | `0x03` | `PRIVATE_CMD_MOTOR_ENABLE` | Host -> Motor | 0 | 电机使能 |
| 4 | `0x04` | `PRIVATE_CMD_MOTOR_STOP` | Host -> Motor | 0 | 电机停止/失能 |
| 6 | `0x06` | `PRIVATE_CMD_SET_ZERO` | Host -> Motor | 0 | 设置当前位置为零点 |
| 7 | `0x07` | `PRIVATE_CMD_SET_ID` | Host -> Motor | 4 | 修改 CAN ID |
| 8 | `0x08` | `PRIVATE_CMD_CALIBRATE` | Host -> Motor | 0 或 1 | 触发校准 |
| 11 | `0x0B` | `PRIVATE_CMD_RESET` | Host -> Motor | 0 | 软件复位 |
| 12 | `0x0C` | `PRIVATE_CMD_CLEAR_FAULT` | Host -> Motor | 0 | 清故障 |
| 13 | `0x0D` | `PRIVATE_CMD_BOOTLOADER` | Host -> Motor | 0 | 请求进入 bootloader |
| 17 | `0x11` | `PRIVATE_CMD_PARAM_READ` | Host -> Motor | 2 | 读取参数 |
| 18 | `0x12` | `PRIVATE_CMD_PARAM_WRITE` | Host -> Motor | 8 | 写入参数 |
| 21 | `0x15` | `PRIVATE_CMD_FAULT` | Motor -> Host | 8 | 故障上报 |
| 22 | `0x16` | `PRIVATE_CMD_SAVE` | Host -> Motor | 0 | 保存参数到 Flash |
| 23 | `0x17` | `PRIVATE_CMD_SET_BAUDRATE` | Host -> Motor | 1 | 修改 CAN 波特率参数 |
| 24 | `0x18` | `PRIVATE_CMD_REPORT` | Host -> Motor | 1 | 开关自动上报 |
| 25 | `0x19` | `PRIVATE_CMD_SET_PROTOCOL` | Host -> Motor | 1 | 切换协议 |
| 26 | `0x1A` | `PRIVATE_CMD_GET_VERSION` | Host -> Motor | 0 | 获取固件版本 |
| 30 | `0x1E` | `PRIVATE_CMD_FAULT_QUERY` | Host -> Motor | 0 | 查询故障详情 |

## 3. 指令详细说明

### 3.0 `GET_ID` (`CMD = 0`)

说明：

- 由 `manager.c` 特殊处理。
- 支持定向查询和广播查询。
- 非目标节点会直接忽略。

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x00` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID 或 `0x7F` |
| `DLC` | `0` |

响应：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x00` |
| `Info` | `[15:8] = Source CAN ID` |
| `Target ID` | `0xFE` |
| `DLC` | `8` |

Payload：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| UID0 | UID1 | UID2 | UID3 | UID4 | UID5 | UID6 | UID7 |

说明：

- 当前实现发送 STM32 UID 的前 8 字节，即 `word0 + word1`。

### 3.1 `MOTOR_CTRL` (`CMD = 1`)

说明：

- 当前实现将其解析为 `CONTROL_MODE_MIT`。
- 命令被解析后会同时带上“有效使能命令”，即发送控制帧即视为使能。

请求 ID：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x01` |
| `Info` | `Torque_FF`，`uint16` |
| `Target ID` | 本机 ID |

Payload，DLC = `8`：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Pos H | Pos L | Vel H | Vel L | Kp H | Kp L | Kd H | Kd L |

编码范围：

| 字段 | 编码 | 实际范围 |
| --- | --- | --- |
| `Torque_FF` | `uint16` | `-120.0 ~ 120.0 Nm` |
| `Position` | `uint16` | `-12.57 ~ 12.57 rad` |
| `Velocity` | `uint16` | `-15.0 ~ 15.0 rad/s` |
| `Kp` | `uint16` | `0.0 ~ 500.0` |
| `Kd` | `uint16` | `0.0 ~ 100.0` |

### 3.2 `MOTOR_FEEDBACK` (`CMD = 2`)

说明：

- 由 `ProtocolPrivate_BuildFeedback()` 构帧。
- 当前实现固定将 `Mode` 字段置为 `2`。

响应 ID：

| Bit | 含义 |
| --- | --- |
| `[28:24]` | `0x02` |
| `[23:22]` | `Mode = 2` |
| `[21:16]` | 故障摘要位 |
| `[15:8]` | Source CAN ID |
| `[7:0]` | `0xFD` |

故障摘要位映射：

| Bit | 条件 |
| ---: | --- |
| 5 | `FAULT_ENCODER_UNCALIBRATED` |
| 4 | `FAULT_STALL_OVERLOAD` |
| 3 | `FAULT_HARDWARE_ID` |
| 2 | `FAULT_OVER_TEMP` |
| 1 | `FAULT_CURRENT_A/B/C` 任一置位 |
| 0 | `FAULT_UNDER_VOLTAGE` |

Payload，DLC = `8`：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Pos H | Pos L | Vel H | Vel L | Tor H | Tor L | Temp H | Temp L |

编码范围：

| 字段 | 编码 | 实际范围 |
| --- | --- | --- |
| `Position` | `uint16` | `-12.57 ~ 12.57 rad` |
| `Velocity` | `uint16` | `-15.0 ~ 15.0 rad/s` |
| `Torque` | `uint16` | `-120.0 ~ 120.0 Nm` |
| `Temperature` | `int16` | 摄氏度乘以 `10` |

### 3.3 `MOTOR_ENABLE` (`CMD = 3`)

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x03` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

效果：

- 设置 `has_enable_command = true`
- 设置 `enable_motor = true`

### 3.4 `MOTOR_STOP` (`CMD = 4`)

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x04` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

效果：

- 设置 `has_enable_command = true`
- 设置 `enable_motor = false`

### 3.5 `SET_ZERO` (`CMD = 6`)

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x06` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

效果：

- 设置 `cmd->set_zero = true`

### 3.6 `SET_ID` (`CMD = 7`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x07` |
| `Info` | `0x0000` |
| `Target ID` | 当前节点 ID |
| `DLC` | `4` |

Payload：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 |
| --- | --- | --- | --- |
| New ID | Reserved | Reserved | Reserved |

效果：

- 调用 `Param_WriteUint8(PARAM_CAN_ID, data[0])`
- 调用 `Param_ScheduleSave()`

### 3.7 `CALIBRATE` (`CMD = 8`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x08` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` 或 `1` |

Payload：

| Byte 0 |
| --- |
| `Type` |

说明：

- 当 `DLC == 0` 时，当前实现默认 `type = 0`
- 当 `DLC >= 1` 时，将 `data[0]` 原样传给 `Motor_RequestCalibration()`
- 当前协议层不校验 `type` 枚举值

### 3.8 `RESET` (`CMD = 11`)

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x0B` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

效果：

- 调用 `HAL_NVIC_SystemReset()`

### 3.9 `CLEAR_FAULT` (`CMD = 12`)

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x0C` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

效果：

- 调用 `Safety_ClearFaults(&g_ds402_state_machine)`
- 调用 `Motor_ClearFaults(&motor_data)`

### 3.10 `BOOTLOADER` (`CMD = 13`)

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x0D` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

效果：

- 调用 `Boot_RequestUpgrade()`

### 3.11 `PARAM_READ` (`CMD = 17`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x11` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `2` |

Payload：

| Byte 0 | Byte 1 |
| --- | --- |
| Index Low | Index High |

说明：

- 参数索引按小端编码
- 当前实现要求最小 `DLC = 2`

响应：

由 `ProtocolPrivate_BuildParamResponse()` 构造：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x12` |
| `Info` | `[15:8] = Source CAN ID` |
| `Target ID` | `0xFD` |
| `DLC` | `8` |

Payload：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Index Low | Index High | `0x00` | `0x00` | Float0 | Float1 | Float2 | Float3 |

### 3.12 `PARAM_WRITE` (`CMD = 18`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x12` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `8` |

Payload：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Index Low | Index High | Reserved | Reserved | Float0 | Float1 | Float2 | Float3 |

说明：

- 参数索引小端编码
- 参数值按原始 `float` 字节拷贝
- 协议层只解析，不在此处直接执行写入

### 3.13 `FAULT` (`CMD = 21`)

说明：

- 由 `ProtocolPrivate_BuildFault()` 构帧
- 当前实现忽略 `warning_code`

响应：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x15` |
| `Info` | `[15:8] = Source CAN ID` |
| `Target ID` | `0xFD` |
| `DLC` | `8` |

Payload：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Fault[31:24] | Fault[23:16] | Fault[15:8] | Fault[7:0] | `0x00` | `0x00` | `0x00` | `0x00` |

### 3.14 `SAVE` (`CMD = 22`)

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x16` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

效果：

- 调用 `Param_ScheduleSave()`

### 3.15 `SET_BAUDRATE` (`CMD = 23`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x17` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `1` |

Payload：

| Byte 0 |
| --- |
| Baudrate Enum |

效果：

- 调用 `Param_WriteUint8(PARAM_CAN_BAUDRATE, data[0])`
- 调用 `Param_ScheduleSave()`

### 3.16 `REPORT` (`CMD = 24`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x18` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `1` |

Payload：

| Byte 0 |
| --- |
| `0` = disable, non-zero = enable |

效果：

- 调用 `CmdService_SetReportEnable(data[0] != 0)`

### 3.17 `SET_PROTOCOL` (`CMD = 25`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x19` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `1` |

Payload：

| Byte 0 |
| --- |
| Protocol Type |

当前编码：

| 值 | 协议 |
| ---: | --- |
| 0 | Inovxio |
| 1 | CANopen |
| 2 | MIT |

效果：

- 设置 `cmd->is_protocol_switch = true`
- 设置 `cmd->target_protocol = data[0]`

### 3.18 `GET_VERSION` (`CMD = 26`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x1A` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

响应：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x1A` |
| `Info` | `[15:8] = Source CAN ID` |
| `Target ID` | `0xFD` |
| `DLC` | `8` |

Payload：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Major | Minor | Patch | Dirty | Hash[0] | Hash[1] | Hash[2] | Hash[3] |

说明：

- `Dirty`：`0` 表示工作区干净，`1` 表示构建时仓库有未提交修改
- `Hash[0..3]`：`FW_GIT_HASH` 的前 4 个字符

### 3.19 `FAULT_QUERY` (`CMD = 30`)

请求：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x1E` |
| `Info` | `0x0000` |
| `Target ID` | 本机 ID |
| `DLC` | `0` |

协议层效果：

- 设置 `cmd->is_fault_query = true`

响应：

由 `ProtocolPrivate_BuildFaultDetail()` 构造：

| 字段 | 值 |
| --- | --- |
| `CMD` | `0x1E` |
| `Info` | `[15:8] = Source CAN ID` |
| `Target ID` | `0xFD` |
| `DLC` | `8` |

Payload：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Fault[15:8] | Fault[7:0] | Fault[31:24] | Fault[23:16] | Ts[31:24] | Ts[23:16] | Ts[15:8] | Ts[7:0] |

说明：

- `Ts` 为 `Safety_GetLastFaultTime()` 返回值，按大端写入

## 4. 测试状态

当前实现对应的 host tests 位于：

- `test/test_inovxio_protocol.c`
- `test/test_manager.c`

它们覆盖：

- `inovxio_protocol.c` 中全部已实现的命令分支
- `GET_ID` 的广播、自身定向、非目标节点忽略
- 响应帧构造，包括 `FEEDBACK` / `FAULT` / `PARAM_RESPONSE` / `FAULT_DETAIL`

## 5. 已知边界

- 本文档描述的是当前固件实现，不代表上位机 SDK 的所有历史兼容行为。
- 协议逻辑已做 host tests，但实板层仍需要验证：
  - 多节点 CAN 总线下的时序和仲裁
  - 波特率切换后的重连流程
  - bootloader 跳转链路
  - 故障触发与恢复链路
