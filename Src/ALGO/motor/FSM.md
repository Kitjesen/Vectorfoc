# 电机有限状态机 (FSM) 模块文档

## 概述
电机有限状态机 (Motor FSM) 模块实现了电机控制系统的核心状态管理逻辑。该模块严格遵循 **CiA 402 (DS402)** 驱动和运动控制标准配置文件，管理电机控制器的生命周期，处理初始化、就绪、运行使能及故障处理等状态之间的转换。

该模块采用**表驱动 (Table-Driven)** 架构，确保了逻辑的清晰度、可维护性和执行效率。

## 目录结构
- `fsm.h`: 核心状态定义、控制字/状态字联合体定义及公共 API 声明。
- `fsm.c`: 状态机逻辑实现，包含转换表、状态字映射表及底层硬件钩子。

## 核心概念

### 1. 标准遵循
本实现遵循 **CiA 402 Power State Machine** (电力驱动状态机)。
- **控制字 (Controlword, 6040h)**: 外部命令输入，用于触发状态转换（位掩码匹配）。
- **状态字 (Statusword, 6041h)**: 内部状态输出，反馈当前控制器状态给上层应用或通信接口。

### 2. 状态定义 (MotorState)
FSM 定义了以下状态，涵盖标准 DS402 状态及自定义扩展状态：

| 状态枚举值 | 描述 | 行为特徵 |
| :--- | :--- | :--- |
| `STATE_NOT_READY_TO_SWITCH_ON` | **未准备好切换** | 初始化过程中的临时状态，控制器正在进行自检。 |
| `STATE_SWITCH_ON_DISABLED` | **切换禁止** | 初始化完成，高压供电未就绪或被禁止。PWM 输出关闭。 |
| `STATE_READY_TO_SWITCH_ON` | **准备切换** | 高压供电已就绪，等待主站命令进行切换。 |
| `STATE_SWITCHED_ON` | **已切换** | 功率级已使能，但放大器尚未激活（无 PWM 输出）。 |
| `STATE_OPERATION_ENABLED` | **运行使能** | **核心运行状态**。电机已激活，PWM 输出开启，闭环控制执行中。 |
| `STATE_QUICK_STOP_ACTIVE` | **快速停止激活** | 正在执行快速停机程序（如减速停车）。 |
| `STATE_FAULT_REACTION_ACTIVE` | **故障反应激活** | 检测到故障，正在执行安全停机或保护动作。 |
| `STATE_FAULT` | **故障** | 故障状态，系统已锁定，需复位才能恢复。 |
| `STATE_CALIBRATING` | **校准中 (扩展)** | 自定义状态，用于执行电机参数辨识（电阻、电感检测）。 |

### 3. 表驱动逻辑
本尽可能避免复杂的嵌套 `if-else`，而是通过查找表定义行为：

#### 状态转换表 (Transition Table)
定义了 `(当前状态 + 控制字命令) -> 目标状态` 的映射。逻辑如下：
1. 遍历转换表。
2. 检查 `当前状态` 是否匹配。
3. 检查 `(控制字 & 掩码) == 目标值`。
4. 若匹配，则执行状态切换。

**主要转换路径包括：**
- **Shutdown**: 运行/已切换 -> 准备切换
- **Switch On**: 准备切换 -> 已切换
- **Enable Operation**: 已切换 -> 运行使能
- **Disable Voltage**: 任何状态 -> 切换禁止 (自由停车)
- **Quick Stop**: 运行使能 -> 快速停止
- **Fault Reset**: 故障 -> 切换禁止

#### 状态字映射表 (Status Table)
定义了每个 `当前状态` 对应的 `Statusword` 位模式。这确保了状态字始终与内部状态严格同步，符合 DS402 位定义。

## API 参考

### 初始化与循环

#### `void StateMachine_Init(StateMachine *sm)`
初始化状态机结构体，清零所有字段，并将初始状态置为 `STATE_NOT_READY_TO_SWITCH_ON`。

#### `void StateMachine_Update(StateMachine *sm)`
**核心更新函数**。应在电机控制的高优先级任务中周期性调用。
- 处理自动状态转换（如初始化完成自动跳转）。
- 解析当前的 `Controlword` 并尝试匹配转换表。
- 更新 `Statusword`。

### 控制与反馈

#### `void StateMachine_SetControlword(StateMachine *sm, uint16_t controlword)`
设置输入的控制字。通常由通信层（CAN/UART/USB）接收到主站 PDO 数据后调用。

#### `uint16_t StateMachine_GetStatusword(const StateMachine *sm)`
获取当前生成的状态字，用于填充返回给主站的 PDO 数据。

#### `MotorState StateMachine_GetState(const StateMachine *sm)`
获取当前的内部状态枚举值，供应用层逻辑判断使用。

### 手动控制与故障管理

#### `bool StateMachine_RequestState(StateMachine *sm, MotorState target_state)`
手动请求跳转到目标状态。
- **功能**: 内部自动计算所需的 `Controlword` 并应用，模拟外部命令。
- **用途**: 用于调试或非标准流程控制（如强制启动）。

#### `void StateMachine_EnterFault(StateMachine *sm, uint32_t fault_code)`
触发故障。
- 将状态强制切换到 `STATE_FAULT` (或经过 Reaction 状态)。
- 记录 `fault_code` 到故障历史环形缓冲区。
- 自动关闭 PWM 输出。

#### `bool StateMachine_ClearFault(StateMachine *sm)`
尝试清除故障状态。
- 相当于发送 `Fault Reset` 命令。
- 仅在当前处于 `STATE_FAULT` 且故障条件已移除时有效。

## 硬件抽象集成
FSM 模块控制着功率级的物理状态：
- **进入 `STATE_OPERATION_ENABLED`**: 调用 `MHAL_PWM_Enable()` 开启 PWM。
- **进入 `STATE_SWITCH_ON_DISABLED` 或 `STATE_FAULT`**: 调用 `MHAL_PWM_Disable()` 关闭 PWM，确保安全。

## 集成指南 (Integration Guide)

### 1. 周期性调用 (StateMachine_Update)
必须在主控制循环或高优先级定时器中断中调用 `StateMachine_Update`，以确保状态机能及时响应命令和自动流转。

**推荐位置**: `APP/motor_task/motor_task.c` -> `HAL_ADCEx_InjectedConvCpltCallback`
```c
// 在完成保护检测和核心控制后调用
StateMachine_Update(&g_ds402_state_machine);
```

### 2. 接收命令 (SetControlword)
当从 CAN 总线或串口收到新的控制帧时，应提取控制字并注入状态机。

**示例 (CAN回调):**
```c
void On_CAN_Message_Received(uint16_t control_word) {
    StateMachine_SetControlword(&g_ds402_state_machine, control_word);
}
```

### 3. 连接底层驱动
当前实现已通过 `hal_pwm.h` 连接了 `MHAL_PWM_Enable/Disable`。需确保这些底层函数已正确实现并映射到定时器硬件。
