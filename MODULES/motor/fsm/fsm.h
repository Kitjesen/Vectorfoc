#ifndef FSM_H
#define FSM_H

#include <stdbool.h>
#include <stdint.h>

/* 状态机状态定义 (基于CANopen DS402) */
typedef enum {
  STATE_NOT_READY_TO_SWITCH_ON = 0, // 未准备好切换
  STATE_SWITCH_ON_DISABLED,         // 切换禁止
  STATE_READY_TO_SWITCH_ON,         // 准备切换
  STATE_SWITCHED_ON,                // 已切换
  STATE_OPERATION_ENABLED,          // 运行使能
  STATE_QUICK_STOP_ACTIVE,          // 快速停止激活
  STATE_FAULT_REACTION_ACTIVE,      // 故障反应激活
  STATE_FAULT,                      // 故障

  /* 额外状态 (非DS402标准) */
  STATE_CALIBRATING, // 校准中
} MotorState;

/* 控制字 (CANopen 6040h) */
typedef union {
  uint16_t word;
  struct {
    uint16_t switch_on : 1;        // bit0: 切换
    uint16_t enable_voltage : 1;   // bit1: 使能电压
    uint16_t quick_stop : 1;       // bit2: 快速停止
    uint16_t enable_operation : 1; // bit3: 使能操作
    uint16_t reserved1 : 3;        // bit4-6: 保留
    uint16_t fault_reset : 1;      // bit7: 故障复位
    uint16_t halt : 1;             // bit8: 停止
    uint16_t reserved2 : 7;        // bit9-15: 保留
  } bits;
} Controlword;

/* 状态字 (CANopen 6041h) */
typedef union {
  uint16_t word;
  struct {
    uint16_t ready_to_switch_on : 1; // bit0: 准备切换
    uint16_t switched_on : 1;        // bit1: 已切换
    uint16_t operation_enabled : 1;  // bit2: 操作使能
    uint16_t fault : 1;              // bit3: 故障
    uint16_t voltage_enabled : 1;    // bit4: 电压使能
    uint16_t quick_stop : 1;         // bit5: 快速停止
    uint16_t switch_on_disabled : 1; // bit6: 切换禁止
    uint16_t warning : 1;            // bit7: 警告
    uint16_t manufacturer1 : 1;      // bit8: 制造商特定
    uint16_t remote : 1;             // bit9: 远程
    uint16_t target_reached : 1;     // bit10: 目标已达到
    uint16_t internal_limit : 1;     // bit11: 内部限制激活
    uint16_t reserved : 4;           // bit12-15: 保留
  } bits;
} Statusword;

/* 状态机结构 */
typedef struct {
  MotorState current_state;    // 当前状态
  MotorState target_state;     // 目标状态
  Controlword controlword;     // 控制字
  Statusword statusword;       // 状态字
  uint32_t state_entry_time;   // 进入当前状态的时间戳
  bool transition_in_progress; // 状态转换进行中标志
  uint32_t active_fault_code;  // 当前故障码
  uint16_t prev_controlword;   // 上一次的控制字 (用于边沿检测)
  bool (*pre_check_callback)(MotorState to_state); // 状态进入前检查回调
} StateMachine;

/**
 * @brief 初始化状态机
 * @param sm 状态机指针
 */
void StateMachine_Init(StateMachine *sm);

/**
 * @brief 更新状态机 (在主循环中调用)
 * @param sm 状态机指针
 */
void StateMachine_Update(StateMachine *sm);

/**
 * @brief 请求状态转换
 * @param sm 状态机指针
 * @param target_state 目标状态
 * @return true=请求成功, false=请求失败
 */
bool StateMachine_RequestState(StateMachine *sm, MotorState target_state);

/**
 * @brief 设置控制字
 * @param sm 状态机指针
 * @param controlword 控制字
 */
void StateMachine_SetControlword(StateMachine *sm, uint16_t controlword);

/**
 * @brief 获取状态字
 * @param sm 状态机指针
 * @return 状态字
 */
uint16_t StateMachine_GetStatusword(const StateMachine *sm);

/**
 * @brief 获取当前状态
 * @param sm 状态机指针
 * @return 当前状态
 */
MotorState StateMachine_GetState(const StateMachine *sm);

/**
 * @brief 进入故障状态
 * @param sm 状态机指针
 * @param fault_code 故障码
 */
void StateMachine_EnterFault(StateMachine *sm, uint32_t fault_code);

/**
 * @brief 清除故障
 * @param sm 状态机指针
 * @return true=成功, false=失败
 */
bool StateMachine_ClearFault(StateMachine *sm);

/**
 * @brief 设置状态预检查回调函数
 * @param sm 状态机指针
 * @param callback 回调函数指针
 */
void StateMachine_SetPreCheckCallback(StateMachine *sm,
                                      bool (*callback)(MotorState to_state));

#endif /* FSM_H */
