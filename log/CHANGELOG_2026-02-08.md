# VectorFOC 代码审查与修复记录

**日期**: 2026-02-08  
**范围**: FSM 状态机、通信层、FOC 算法、BSP 驱动、UI 调试接口  

---

## 一、FSM 状态机 (`fsm.h` / `fsm.c`)

### [Critical] 1. `prev_controlword` 边沿检测 Bug

**文件**: `Src/ALGO/motor/fsm.c` — `ProcessStateTransitions()`

**问题**: 故障复位 (Fault Reset) 依赖控制字 bit7 的上升沿检测，但 `prev_controlword` 仅在函数末尾更新。故障复位和标准转换两条路径都有 `return`，跳过了更新，导致边沿检测可能失效。

**修复**: 将 `sm->prev_controlword = cw` 移至函数开头，捕获旧值后立即更新。

```c
// Before (bug)
static void ProcessStateTransitions(StateMachine *sm) {
  uint16_t cw = sm->controlword.word;
  uint16_t prev_cw = sm->prev_controlword;
  if (sm->current_state == STATE_FAULT) {
    if (IsBitEdgeRising(prev_cw, cw, 0x0080))
      ExecuteTransition(sm, STATE_SWITCH_ON_DISABLED);
    return; // ← prev_controlword 未更新
  }
  // ...
  sm->prev_controlword = cw; // ← 仅到达此处才更新
}

// After (fix)
static void ProcessStateTransitions(StateMachine *sm) {
  uint16_t cw = sm->controlword.word;
  uint16_t prev_cw = sm->prev_controlword;
  sm->prev_controlword = cw; // ← 立即更新，所有路径覆盖
  // ...
}
```

---

### [Critical] 2. `EnterFault` 跳过 `FAULT_REACTION_ACTIVE`

**文件**: `Src/ALGO/motor/fsm.c` — `StateMachine_EnterFault()`

**问题**: 直接跳转到 `STATE_FAULT`，跳过了 DS402 标准要求的 `FAULT_REACTION_ACTIVE` 阶段（受控停机）。

**修复**: 先进入 `FAULT_REACTION_ACTIVE`（立即禁用 PWM），再由 `ProcessStateTransitions` 自动转换到 `FAULT`。

```c
// Before
ExecuteTransition(sm, STATE_FAULT);

// After
if (sm->current_state != STATE_FAULT &&
    sm->current_state != STATE_FAULT_REACTION_ACTIVE) {
  ExecuteTransition(sm, STATE_FAULT_REACTION_ACTIVE);
}
```

---

### [Critical] 3. `RequestState` 多步转换失效

**文件**: `Src/ALGO/motor/fsm.c` — `StateMachine_RequestState()`

**问题**: `RequestState(STATE_OPERATION_ENABLED)` 直接设 `cw=0x0F`，但从 `SWITCH_ON_DISABLED` 需要 `cw=0x06` (Shutdown) 才能前进到 `READY_TO_SWITCH_ON`。`0x0F & 0x0087 = 0x07 ≠ 0x06`，永远匹配不上，状态机卡死。

**修复**: 新增 `AutoAdvanceToTarget()` 机制 + `auto_advance` 标志。根据**当前状态**和**目标状态**自动计算每一步的中间控制字，每次 `Update` 推进一步。`SetControlword` 会自动禁用 auto_advance，保证外部 CAN 控制字优先。

```
SWITCH_ON_DISABLED ─(0x06)→ READY_TO_SWITCH_ON ─(0x07)→ SWITCHED_ON ─(0x0F)→ OPERATION_ENABLED
     Update #1                   Update #2                  Update #3
```

---

### [Medium] 4. `ClearFault` ISR 安全隐患

**文件**: `Src/ALGO/motor/fsm.c` — `StateMachine_ClearFault()`

**问题**: 旧实现在 `__disable_irq()` 环境中调用 `StateMachine_Update()` 两次模拟边沿，存在嵌套调用风险。

**修复**: 改为直接 `ExecuteTransition(sm, STATE_SWITCH_ON_DISABLED)`，简洁且 ISR 安全。

---

### [Medium] 5. `HandleStateEntry` 不完整

**文件**: `Src/ALGO/motor/fsm.c` — `HandleStateEntry()`

**问题**: 缺少 `QUICK_STOP_ACTIVE`、`FAULT_REACTION_ACTIVE`、`NOT_READY_TO_SWITCH_ON` 的处理。

**修复**: 
- `FAULT_REACTION_ACTIVE` → 立即 `MHAL_PWM_Disable()` 确保安全
- `QUICK_STOP_ACTIVE` → PWM 保持开启用于受控减速
- `NOT_READY_TO_SWITCH_ON` → `MHAL_PWM_Disable()`

---

### [Medium] 6. 故障历史不支持多实例

**文件**: `Src/ALGO/motor/fsm.c` / `fsm.h`

**问题**: `s_fault_history` 和 `s_fault_history_index` 是模块全局静态变量，不支持多个状态机实例。

**修复**: 移入 `StateMachine` 结构体作为 per-instance 成员。

```c
typedef struct {
  // ...
  uint32_t fault_history[FAULT_HISTORY_SIZE];
  uint8_t fault_history_index;
} StateMachine;
```

---

### [Minor] 7. 缺少 `STATE_COUNT` + 校准退出路径

**文件**: `Src/ALGO/motor/fsm.h` / `fsm.c`

**问题**: 枚举无边界标记，`RequestState` 无法做参数校验；校准状态只能通过 Disable Voltage 退出。

**修复**: 
- 枚举末尾增加 `STATE_COUNT`
- 转换表增加 `CALIBRATING → READY_TO_SWITCH_ON` (Shutdown 命令) 路径
- `RequestState` 增加 `target_state >= STATE_COUNT` 校验

---

## 二、通信层与执行器

### [Medium] 8. `executor.c` 多临界区竞态风险

**文件**: `Src/COMM/executor/executor.c`

**问题**: 控制模式切换（C 段）和设定值更新（D 段）分两个 `__disable_irq` 区间，中间窗口 ISR 可能读到模式已切换但设定值未更新的不一致状态。参数写入中每个 case 单独 disable/enable_irq，冗余且不优雅。

**修复**: 
- C+D 段合并为单一临界区
- 参数写入改为先在栈上确定 `new_mode`，再用单一临界区写入

---

## 三、FOC 算法

### [Minor] 9. 魔法数字

**文件**: `Src/ALGO/foc/foc_algorithm.c`

**问题**: 电压限幅使用硬编码 `0.57735f` 和 `0.95f`，可读性差。

**修复**: 替换为命名常量 `ONE_OVER_SQRT3` (1/√3) 和 `VOLTAGE_MARGIN` (5% 安全裕度)。

```c
#define ONE_OVER_SQRT3 0.57735026918962576f
#define VOLTAGE_MARGIN 0.95f

float V_max = input->Vbus * ONE_OVER_SQRT3 * VOLTAGE_MARGIN;
```

---

## 四、校准模块

### [Medium] 10. 磁链校准低速防护不足

**文件**: `Src/ALGO/motor/flux_calib.c`

**问题**: 电气角速度 `omega_e < 1e-3` 时钳位到 `1e-3 rad/s`，导致 `back_emf / 0.001` 被放大数千倍，引起数值不稳定和错误的校准结果。

**修复**: 改为速度不足时跳过本次采样（`goto skip_sample`），阈值提高到 `1.0 rad/s`。

---

## 五、BSP 驱动层

### [Critical] 11. ADC DMA 缓冲区缺少 `volatile`

**文件**: `Src/HAL/bsp/bsp_adc.h` / `bsp_adc.c`

**问题**: `adc1_dma_value` 和 `adc2_dma_value` 由 DMA 硬件异步写入，但声明时未加 `volatile`。编译器可能优化掉重复读取，导致 CPU 使用缓存中的过期值。

**修复**: 声明和定义均加 `volatile` 限定符。

---

### [Medium] 12. `bsp_can.c` malloc + 残缺函数 + DLC 越界

**文件**: `Src/HAL/bsp/bsp_can.c`

**问题**:
1. `FDCANRegister()` 使用 `malloc` 分配 CAN 实例，嵌入式环境易造成内存碎片/泄漏
2. `FDCANSendData()` 函数体残缺，只有 `return 1;`
3. CAN 接收回调中 `memcpy(frame.data, can_rx_buff, frame.dlc)` 未校验 DLC 上限

**修复**:
1. 改用静态内存池 `fdcan_instance_pool[FDCAN_MX_REGISTER_CNT]`，移除 `stdlib.h`
2. 补全 `FDCANSendData()` 函数体
3. DLC 拷贝加安全截断 `copy_len = (dlc > 8) ? 8 : dlc`

---

## 六、UI 调试接口

### [Critical] 13. VOFA 接收缓冲区溢出

**文件**: `Src/UI/vofa/vofa.c` — `vofa_Receive()`

**问题**: `memcpy(receive_buf, buf, len)` 无边界检查，当 `len > MAX_RXBUFFER_SIZE` 时越界写入。且未 null 终止，后续 `strstr/atof` 可能读到垃圾数据。

**修复**: 截断到 `MAX_RXBUFFER_SIZE - 1`，并显式 null 终止。

```c
if (len >= MAX_RXBUFFER_SIZE) {
  len = MAX_RXBUFFER_SIZE - 1;
}
memcpy(receive_buf, buf, len);
receive_buf[len] = '\0';
```

---

### [Medium] 14. `vofa_cmd_parse` 偏移量计算错误

**文件**: `Src/UI/vofa/vofa.c` — `vofa_cmd_parse()`

**问题**: 旧实现 `atof(cmdBuf + strlen(arg))` 假设命令从缓冲区起始位置开始。若前面有其他字符，偏移量错误会解析到错误的值。

**修复**: 改用 `strstr` 定位参数在缓冲区中的实际位置。

```c
// Before
return atof((char *)cmdBuf + strlen(arg));

// After
const char *pos = strstr(recvStr, arg);
if (pos == NULL) return 0.0f;
return atof(pos + strlen(arg));
```

---

## 修改文件清单

| 文件 | 修改项 |
|------|--------|
| `Src/ALGO/motor/fsm.h` | STATE_COUNT、auto_advance、fault_history per-instance、注释更新 |
| `Src/ALGO/motor/fsm.c` | 全部重写：prev_cw 修复、EnterFault DS402 合规、AutoAdvance 多步转换、ClearFault 简化、HandleStateEntry 完善、校准退出路径 |
| `Src/ALGO/foc/foc_algorithm.c` | ONE_OVER_SQRT3 / VOLTAGE_MARGIN 常量替换 |
| `Src/ALGO/motor/flux_calib.c` | 低速防护改为跳过采样 |
| `Src/COMM/executor/executor.c` | 临界区合并、参数写入简化 |
| `Src/HAL/bsp/bsp_adc.h` | volatile 声明 |
| `Src/HAL/bsp/bsp_adc.c` | volatile 定义 |
| `Src/HAL/bsp/bsp_can.c` | 静态内存池、补全 FDCANSendData、DLC 安全截断 |
| `Src/UI/vofa/vofa.c` | 缓冲区溢出修复、null 终止、cmd_parse 用 strstr 定位 |

---

## 未修复的已知问题（建议后续处理）

| 优先级 | 描述 | 位置 |
|--------|------|------|
| Medium | `bsp_can.c` 错误时 `while(1)` 死循环无 watchdog feed | L87, L95 |
| Medium | Flash 双页写入非原子，掉电可能损坏参数 | `param_storage.c` |
| Minor | `motor_adc.c` 冒泡排序对 volatile 数组直接排序可能被优化 | 排序函数 |
| Minor | `control.c:84` 用 `<=` 比较枚举值，枚举变更时会静默出错 | 应改为显式判断 |
| Minor | PID / RateLimiter 缺少 NaN/Inf 输入防护 | `pid.c`, `rate_limiter.c` |
