# Algorithm Module

**Status**: ✅ Active - Production Ready

## 概述

本模块提供电机控制系统所需的通用算法实现，包括PID控制器、速率限制器、梯形轨迹规划器和CRC校验工具。

## 目录结构

```
algorithm/
├── pid.h/c              # PID控制器
├── rate_limiter.h/c     # 速率限制器
├── trap_traj.h/c        # 梯形轨迹规划器
└── crc/                 # CRC校验工具
    ├── crc8.h/c         # CRC-8算法
    └── crc16.h/c        # CRC-16算法
```

## 模块说明

### 1. PID控制器 (`pid.h/c`)

**功能**: 通用PID控制器实现

**特性**:
- 支持比例(P)、积分(I)、微分(D)三项控制
- 积分抗饱和(Anti-windup)
- 输出限幅
- 可配置采样周期

**主要API**:
```c
void PID_Init(PID_t *pid, float kp, float ki, float kd, float dt);
float PID_Update(PID_t *pid, float setpoint, float measurement);
void PID_Reset(PID_t *pid);
void PID_SetLimits(PID_t *pid, float min, float max);
```

**使用示例**:
```c
PID_t velocity_pid;
PID_Init(&velocity_pid, 0.1f, 0.01f, 0.0f, 0.001f);  // Kp=0.1, Ki=0.01, Kd=0, dt=1ms
PID_SetLimits(&velocity_pid, -10.0f, 10.0f);

float output = PID_Update(&velocity_pid, target_vel, current_vel);
```

---

### 2. 速率限制器 (`rate_limiter.h/c`)

**功能**: 限制信号变化速率，防止突变

**特性**:
- 可配置上升/下降速率
- 平滑过渡
- 防止超调

**主要API**:
```c
void RateLimiter_Init(RateLimiter_t *limiter, float rate_limit, float dt);
float RateLimiter_Update(RateLimiter_t *limiter, float input);
void RateLimiter_Reset(RateLimiter_t *limiter, float initial_value);
```

**使用示例**:
```c
RateLimiter_t torque_limiter;
RateLimiter_Init(&torque_limiter, 100.0f, 0.001f);  // 100 A/s, dt=1ms

float limited_torque = RateLimiter_Update(&torque_limiter, target_torque);
```

---

### 3. 梯形轨迹规划器 (`trap_traj.h/c`)

**功能**: 生成梯形速度曲线轨迹

**特性**:
- 加速-匀速-减速三段式轨迹
- 可配置最大速度和加速度
- 实时轨迹计算
- 位置/速度/加速度输出

**主要API**:
```c
void TRAJ_plan(TrapTraj_t *traj, float x0, float xf, float v_max, float a_max);
void TRAJ_eval(TrapTraj_t *traj, float t);
float TRAJ_get_position(TrapTraj_t *traj);
float TRAJ_get_velocity(TrapTraj_t *traj);
```

**使用示例**:
```c
TrapTraj_t position_traj;
TRAJ_plan(&position_traj, 0.0f, 10.0f, 5.0f, 20.0f);  // 0→10rad, v_max=5rad/s, a_max=20rad/s²

// 在控制循环中
float t = get_time();
TRAJ_eval(&position_traj, t);
float pos_ref = TRAJ_get_position(&position_traj);
float vel_ref = TRAJ_get_velocity(&position_traj);
```

---

### 4. CRC校验 (`crc/`)

**功能**: 数据完整性校验

#### CRC-8 (`crc8.h/c`)
- 多项式: 0x07
- 用于短数据包校验

```c
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t length);
bool CRC8_Verify(const uint8_t *data, uint16_t length, uint8_t expected_crc);
```

#### CRC-16 (`crc16.h/c`)
- 多项式: 0x1021 (CCITT)
- 用于长数据包校验

```c
uint16_t CRC16_Calculate(const uint8_t *data, uint16_t length);
bool CRC16_Verify(const uint8_t *data, uint16_t length, uint16_t expected_crc);
```

---

## 性能指标

| 算法 | 执行时间 | 内存占用 |
|------|---------|---------|
| PID | ~1μs | 32 bytes |
| Rate Limiter | ~0.5μs | 16 bytes |
| Trap Trajectory | ~5μs | 48 bytes |
| CRC-8 | ~2μs/byte | 0 bytes (无状态) |
| CRC-16 | ~3μs/byte | 0 bytes (无状态) |

---

## 使用建议

### PID调参建议
1. **先P后I**: 先调Kp直到系统响应快速但有振荡
2. **加I消差**: 添加Ki消除稳态误差
3. **D项慎用**: Kd对噪声敏感，通常设为0或很小值

### 轨迹规划建议
- 根据电机性能设置合理的`v_max`和`a_max`
- 对于短距离移动，可能无法达到`v_max`（三角形轨迹）
- 使用`TRAJ_eval()`前确保调用了`TRAJ_plan()`

---

## 与其他模块的关系

```
motor/control/
    ↓ 使用
algorithm/pid.c (电流环/速度环/位置环)
algorithm/trap_traj.c (位置规划)
algorithm/rate_limiter.c (力矩限制)

communication/
    ↓ 使用
algorithm/crc/ (数据校验)
```

---

## 注意事项

> [!IMPORTANT]
> **采样周期一致性**: PID和RateLimiter的`dt`参数必须与实际控制周期一致

> [!WARNING]
> **数值稳定性**: 梯形轨迹规划器对极小的移动距离可能产生数值误差

> [!TIP]
> **性能优化**: CRC计算可使用查表法优化，当前实现为逐位计算
