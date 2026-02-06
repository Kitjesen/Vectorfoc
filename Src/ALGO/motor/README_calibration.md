# Motor Parameter Calibration Module

## 1. 概述 (Overview)

本模块实现了无刷电机 (PMSM/BLDC) 的全自动参数辨识功能。系统可以自动测量电阻、电感、极对数、编码器偏移及线性化误差、磁链等关键参数，并自动保存到 Flash 中。

该模块设计为独立的子系统，通过 `CalibrationContext` 维护状态，支持多电机实例。

### 核心功能

- **Current Offset**: 电流传感器零偏自动校准。
- **Resistance (Rs)**: 定子相电阻测量。
- **Inductance (Ls)**: 定子相电感测量（高频注入法）。
- **Encoder**: 自动识别方向、极对数，并进行 128 点线性化误差补偿。
- **Flux**: 永磁体磁链观测与辨识。

---

## 2. 校准流程详解 (Calibration Process)

整个校准过程由有限状态机 (FSM) 驱动，确保步骤的严格时序。

### 2.1 整体流程图 (State Machine)

```mermaid
graph TD
    Start[触发校准] --> Current[1. 电流零偏校准]
    Current -->|Success| Init[初始化 RSLS 上下文]
    Init --> Res[2. 电阻校准 (Rs)]
    Res -->|Success| Ind[3. 电感校准 (Ls)]
    Ind -->|Success| DirPP[4. 方向与极对数]
    DirPP -->|Success| Enc[5. 编码器线性化]
    Enc -->|Success| Flux[6. 磁链辨识]
    Flux -->|Success| Save[保存参数至 Flash]
    Save --> Run[进入运行模式]
  
    Current -->|Fail| Error
    Res -->|Fail| Error
    Ind -->|Fail| Error
    DirPP -->|Fail| Error
    Enc -->|Fail| Error
    Flux -->|Fail| Error
```

### 2.2 各步骤详细说明

#### 1. 电流传感器校准 (Current Calibration)

- **文件**: `current_calib.c`
- **原理**: 在 PWM 关闭（或 0 占空比）状态下，连续采集数千次 ADC 值取平均，作为 0A 基准。
- **配置**: `CURRENT_CALIB_CYCLES` (默认 2000 次)

#### 2. 电阻校准 (Resistance Calibration)

- **文件**: `calib_resistance.c`
- **原理**: 向电机注入恒定 DC 电压 (Alpha 轴)，利用 PI 闭环控制电流稳定在目标值 (`CURRENT_MAX_CALIB`)。根据 $R = V/I$ 计算电阻。
- **特点**: 自动计算 1.5 倍关系 ($R_{phase} = R_{measured} * 2/3$)。

#### 3. 电感校准 (Inductance Calibration)

- **文件**: `calib_inductance.c`
- **原理**: 施加高频方波电压，测量电流变化率 $di/dt$。利用 $L = V / (di/dt)$ 计算。
- **配置**: `VOLTAGE_MAX_CALIB`

#### 4. 方向与极对数 (Direction & Pole Pair)

- **文件**: `calib_encoder.c`
- **原理**: 开环强拖电机旋转 $N$ 个电角度周期，对比编码器机械角度变化量，计算极对数 $PP = \Delta \theta_{elec} / \Delta \theta_{mech}$。同时识别编码器增减方向与电机旋转方向的关系。

#### 5. 编码器线性化 (Encoder Linearization)

- **文件**: `calib_encoder.c`
- **原理**:
  1. 电机单向慢速开环旋转。
  2. 记录每个电角度步进下的 **[理论角度 - 实际角度]** 误差。
  3. 生成 128 点的误差查找表 (LUT)。
- **作用**: 消除磁编码器的非线性误差和安装误差，大幅提升低速平顺性。

#### 6. 磁链辨识 (Flux Calibration)

- **文件**: `flux_calib.c`
- **原理**: 这是一个独立步骤。电机运行在恒定速度的空载闭环模式 (Id=0)，利用电压平衡方程 $V_q = I_q R_s + \omega_e \psi_f$ 反算磁链 $\psi_f$。
- **配置**: 需要电机能稳定旋转，速度由 `FLUX_CALIB_VEL` 设定。

---

## 3. 集成指南 (Integration Guide)

校准模块主要在 `motor/core/motor.c` 的 `MotorInitializeTask` 中调用。

### 调用代码示例

若要在代码中手动触发完整校准：

```c
// 触发完整校准流程
Motor_RequestCalibration(&motor_data, 0); 

// 0: 完整校准
// 1: 仅 R/L 参数
// 2: 仅编码器 (目前逻辑会包含 RSLS 流程)
```

或在初始化时强制进入校准模式：

```c
void System_Init() {
    // ...
    // 设置初始状态为检测模式
    motor_data.state.State_Mode = STATE_MODE_DETECTING;
    motor_data.state.Sub_State = CURRENT_CALIBRATING;
}
```

### 关键配置宏 (Macro Configuration)

位于 `motor/config.h` (部分在各模块头文件)：

| 宏定义                    | 默认值 | 说明                          |
| :------------------------ | :----- | :---------------------------- |
| `CURRENT_MAX_CALIB`     | 2.0f   | [A] R/L 校准时的目标电流      |
| `VOLTAGE_MAX_CALIB`     | 1.0f   | [V] L 校准时的测试电压幅值    |
| `RS_CALIB_CYCLES`       | 20000  | 电阻校准持续周期数 (20k @ 1s) |
| `LS_CALIB_CYCLES`       | 4000   | 电感校准每个状态的周期数      |
| `FLUX_CALIB_VEL`        | 20.0f  | [rad/s] 磁链校准时的目标转速  |
| `SAMPLES_PER_POLE_PAIR` | 128    | 编码器线性化 LUT 大小         |

---

## 4. 上下文结构 (Data Structure)

所有校准状态数据存储在 `CalibrationContext`，不占用全局静态区，便于移植。

```c
typedef struct {
  CurrentCalibContext     current;      // 1. 电流校准上下文
  ResistanceCalibContext  resistance;   // 2. 电阻
  InductanceCalibContext  inductance;   // 3. 电感
  DirectionPoleCalibContext dir_pole;   // 4. 方向/极对数
  EncoderCalibContext     encoder;      // 5. 编码器
  FluxCalibContext        flux;         // 6. 磁链
} CalibrationContext;
```

---

## 5. 常见问题 (Troubleshooting)

### Q1: 校准时电机不转或抖动？

- **检查电压限制**: 确保 `VOLTAGE_MAX_CALIB` 足以克服摩擦力。
- **检查电流限制**: `CURRENT_MAX_CALIB` 是否过小。
- **极对数错误**: 如果极对数预设严重偏差（虽然会自动识别，但初始开环需要大致范围），可能导致拖动失败。

### Q2: 极对数识别错误？

- 确保编码器磁铁安装牢固，无打滑。
- 此时负载应尽可能小（空载最好）。

### Q3: 电感测量值为 0 或负数？

- 检查采样时间 `dt` 是否正确传递。
- 检查电流采样信号是否有严重噪声。

### Q4: 磁链校准失败？

- 磁链校准依赖于准确的 $R_s$ 和 $L_s$。如果前几步不准，磁链也会不准。
- 确保电机在该速度下反电动势不会超过母线电压限制。
