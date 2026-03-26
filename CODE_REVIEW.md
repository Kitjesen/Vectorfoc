# VectorFOC 固件代码审查报告

**审查日期**: 2026-02-22  
**审查人**: FOC 电机控制专家  
**项目路径**: `D:\inovxio\modules\foc\vectorfoc`

---

## 审查总结

| 审查项 | 评分 | 说明 |
|--------|------|------|
| 一、数据流图：FOC 闭环完整性 | ✅ 良好 | 闭环路径清晰，数据流规范 |
| 二、参数中心：物理参数唯一来源 | ✅ 良好 | 参数集中管理，无魔法数字 |
| 三、数学正确性 | ✅ 良好 | Clarke/Park/SVPWM 公式正确 |
| 四、任务调度与时序 | ✅ 良好 | 中断驱动，时序清晰 |
| 五、状态机与模式切换 | ✅ 良好 | DS402 标准状态机，完整 |
| 六、保护与限幅 | ✅ 良好 | 多层保护，anti-windup 完善 |
| 七、可测试性 | ✅ 良好 | HAL 抽象完整，单元测试覆盖 |

**总体评价**: 代码架构清晰，FOC 实现规范，工程质量高。

---

## 一、数据流图：FOC 闭环是否完整

### 评分: ✅ 良好

### 数据流路径

```
ADC 中断 (20kHz)
    │
    ├─► ISR_UpdateSensors() ─► algo_input.Ia/Ib/Ic/Vbus
    │
    ├─► ISR_UpdateEncoder() ─► feedback.position/velocity/phase_angle
    │                          algo_input.theta_elec
    │
    └─► MotorStateTask() ─► MotorControl_Run()
                               │
                               ├─► Control_OuterLoopsUpdate() [5kHz]
                               │       │
                               │       ├─► Position PID ─► vel_set
                               │       └─► Velocity PID/LADRC ─► Iq_ref
                               │
                               └─► Control_InnerCurrentLoop() [20kHz]
                                       │
                                       └─► FOC_Algorithm_CurrentLoop()
                                               │
                                               ├─► Clarke_Transform (abc→αβ)
                                               ├─► Park_Transform (αβ→dq)
                                               ├─► PI 电流环 + 解耦前馈
                                               ├─► 电压限幅 + Anti-windup
                                               ├─► Park_Inverse (dq→αβ)
                                               └─► SVPWM_Modulate (αβ→PWM)
                                                       │
                                                       └─► HAL PWM 输出
```

### 验证结果

1. **ADC → 电流物理量换算**: ✅ 集中在 `config.h` 的 `FAC_CURRENT` 宏
   ```c
   #define FAC_CURRENT ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))
   ```

2. **Clarke → Park → PI → 逆Park → SVPWM 路径**: ✅ 清晰完整
   - 路径在 `foc_algorithm.c` 的 `FOC_Algorithm_CurrentLoop()` 中完整实现
   - 每个步骤有清晰注释

3. **最终输出统一到 PWM 结构体**: ✅ 
   - 输出到 `FOC_AlgorithmOutput_t` 的 `Ta/Tb/Tc`
   - 通过 HAL 接口 `motor->components.hal->pwm->set_duty()` 输出

---

## 二、参数中心：物理参数是否有唯一来源

### 评分: ✅ 良好

### 参数结构体

```c
// motor.h - 电机参数结构体
typedef struct {
  float Rs;       // 相电阻 [Ohm]
  float Ls;       // 相电感 [H]
  float flux;     // 磁链 [V·s]
  int pole_pairs; // 极对数
} MOTOR_PARAMETERS;

// foc_algorithm.h - FOC 配置结构体
typedef struct {
  float Rs;           // 相电阻 [Ohm]
  float Ls;           // 相电感 [H]
  float flux;         // 磁链 [Wb]
  uint8_t pole_pairs; // 极对数
  // ... PI 参数等
} FOC_AlgorithmConfig_t;
```

### 验证结果

1. **统一的 MotorConfig 结构体**: ✅
   - `MOTOR_PARAMETERS` 存储电机物理参数
   - `MOTOR_CONTROLLER` 存储控制参数
   - `FOC_AlgorithmConfig_t` 存储 FOC 算法配置

2. **极对数只定义一次**: ✅
   - 默认值在 `config.h`: `#define DEFAULT_POLE_PAIRS 7`
   - 运行时存储在 `motor->parameters.pole_pairs`
   - 同步到 `motor->algo_config.pole_pairs`

3. **无魔法数字**: ✅
   - 所有常量都有宏定义和注释
   - 数学常量集中在 `math_common.h`
   ```c
   #define MATH_SQRT3_BY_2 (0.866025403784439f)
   #define MATH_2_BY_3 (0.666666666666667f)
   ```

---

## 三、数学正确性：Clarke/Park/SVPWM 公式

### 评分: ✅ 良好

### Clarke 变换 (`clarke.c`)

```c
void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta) {
    *Ialpha = Ia;
    *Ibeta = (Ib - Ic) * MATH_ONE_BY_SQRT3;
}
```

**分析**: ✅ 正确
- 使用等幅值 Clarke 变换（非功率不变）
- 假设三相对称 `Ia + Ib + Ic = 0`
- 公式: `Iα = Ia`, `Iβ = (Ib - Ic) / √3`

### Park 变换 (`park.c`)

```c
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq) {
  float sin_val, cos_val;
  Trig_FastSinCos(theta, &sin_val, &cos_val);
  *Id = Ialpha * cos_val + Ibeta * sin_val;
  *Iq = -Ialpha * sin_val + Ibeta * cos_val;
}
```

**分析**: ✅ 正确
- 标准 Park 变换公式
- 角度单位: 弧度
- d/q 轴约定: d 轴对齐转子磁链

### SVPWM (`svpwm.c`)

```c
float mod_alpha = Valpha / (Vbus * MATH_2_BY_3);
float mod_beta = Vbeta / (Vbus * MATH_2_BY_3);
// ... 中点钳位 + 过调制处理
```

**分析**: ✅ 正确
- 使用中点钳位 SVPWM
- 过调制时自动缩放
- 最终 duty cycle 限幅到 [0, 1]

### 三角函数 (`trigonometry.c`)

**分析**: ✅ 正确
- 使用多项式近似，精度约 0.1%
- 角度归一化到 [-π, π]

---

## 四、任务调度与时序

### 评分: ✅ 良好

### 中断入口 (`isr_foc.c`)

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  // 1. 喂狗
  HAL_WatchdogFeed();
  // 2. 采样传感器 (20kHz)
  ISR_UpdateSensors(&motor_data);
  // 3. 更新编码器 (20kHz)
  ISR_UpdateEncoder(&motor_data);
  // 4. 观测器更新 (20kHz)
  Motor_API_Observer_Update(&motor_data);
  // 5. 快速安全检测 (20kHz)
  Safety_Update_Fast(&motor_data, &g_ds402_state_machine);
  // 6. 状态机更新 (1kHz, 分频)
  if (++s_fsm_counter >= FSM_UPDATE_DIV) { ... }
  // 7. 高级控制 (5kHz, 分频)
  if (++s_adv_counter >= ADV_CONTROL_DIV) { ... }
  // 8. FOC 主循环 (20kHz)
  MotorStateTask(&motor_data);
}
```

### 验证结果

1. **电流环入口函数**: ✅ `HAL_ADCEx_InjectedConvCpltCallback` (ADC 注入转换完成中断)

2. **执行顺序**: ✅ 严格按 采样→变换→PI→调制→PWM
   - 采样: `ISR_UpdateSensors()`
   - 变换+PI+调制: `FOC_Algorithm_CurrentLoop()`
   - PWM: `motor->components.hal->pwm->set_duty()`

3. **速度环较慢频率**: ✅
   - 外环在 `Control_OuterLoopsUpdate()` 中 4 分频执行 (5kHz)
   ```c
   if (++ctx->loop_count < 4) return;
   ```

### 分频配置 (`config.h`)

```c
#define CONTROL_FREQ_HZ 20000   // FOC 频率
#define FSM_UPDATE_HZ 1000      // 状态机 1kHz
#define ADV_CONTROL_HZ 5000     // 高级控制 5kHz
```

---

## 五、状态机与模式切换

### 评分: ✅ 良好

### 状态枚举 (`fsm.h`)

```c
typedef enum {
  STATE_NOT_READY_TO_SWITCH_ON = 0,
  STATE_SWITCH_ON_DISABLED,
  STATE_READY_TO_SWITCH_ON,
  STATE_SWITCHED_ON,
  STATE_OPERATION_ENABLED,
  STATE_QUICK_STOP_ACTIVE,
  STATE_FAULT_REACTION_ACTIVE,
  STATE_FAULT,
  STATE_CALIBRATING,  // 扩展状态
  STATE_COUNT
} MotorState;
```

**分析**: ✅ 完整实现 CANopen DS402 状态机

### 状态切换 (`fsm.c`)

- ✅ 状态切换集中在 `ProcessStateTransitions()` 和 `ExecuteTransition()`
- ✅ 支持控制字驱动和 API 请求两种方式
- ✅ 故障处理: `StateMachine_EnterFault()` → `FAULT_REACTION_ACTIVE` → `FAULT`

### 启动流程

```
上电 → NOT_READY_TO_SWITCH_ON
    → SWITCH_ON_DISABLED (自动)
    → READY_TO_SWITCH_ON (Shutdown 命令)
    → SWITCHED_ON (Switch On 命令)
    → OPERATION_ENABLED (Enable Operation 命令)
```

---

## 六、保护与限幅

### 评分: ✅ 良好

### PI 输出限幅 (`foc_algorithm.c`)

```c
// 电压限幅
float V_max = input->Vbus * ONE_OVER_SQRT3 * VOLTAGE_MARGIN;
float V_mag = sqrtf(output->Vd * output->Vd + output->Vq * output->Vq);
if (V_mag > V_max) {
  float scale = V_max / V_mag;
  output->Vd *= scale;
  output->Vq *= scale;
  output->voltage_saturated = true;
}
```

### 积分抗饱和 (`foc_algorithm.c`)

```c
// Back-calculation anti-windup
float Kb = config->Kb_current;
state->integral_d += (config->Ki_current_d * error_d + 
                      Kb * (output->Vd - (Vd_raw + Vd_ff))) * dt;
```

**分析**: ✅ 使用 Back-calculation 方法，工业标准实现

### PID 抗饱和 (`pid.c`)

```c
// Conditional Integration (Clamping)
if (out_unlimited > pid->max_out) {
  pid->out = pid->max_out;
  if (pid->error[0] > 0.0f) {
    i_term = pid->Iout; // Revert integration
  }
}
```

### 故障检测 (`fault_detection.c`)

| 保护类型 | 阈值 | 实现 |
|----------|------|------|
| 过压 | 60V | ✅ `Detection_CheckVoltage()` |
| 欠压 | 12V | ✅ `Detection_CheckVoltage()` |
| 过流 | 90A | ✅ `Detection_CheckCurrent()` |
| 过温 | 145°C | ✅ `Detection_CheckTemperature()` |
| 堵转 | 500ms | ✅ `Detection_CheckStall()` |
| 编码器丢失 | 20次连续错误 | ✅ `Detection_CheckEncoder()` |

### 分层安全检测

```c
Safety_Update_Fast()  // 20kHz - 仅过流检测
Safety_Update_Slow()  // 200Hz - 全部检测
```

---

## 七、可测试性

### 评分: ✅ 良好

### 硬件抽象层 (`motor_hal_api.h`)

```c
typedef struct {
    const Motor_HAL_PwmInterface_t     *pwm;
    const Motor_HAL_AdcInterface_t     *adc;
    const Motor_HAL_EncoderInterface_t *encoder;
} Motor_HAL_Handle_t;
```

**分析**: ✅ 完整的 HAL 抽象，支持 Mock 替换

### 数学模块独立性

| 模块 | 依赖 | 可独立测试 |
|------|------|------------|
| `clarke.c` | `math_common.h` | ✅ |
| `park.c` | `trigonometry.h` | ✅ |
| `svpwm.c` | `math_common.h` | ✅ |
| `pid.c` | `common.h` | ✅ |
| `foc_algorithm.c` | 上述模块 | ✅ |

### 单元测试覆盖 (`test/`)

| 测试文件 | 覆盖模块 |
|----------|----------|
| `test_foc_core.c` | Clarke, Park, SVPWM |
| `test_foc_integration.c` | FOC 闭环 |
| `test_pid.c` | PID 控制器 |
| `test_ladrc.c` | LADRC 控制器 |
| `test_trigonometry.c` | 三角函数 |
| `test_rate_limiter.c` | 斜坡限制器 |
| `test_trap_traj.c` | 梯形轨迹 |

**分析**: ✅ 测试覆盖完整，有 Mock HAL 支持

---

## 发现的问题与代码修复

### 问题 1: PID_Calc 使用 dt=1.0f ✅ 已修复

**位置**: `Src/ALGO/control/outer.c`

**问题**: 外环调用 `PID_Calc()` 时 dt=1.0f，依赖调用者预先缩放 Ki/Kd。

**修复**: 将所有 `PID_Calc()` 调用改为 `PID_CalcDt()` 并传入实际 dt (`OUTER_LOOP_DT`)。

```c
// 修复前
return PID_Calc(&motor->VelPID, vel_fdb, vel_ref);

// 修复后
return PID_CalcDt(&motor->VelPID, vel_fdb, vel_ref, OUTER_LOOP_DT);
```

### 问题 2: 电流滤波器截止频率默认值 ✅ 已修复

**位置**: `Src/ALGO/motor/motor_data.c`

**问题**: `current_filter_fc` 未在初始化时设置默认值，可能导致无滤波。

**修复**: 在 `algo_config` 初始化中添加默认值:

```c
.algo_config = {
    // ...
    .current_filter_fc = 1000.0f,  // 默认 1kHz 滤波
}
```

### 问题 3: Math_NormalizeAngle 使用 while 循环 ✅ 已修复

**位置**: `Src/ALGO/foc/math_common.h`

**问题**: 极端输入可能导致多次循环。

**修复**: 使用 `fmodf` 实现 O(1) 复杂度:

```c
static inline float Math_NormalizeAngle(float angle) {
  /* Fast path for common case */
  if (angle >= -MATH_PI && angle <= MATH_PI) {
    return angle;
  }
  /* Use fmodf for O(1) normalization */
  angle = fmodf(angle + MATH_PI, MATH_2PI);
  if (angle < 0.0f) {
    angle += MATH_2PI;
  }
  return angle - MATH_PI;
}
```

---

## 代码质量亮点

1. **架构清晰**: FOC 核心算法与硬件完全解耦
2. **注释完整**: 每个函数都有详细的 Doxygen 注释
3. **参数集中**: 所有配置参数在 `config.h` 统一管理
4. **安全分层**: 快速/慢速安全检测分离，保证实时性
5. **状态机规范**: 完整实现 DS402 标准
6. **测试完善**: 单元测试覆盖核心算法

---

## 结论

VectorFOC 固件代码质量优秀，FOC 闭环实现完整正确，架构设计合理。本次审查发现的 3 个小问题已全部修复。

**推荐评级**: ⭐⭐⭐⭐⭐ (5/5)

---

## 修改文件清单

| 文件 | 修改内容 |
|------|----------|
| `Src/ALGO/foc/math_common.h` | `Math_NormalizeAngle` 改用 fmodf 实现 |
| `Src/ALGO/control/outer.c` | `PID_Calc` 改为 `PID_CalcDt` 并传入实际 dt |
| `Src/ALGO/motor/motor_data.c` | `algo_config` 添加完整初始化，包括 `current_filter_fc` |

---

# 深度细节审查报告 (2026-02-22)

本节针对 FOC 电机控制中真正拉开差距的关键细节进行深度审查。

---

## 一、死区补偿

### 评分: ⚠️ 一般

### 代码位置
`Src/ALGO/control/inner.c` 第 30-55 行

### 当前实现

```c
#ifdef DEADTIME_COMP
  float deadtime_duty = (float)DEADTIME_COMP / (float)PWM_ARR;
  // 用 Valpha/Vbeta 反推相电压，根据电压方向判断（而非电流方向）
  float Valpha = motor->algo_output.Valpha;
  float Vbeta = motor->algo_output.Vbeta;
  float Va = Valpha;
  float Vb = -0.5f * Valpha + MATH_SQRT3_BY_2 * Vbeta;
  float Vc = -0.5f * Valpha - MATH_SQRT3_BY_2 * Vbeta;
  
  // 过零阈值处理
  float v_thresh = (vbus > 1.0f ? vbus : 1.0f) * 0.02f;
  float w_a = (fabsf(Va) < v_thresh) ? (fabsf(Va) / v_thresh) : 1.0f;
  
  // 补偿方向：根据电压方向
  float s_a = (Va > 0.0f) ? -1.0f : 1.0f;
  Ta += deadtime_duty * s_a * w_a;
#endif
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 实时电流方向判断 | ❌ 缺失 | 使用电压方向而非电流方向判断 |
| 电压补偿方向 | ⚠️ 近似 | 电压方向在稳态时与电流方向一致，但动态响应时可能不准 |
| 电流过零点处理 | ✅ 有 | 使用权重 `w_a` 在过零点附近平滑过渡 |

### 改进建议

**问题**: 死区补偿应基于电流方向，而非电压方向。当电流过零时，电压方向可能与电流方向不一致。

**推荐实现**:

```c
// 使用实际相电流方向判断
float Ia = motor->algo_input.Ia;
float Ib = motor->algo_input.Ib;
float Ic = motor->algo_input.Ic;

// 电流过零阈值 (约 0.5A)
float i_thresh = 0.5f;

// 根据电流方向确定补偿符号
float s_a = (Ia > i_thresh) ? 1.0f : ((Ia < -i_thresh) ? -1.0f : 0.0f);
float s_b = (Ib > i_thresh) ? 1.0f : ((Ib < -i_thresh) ? -1.0f : 0.0f);
float s_c = (Ic > i_thresh) ? 1.0f : ((Ic < -i_thresh) ? -1.0f : 0.0f);

// 在过零区间内线性插值
if (fabsf(Ia) < i_thresh) {
    s_a = Ia / i_thresh;  // 平滑过渡
}

Ta += deadtime_duty * s_a;
```

---

## 二、电流采样方案

### 评分: ⚠️ 一般

### 代码位置
- `Src/HAL/stm32g4/motor_hal_g431.c` 第 70-95 行
- `Src/HAL/stm32g4/motor_adc.c` 第 60-90 行

### 当前实现

```c
// motor_hal_g431.c - ADC 更新
static void G431_ADC_Update(Motor_HAL_SensorData_t *data) {
  // 从 JDR 寄存器读取（注入组）
  float adc_i_a = (float)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IA;
  float adc_i_b = (float)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IB;
  float adc_i_c = (float)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IC;
  
  data->i_a = (adc_i_a - current_data.Ia_offset) * FAC_CURRENT;
  data->i_b = (adc_i_b - current_data.Ib_offset) * FAC_CURRENT;
  data->i_c = (adc_i_c - current_data.Ic_offset) * FAC_CURRENT;
}

// Offset 校准
static void G431_ADC_CalibrateOffsets(void) {
  uint32_t sum_a = 0, sum_b = 0, sum_c = 0;
  const int samples = 1000;
  for (int i = 0; i < samples; i++) {
    HAL_Delay(1);
    sum_a += HW_ADC_CURRENT.Instance->HW_ADC_JDR_IA;
    // ...
  }
  current_data.Ia_offset = (float)sum_a / samples;
}
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 采样时刻在 PWM 中点 | ✅ 正确 | 使用 ADC 注入组，由 TIM1 TRGO 触发，配置为中点采样 |
| ADC Offset 校准 | ⚠️ 基础 | 仅做简单平均，无温漂补偿 |
| 采样噪声滤波 | ⚠️ 可选 | 有中值滤波和均值滤波函数，但未在主采样路径使用 |

### 改进建议

**1. ADC Offset 温漂补偿**

```c
// 建议：周期性更新 offset（电机静止时）
void ADC_UpdateOffsetOnline(MOTOR_DATA *motor) {
    // 仅在电机静止且电流接近零时更新
    if (fabsf(motor->feedback.velocity) < 0.01f &&
        fabsf(motor->algo_output.Iq) < 0.1f) {
        // 指数移动平均更新 offset
        float alpha = 0.001f;
        current_data.Ia_offset += alpha * (adc_raw_a - current_data.Ia_offset);
    }
}
```

**2. 采样噪声滤波**

当前 `motor_adc.c` 中有 `adc1_median_filter()` 和 `adc1_avg_filter()` 函数，但未在主采样路径使用。建议在高噪声环境下启用：

```c
// 可选：在 ADC 更新中使用滤波
float adc_i_a = (float)adc1_median_filter(ADC_CH_IA);
```

---

## 三、电流环带宽设计

### 评分: ✅ 优秀

### 代码位置
`Src/ALGO/control/control.c` 第 80-110 行

### 当前实现

```c
void CurrentLoop_UpdateGain(MOTOR_DATA *motor) {
#if CURRENT_AUTO_CALIBRATION
  // 自动计算带宽 (BW ≈ vel_limit * pole_pairs * 2pi)
  float bandwidth = motor->Controller.vel_limit * 
                    motor->parameters.pole_pairs * M_2PI;
  motor->Controller.current_ctrl_p_gain = motor->parameters.Ls * bandwidth;
  motor->Controller.current_ctrl_i_gain = motor->parameters.Rs * bandwidth;
#else
  // 手动配置
  motor->Controller.current_ctrl_p_gain = 
      motor->parameters.Ls * motor->Controller.current_ctrl_bandwidth * 1.0f;
  motor->Controller.current_ctrl_i_gain = 
      motor->parameters.Rs * motor->Controller.current_ctrl_bandwidth;
#endif
}
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 带宽法计算 PI 参数 | ✅ 正确 | `Kp = L × ωc`, `Ki = R × ωc` |
| 带宽选择合理性 | ⚠️ 需验证 | 自动模式下带宽 = vel_limit × pole_pairs × 2π |

### 带宽验证

```
PWM 频率: 20kHz
推荐电流环带宽: PWM/10 ~ PWM/20 = 1kHz ~ 2kHz

当前自动计算:
- vel_limit = 1000 rpm = 16.67 turn/s
- pole_pairs = 7
- bandwidth = 16.67 × 7 × 2π ≈ 733 rad/s ≈ 117 Hz

问题: 自动计算的带宽偏低！
```

### 改进建议

```c
// 建议：限制带宽范围
float bandwidth = motor->Controller.vel_limit * 
                  motor->parameters.pole_pairs * M_2PI;

// 限制在合理范围 (500 ~ 2000 Hz)
float min_bw = 500.0f * M_2PI;   // 500 Hz
float max_bw = 2000.0f * M_2PI;  // 2000 Hz
bandwidth = CLAMP(bandwidth, min_bw, max_bw);
```

---

## 四、积分抗饱和策略

### 评分: ✅ 优秀

### 代码位置
- `Src/ALGO/foc/foc_algorithm.c` 第 90-120 行 (电流环)
- `Src/ALGO/pid/pid.c` 第 50-100 行 (通用 PID)

### 当前实现

**电流环 (Back-calculation)**:
```c
// foc_algorithm.c
float Kb = config->Kb_current;
state->integral_d += (config->Ki_current_d * error_d + 
                      Kb * (output->Vd - (Vd_raw + Vd_ff))) * dt;
```

**通用 PID (Conditional Integration)**:
```c
// pid.c
if (out_unlimited > pid->max_out) {
  pid->out = pid->max_out;
  if (pid->error[0] > 0.0f) {
    i_term = pid->Iout; // 回退积分
  }
}
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 抗饱和策略 | ✅ 优秀 | 电流环用 Back-calculation，速度/位置环用 Conditional Integration |
| 输出限幅形状 | ✅ 圆形 | 电流环使用圆形限幅 `V_mag = sqrt(Vd² + Vq²)` |

### 代码验证

```c
// foc_algorithm.c - 圆形电压限幅
float V_max = input->Vbus * ONE_OVER_SQRT3 * VOLTAGE_MARGIN;
float V_mag = sqrtf(output->Vd * output->Vd + output->Vq * output->Vq);
if (V_mag > V_max) {
  float scale = V_max / V_mag;
  output->Vd *= scale;
  output->Vq *= scale;
}
```

**结论**: 积分抗饱和实现优秀，电流环使用工业标准的 Back-calculation + 圆形限幅。

---

## 五、速度环滤波

### 评分: ⚠️ 一般

### 代码位置
`Src/ALGO/control/outer.c` 第 30-80 行

### 当前实现

```c
void Control_OuterLoopsUpdate(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  // 4 分频执行 (5kHz)
  if (++ctx->loop_count < 4) return;
  ctx->loop_count = 0;
  
  // 直接使用速度反馈，无滤波
  motor->algo_input.Iq_ref = VelLoop_Calc(motor, ctx->vel_set,
                                          motor->feedback.velocity);
}
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 速度反馈低通滤波 | ❌ 缺失 | 直接使用编码器计算的速度，无滤波 |
| 速度前馈 | ❌ 缺失 | 无速度前馈项 |
| 加速度前馈 | ❌ 缺失 | 无加速度前馈项 |

### 改进建议

**1. 速度反馈滤波**

```c
// 建议：在 outer.c 中添加速度滤波
static float s_vel_filtered = 0.0f;
#define VEL_FILTER_FC 100.0f  // 100Hz 截止频率

void Control_OuterLoopsUpdate(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  if (++ctx->loop_count < 4) return;
  ctx->loop_count = 0;
  
  // 速度低通滤波
  float alpha = (M_2PI * VEL_FILTER_FC * OUTER_LOOP_DT) / 
                (1.0f + M_2PI * VEL_FILTER_FC * OUTER_LOOP_DT);
  s_vel_filtered += alpha * (motor->feedback.velocity - s_vel_filtered);
  
  motor->algo_input.Iq_ref = VelLoop_Calc(motor, ctx->vel_set, s_vel_filtered);
}
```

**2. 速度/加速度前馈**

```c
// 建议：添加前馈项
float vel_ff = motor->Controller.vel_setpoint * motor->advanced.vel_ff_gain;
float accel_ff = motor->Controller.accel_setpoint * motor->parameters.inertia / 
                 motor->parameters.torque_const;

motor->algo_input.Iq_ref = VelLoop_Calc(...) + vel_ff + accel_ff;
```

---

## 六、无感观测器 (SMO)

### 评分: ⚠️ 一般

### 代码位置
`Src/ALGO/observer/smo_observer.c`

### 当前实现

```c
#define SMO_PLL_BW_HZ 200.0f
#define SMO_BLEND_VEL_THRESH_RAD_S 10.0f

void SMO_Observer_Update(void *pMemory, MOTOR_DATA *motor) {
  // 滑模增益和滤波系数从外部配置
  float K_slide = smo->alpha;  // 滑模增益
  float K_filt = smo->beta;    // 滤波系数 (0.0 - 1.0)
  
  // 电流观测器
  float err_alpha = smo->est_i_alpha - i_alpha;
  float z_alpha = (err_alpha > 0.0f) ? K_slide : -K_slide;
  
  // BEMF 重构 (低通滤波)
  smo->est_bemf_alpha += (z_alpha - smo->est_bemf_alpha) * K_filt;
  
  // PLL 角度/速度估计
  float pll_kp = (smo->pll_kp > 0.0f) ? smo->pll_kp : (2.0f * SMO_PLL_BW_HZ);
  float pll_ki = (smo->pll_ki > 0.0f) ? smo->pll_ki : (0.25f * pll_kp * pll_kp);
}
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| SMO 滑模增益设置 | ⚠️ 外部配置 | 通过 `smo->alpha` 配置，无自动计算 |
| 低通滤波截止频率 | ⚠️ 外部配置 | 通过 `smo->beta` 配置，无明确频率关系 |
| PLL 带宽 | ✅ 有默认值 | 默认 200Hz，可配置 |

### 改进建议

**1. 滑模增益自动计算**

```c
// 建议：根据电机参数自动计算滑模增益
// K_slide 应大于最大反电动势
float max_bemf = motor->parameters.flux * motor->Controller.vel_limit * 
                 motor->parameters.pole_pairs * M_2PI;
float K_slide = max_bemf * 1.5f;  // 1.5 倍裕量
```

**2. 滤波系数与截止频率关系**

```c
// 建议：用截止频率配置滤波
#define SMO_LPF_FC_HZ 500.0f  // 500Hz 截止频率
float K_filt = (M_2PI * SMO_LPF_FC_HZ * dt) / 
               (1.0f + M_2PI * SMO_LPF_FC_HZ * dt);
```

---

## 七、弱磁控制

### 评分: ⚠️ 一般

### 代码位置
`Src/ALGO/control/field_weakening.c`

### 当前实现

```c
static float FieldWeakening_CalcIdRef(const FieldWeakening_Config_t *cfg,
                                      float velocity) {
  float abs_vel = fabsf(velocity);
  if (abs_vel <= cfg->start_velocity) {
    return 0.0f;
  }
  
  // 线性弱磁
  float ratio = (abs_vel - cfg->start_velocity) / cfg->start_velocity;
  ratio = CLAMP(ratio, 0.0f, 1.0f);
  return -cfg->max_weakening_current * ratio;
}

void FieldWeakening_Update(MOTOR_DATA *motor, const FieldWeakening_Config_t *cfg) {
  float id_fw = FieldWeakening_CalcIdRef(cfg, motor->feedback.velocity);
  motor->algo_input.Id_ref = CLAMP(id_ref, -cfg->max_weakening_current, 
                                   cfg->max_weakening_current);
}
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| Id 给定策略 | ⚠️ 简单线性 | 基于速度的线性弱磁，未考虑电压饱和 |
| 进入/退出弱磁判断 | ⚠️ 仅速度判断 | 未检测电压饱和状态 |

### 改进建议

**推荐：基于电压饱和的弱磁控制**

```c
void FieldWeakening_Update_Advanced(MOTOR_DATA *motor, 
                                    const FieldWeakening_Config_t *cfg) {
  // 检测电压饱和
  if (motor->algo_output.voltage_saturated) {
    // 电压饱和时，增加负 Id
    float id_step = cfg->fw_ramp_rate * OUTER_LOOP_DT;
    motor->algo_input.Id_ref -= id_step;
  } else {
    // 未饱和时，缓慢恢复 Id 到 0
    float id_step = cfg->fw_ramp_rate * 0.1f * OUTER_LOOP_DT;
    if (motor->algo_input.Id_ref < 0.0f) {
      motor->algo_input.Id_ref += id_step;
      if (motor->algo_input.Id_ref > 0.0f) {
        motor->algo_input.Id_ref = 0.0f;
      }
    }
  }
  
  // 限幅
  motor->algo_input.Id_ref = CLAMP(motor->algo_input.Id_ref, 
                                   -cfg->max_weakening_current, 0.0f);
}
```

---

## 八、自动参数辨识

### 评分: ✅ 优秀

### 代码位置
- `Src/ALGO/motor/calib_resistance.c`
- `Src/ALGO/motor/calib_inductance.c`
- `Src/ALGO/motor/calib_encoder.c`

### 当前实现

**电阻辨识 (calib_resistance.c)**:
```c
// PI 控制器驱动电流到目标值
ctx->voltage += ctx->kI * dt * (CURRENT_MAX_CALIB - motor->algo_input.Ia);

// 计算电阻: R = V / I * (2/3)
motor->parameters.Rs = (ctx->voltage / CURRENT_MAX_CALIB) * (2.0f / 3.0f);
```

**电感辨识 (calib_inductance.c)**:
```c
// 交替施加正负电压
ctx->voltages[0] = -VOLTAGE_MAX_CALIB;
ctx->voltages[1] = +VOLTAGE_MAX_CALIB;

// 计算电感: L = V / (di/dt) * (2/3)
float dI_by_dt = (ctx->Ialphas[1] - ctx->Ialphas[0]) / dt;
motor->parameters.Ls = fabsf(VOLTAGE_MAX_CALIB / dI_by_dt * 2.0f / 3.0f);
```

**编码器偏置角校准 (calib_encoder.c)**:
```c
// 正反转采样，取平均消除齿槽效应
// CW 方向采样
ctx->error_array[ctx->sample_count] = error;
// CCW 方向采样
ctx->error_array[ctx->sample_count] = (ctx->error_array[ctx->sample_count] + error) / 2;

// 生成 LUT 查找表
for (int i = 0; i < OFFSET_LUT_NUM; i++) {
  // FIR 滤波生成平滑 LUT
}
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 自动测 R | ✅ 有 | PI 控制器稳态法 |
| 自动测 L | ✅ 有 | 电压阶跃法 |
| 编码器偏置角校准 | ✅ 优秀 | 正反转平均 + LUT 补偿 |
| 极对数自动检测 | ✅ 有 | 旋转 4 个电周期计数 |

**结论**: 参数辨识实现完整，编码器校准尤其出色（正反转平均 + LUT 补偿）。

---

## 九、sin/cos 精度

### 评分: ✅ 优秀

### 代码位置
`Src/ALGO/foc/trigonometry.c`

### 当前实现

```c
void Trig_FastSinCos(float angle, float *sin_val, float *cos_val) {
  // 角度归一化到 [-π, π]
  angle = Math_NormalizeAngle(angle);
  
  // 多项式近似 (Bhaskara I 改进版)
  if (angle < 0.0f) {
    *sin_val = 1.27323954f * angle + 0.405284735f * angle * angle;
    // 精度修正项
    if (*sin_val < 0.0f)
      *sin_val = 0.225f * (*sin_val * -*sin_val - *sin_val) + *sin_val;
    else
      *sin_val = 0.225f * (*sin_val * *sin_val - *sin_val) + *sin_val;
  }
  // ...
}
```

### 问题分析

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 实现方式 | 多项式近似 | 非查表，实时计算 |
| 精度 | ✅ 约 0.1% | 带精度修正项的 Bhaskara 近似 |
| 性能 | ✅ 优秀 | 无分支预测失败，无内存访问 |

### 精度验证

```
Bhaskara I 近似 (无修正): 最大误差 ~1.8%
带修正项后: 最大误差 ~0.1%

对于 FOC 应用，0.1% 精度完全足够。
```

**结论**: 三角函数实现优秀，使用带精度修正的多项式近似，比查表法更适合现代 MCU（无 cache miss）。

---

## 深度审查总结

| 审查项 | 评分 | 关键发现 |
|--------|------|----------|
| 一、死区补偿 | ⚠️ 一般 | 使用电压方向而非电流方向判断 |
| 二、电流采样方案 | ⚠️ 一般 | Offset 校准无温漂补偿 |
| 三、电流环带宽设计 | ✅ 优秀 | 带宽法计算正确，但自动模式带宽偏低 |
| 四、积分抗饱和策略 | ✅ 优秀 | Back-calculation + 圆形限幅 |
| 五、速度环滤波 | ⚠️ 一般 | 无速度反馈滤波，无前馈 |
| 六、无感观测器 | ⚠️ 一般 | 参数需手动配置，无自动计算 |
| 七、弱磁控制 | ⚠️ 一般 | 简单线性弱磁，未检测电压饱和 |
| 八、自动参数辨识 | ✅ 优秀 | R/L/编码器校准完整 |
| 九、sin/cos 精度 | ✅ 优秀 | 多项式近似，精度 0.1% |

### 优先改进项

1. **死区补偿** - ✅ 已修复：改用电流方向判断，影响低速扭矩精度
2. **速度环滤波** - ✅ 已修复：添加 100Hz 低通滤波，减少噪声放大
3. **弱磁控制** - ✅ 已修复：改用电压饱和检测 + 线性前馈组合，提高高速性能

### 可选改进项

4. ADC Offset 在线校准
5. SMO 参数自动计算
6. 电流环带宽下限保护

---

## 本次深度审查修改文件清单

| 文件 | 修改内容 |
|------|----------|
| `Src/ALGO/control/inner.c` | 死区补偿改用电流方向判断，添加过零平滑插值 |
| `Src/ALGO/control/outer.c` | 添加速度反馈 100Hz 低通滤波 |
| `Src/ALGO/control/field_weakening.c` | 添加基于电压饱和的弱磁控制，结合线性前馈 |
