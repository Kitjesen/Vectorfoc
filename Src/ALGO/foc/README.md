# FOC 算法模块详细文档

## 📚 目录

- [概述](#概述)
- [目录结构](#目录结构)
- [核心算法](#核心算法)
  - [FOC 电流环算法](#foc-电流环算法)
  - [Clarke 坐标变换](#clarke-坐标变换)
  - [Park 坐标变换](#park-坐标变换)
  - [SVPWM 调制](#svpwm-调制)
- [数学公式详解](#数学公式详解)
- [性能优化](#性能优化)
- [使用示例](#使用示例)

---

## 概述

FOC (Field Oriented Control，磁场定向控制) 是一种高性能的交流电机控制方法。本模块实现了完整的 FOC 算法，包括：

- **坐标变换**：Clarke 变换 (abc → αβ)、Park 变换 (αβ → dq)
- **电流控制**：dq 轴独立 PI 控制器，带前馈解耦
- **PWM 调制**：空间矢量 PWM (SVPWM)
- **数学优化**：快速三角函数查找表

**核心优势**：
- ✅ **纯函数设计**：无硬件依赖，易于测试和移植
- ✅ **高性能**：优化的数学运算，支持 20kHz 控制频率
- ✅ **模块化**：清晰的接口，方便集成和扩展

---

## 目录结构

```
foc/
├── foc_algorithm.c/h       # FOC 主算法（电流环控制）
├── clarke.c/h              # Clarke 坐标变换
├── park.c/h                # Park 坐标变换
├── svpwm.c/h               # 空间矢量 PWM 调制
├── trigonometry.c/h        # 快速三角函数
└── math_common.h           # 数学常数与工具函数
```

---

## 核心算法

### FOC 电流环算法

FOC 电流环是整个算法的核心，实现了从三相电流反馈到三相 PWM 占空比的完整计算过程。

#### 算法流程

```
┌─────────────┐
│ Ia, Ib, Ic  │ 三相电流采样
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Clarke    │ abc → αβ 变换
│  Transform  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    Park     │ αβ → dq 变换（需要电角度 θ）
│  Transform  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   滤波器     │ 一阶低通滤波
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  PI 控制器   │ 误差 = Id_ref - Id, Iq_ref - Iq
│  (d轴/q轴)  │ Vd = Kp·err_d + ∫Ki·err_d
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  前馈解耦    │ 补偿 dq 轴耦合
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ 电压矢量限幅 │ 限制在可调制范围内
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  逆 Park    │ dq → αβ 变换
│  Transform  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    SVPWM    │ αβ → PWM 占空比
│  Modulate   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Ta, Tb, Tc  │ 三相 PWM 占空比输出
└─────────────┘
```

#### 关键代码

```c
void FOC_Algorithm_CurrentLoop(
    const FOC_AlgorithmInput_t *input,    // 输入：电流、角度、指令
    const FOC_AlgorithmConfig_t *config,  // 配置：电机参数、PI 参数
    FOC_AlgorithmState_t *state,          // 状态：积分项、滤波值
    FOC_AlgorithmOutput_t *output         // 输出：PWM 占空比
);
```

---

### Clarke 坐标变换

Clarke 变换将三相静止坐标系 (abc) 转换为两相静止坐标系 (αβ)，实现降维。

#### 数学公式

**正变换** (abc → αβ)：

```
[ Iα ]     2  [  1    -1/2   -1/2  ] [ Ia ]
[    ]  =  -  [                    ] [    ]
[ Iβ ]     3  [  0   √3/2  -√3/2  ] [ Ib ]
                                     [ Ic ]
```

简化形式（利用 Ia + Ib + Ic = 0）：

```
Iα = Ia
Iβ = (Ib - Ic) / √3
```

**逆变换** (αβ → abc)：

```
Va = Vα
Vb = -1/2·Vα + (√3/2)·Vβ
Vc = -1/2·Vα - (√3/2)·Vβ
```

#### 实现代码

```c
void Clarke_Transform(float Ia, float Ib, float Ic, 
                      float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = (Ib - Ic) * MATH_ONE_BY_SQRT3;  // 1/√3 预计算
}
```

**优化说明**：
- 使用简化公式，减少乘法运算
- 预计算常数 `1/√3`，避免除法

---

### Park 坐标变换

Park 变换将两相静止坐标系 (αβ) 转换为两相旋转坐标系 (dq)，使交流量变为直流量。

#### 数学公式

**正变换** (αβ → dq)：

```
[ Id ]   [  cosθ   sinθ ] [ Iα ]
[    ] = [              ] [    ]
[ Iq ]   [ -sinθ   cosθ ] [ Iβ ]
```

展开：

```
Id =  Iα·cosθ + Iβ·sinθ
Iq = -Iα·sinθ + Iβ·cosθ
```

**逆变换** (dq → αβ)：

```
[ Vα ]   [ cosθ  -sinθ ] [ Vd ]
[    ] = [             ] [    ]
[ Vβ ]   [ sinθ   cosθ ] [ Vq ]
```

展开：

```
Vα = Vd·cosθ - Vq·sinθ
Vβ = Vd·sinθ + Vq·cosθ
```

#### 实现代码

```c
void Park_Transform(float Ialpha, float Ibeta, float theta, 
                    float *Id, float *Iq)
{
    float sin_val, cos_val;
    Trig_FastSinCos(theta, &sin_val, &cos_val);  // 快速三角函数
    
    *Id = Ialpha * cos_val + Ibeta * sin_val;
    *Iq = -Ialpha * sin_val + Ibeta * cos_val;
}
```

**优化说明**：
- 使用快速三角函数（泰勒级数 + 查找表）
- `sin` 和 `cos` 同时计算，避免重复

---

### SVPWM 调制

空间矢量 PWM 是一种高效的 PWM 调制方法，可以最大化线性调制范围（提升 15.5%）。

#### 数学原理

**线性调制范围**：

- 传统 SPWM 最大幅值：`Vmax = Vbus/2`
- SVPWM 最大幅值：`Vmax = Vbus/√3 ≈ 0.577·Vbus`
- 提升比例：`(1/√3)/(1/2) = 2/√3 ≈ 1.155` (提升 15.5%)

#### 中点注入法

SVPWM 的实现采用**中点注入法**（也称为零序电压注入）：

**步骤1 - 归一化电压**：

```
V'α = Vα / (Vbus · 2/3)
V'β = Vβ / (Vbus · 2/3)
```

**步骤2 - 计算三相调制波**（逆 Clarke）：

```
Va = V'α
Vb = -1/2·V'α + (√3/2)·V'β
Vc = -1/2·V'α - (√3/2)·V'β
```

**步骤3 - 计算零序电压**（中点注入）：

```
Vcom = [max(Va,Vb,Vc) + min(Va,Vb,Vc)] / 2
```

**步骤4 - 生成最终占空比**：

```
Da = 0.5 + Va - Vcom
Db = 0.5 + Vb - Vcom
Dc = 0.5 + Vc - Vcom
```

#### 过调制处理

当电压矢量超出可调制范围时，按比例缩放：

```
if (Vmax - Vmin) > 1.0:
    scale = 1.0 / (Vmax - Vmin)
    Va, Vb, Vc *= scale
```

#### 实现代码

```c
int SVPWM_Modulate(float Valpha, float Vbeta, float Vbus, 
                   float *Ta, float *Tb, float *Tc)
{
    // 1. 归一化
    float mod_alpha = Valpha / (Vbus * MATH_2_BY_3);
    float mod_beta = Vbeta / (Vbus * MATH_2_BY_3);
    
    // 2. 逆 Clarke（计算三相调制波）
    float Va = mod_alpha;
    float Vb = -0.5f * mod_alpha + MATH_SQRT3_BY_2 * mod_beta;
    float Vc = -0.5f * mod_alpha - MATH_SQRT3_BY_2 * mod_beta;
    
    // 3. 寻找最大最小值 + 零序电压注入
    float Vmax = fmax(Va, fmax(Vb, Vc));
    float Vmin = fmin(Va, fmin(Vb, Vc));
    float Vcom = 0.5f * (Vmax + Vmin);
    
    // 4. 过调制处理
    if (Vmax - Vmin > 1.0f) {
        float scale = 1.0f / (Vmax - Vmin);
        Va *= scale; Vb *= scale; Vc *= scale;
        Vcom = 0.5f * (Vmax * scale + Vmin * scale);
    }
    
    // 5. 计算占空比
    *Ta = Math_Clamp(0.5f + Va - Vcom, 0.0f, 1.0f);
    *Tb = Math_Clamp(0.5f + Vb - Vcom, 0.0f, 1.0f);
    *Tc = Math_Clamp(0.5f + Vc - Vcom, 0.0f, 1.0f);
    
    return 0;
}
```

---

## 数学公式详解

### 1. 电流环 PI 控制器

**连续域控制律**：

```
V(s) = Kp·E(s) + (Ki/s)·E(s)
其中误差：E = Iref - Ifeedback
```

**离散化（前向欧拉）**：

```
Vpi(k) = Kp·e(k) + I(k)
I(k+1) = I(k) + Ki·e(k)·Ts
```

**抗积分饱和（Back-calculation）**：

当电压饱和时，回馈饱和误差到积分器：

```
I(k+1) = I(k) + Ki·e(k)·Ts + Kb·(Vsat - Vraw)·Ts
```

其中：
- `Kb`：抗饱和增益（通常取 Ki）
- `Vsat`：饱和后的电压
- `Vraw`：饱和前的电压

### 2. 前馈解耦

电机 dq 轴电压方程：

```
Vd = Rs·Id + Ls·(dId/dt) - ωe·Ls·Iq
Vq = Rs·Iq + Ls·(dIq/dt) + ωe·(Ls·Id + λ)
```

其中：
- `ωe`：电角速度
- `λ`：永磁体磁链

**解耦前馈项**：

```
Vd_ff = -ωe·Ls·Iq
Vq_ff = ωe·(Ls·Id + λ)
```

**总输出**：

```
Vd = Vd_pi + kff·Vd_ff
Vq = Vq_pi + kff·Vq_ff
```

其中 `kff` 是解耦增益，通常为 0.8~1.0。

### 3. 电流环带宽设计（IMC 方法）

根据内模控制（Internal Model Control）理论：

```
Kp = Ls · ωc
Ki = Rs · ωc
```

其中 `ωc = 2πfc` 是期望的闭环带宽（rad/s）。

**建议值**：
- 电流环带宽 fc：500 Hz ~ 2000 Hz
- 速度环带宽：电流环的 1/5 ~ 1/10
- 位置环带宽：速度环的 1/5 ~ 1/10

### 4. 电流反馈滤波器

一阶低通滤波器（离散化）：

```
Ifilt(k+1) = Ifilt(k) + α·(Imeas(k) - Ifilt(k))
```

其中滤波器系数：

```
α = (ωc·Ts) / (1 + ωc·Ts),  ωc = 2πfc
```

- `fc`：截止频率
- `Ts`：采样周期

---

## 性能优化

### 1. 快速三角函数

使用泰勒级数展开 + 查找表优化：

- **精度**：误差 < 0.1%
- **速度**：约 5 倍于 `math.h`
- **内存**：无查找表（纯计算）

### 2. 常数预计算

在 `math_common.h` 中预定义常用常数：

```c
#define MATH_PI              3.14159265358979323846f
#define MATH_SQRT3           1.73205080756887729352f
#define MATH_SQRT3_BY_2      0.86602540378443864676f
#define MATH_ONE_BY_SQRT3    0.57735026918962576451f
#define MATH_2_BY_3          0.66666666666666666667f
```

### 3. 内联函数

关键工具函数使用 `static inline`：

```c
static inline float Math_Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
```

---

## 使用示例

### 初始化

```c
// 1. 准备配置
FOC_AlgorithmConfig_t config = {
    .Rs = 0.5f,                    // 定子电阻 [Ω]
    .Ls = 0.001f,                  // 定子电感 [H]
    .flux = 0.01f,                 // 永磁体磁链 [Wb]
    .pole_pairs = 21,              // 极对数
    
    .Kp_current_d = 1.0f,          // d轴 Kp
    .Ki_current_d = 100.0f,        // d轴 Ki
    .Kp_current_q = 1.0f,          // q轴 Kp
    .Ki_current_q = 100.0f,        // q轴 Ki
    .Kb_current = 100.0f,          // 抗饱和增益
    
    .Ts_current = 0.00005f,        // 采样周期 (20kHz)
    .voltage_limit = 48.0f,        // 电压限制 [V]
    .current_limit = 10.0f,        // 电流限制 [A]
    
    .enable_decoupling = true,     // 启用解耦
    .decoupling_gain = 0.9f,       // 解耦增益
    .current_filter_fc = 1000.0f   // 滤波器截止频率 [Hz]
};

// 2. 验证配置
if (!FOC_Algorithm_ValidateConfig(&config)) {
    // 配置无效，处理错误
}

// 3. 初始化状态
FOC_AlgorithmState_t state;
FOC_Algorithm_InitState(&state);
```

### 运行时控制

```c
void FOC_CurrentLoop_IRQ(void) {
    // 1. 准备输入数据
    FOC_AlgorithmInput_t input = {
        .Ia = ADC_GetCurrent_A(),
        .Ib = ADC_GetCurrent_B(),
        .Ic = ADC_GetCurrent_C(),
        .Vbus = ADC_GetBusVoltage(),
        .theta_elec = Encoder_GetElecAngle(),
        .omega_elec = Encoder_GetElecVelocity(),
        
        .Id_ref = 0.0f,            // d轴电流指令（通常为0）
        .Iq_ref = torque_cmd / 1.5f / flux,  // q轴电流指令
        .enabled = motor_enabled
    };
    
    // 2. 执行 FOC 算法
    FOC_AlgorithmOutput_t output;
    FOC_Algorithm_CurrentLoop(&input, &config, &state, &output);
    
    // 3. 输出 PWM
    PWM_SetDuty_A(output.Ta);
    PWM_SetDuty_B(output.Tb);
    PWM_SetDuty_C(output.Tc);
    
    // 4. 检查状态
    if (output.voltage_saturated) {
        // 电压饱和，可能需要降低电流指令
    }
    if (output.current_limited) {
        // 电流已被限幅
    }
}
```

### PI 参数自动计算

```c
// 根据期望带宽自动计算 PI 参数
float Kp, Ki;
FOC_Algorithm_CalculateCurrentGains(
    motor_Rs,          // 电阻
    motor_Ls,          // 电感  
    800.0f,            // 期望带宽 800 Hz
    &Kp, &Ki
);

config.Kp_current_d = Kp;
config.Ki_current_d = Ki;
config.Kp_current_q = Kp;
config.Ki_current_q = Ki;
```

---

## 常见问题

### Q1: 为什么 Id_ref 通常设为 0？

在表贴式永磁同步电机（SPMSM）中，d 轴电流不产生转矩，只产生磁场，会增加铜损。因此通常令 `Id = 0`，仅通过 `Iq` 控制转矩（称为 `id = 0` 控制策略）。

### Q2: 如何调试 PI 参数？

1. 先设置一个较小的 Kp（如 0.1），Ki = 0
2. 逐步增大 Kp 直到出现振荡
3. 将 Kp 减半
4. 逐步增大 Ki 直到稳态误差消除
5. 可使用 IMC 方法快速计算初值

### Q3: 电流环带宽如何选择？

- **下限**：高于速度环带宽 5~10 倍
- **上限**：低于 PWM 频率的 1/10（避免混叠）
- **推荐值**：20kHz PWM → 500~1500Hz 带宽

---

## 参考资料

- **经典教材**：《矢量控制与直接转矩控制》- 张兴、杜少武
- **英文资料**：*Vector Control of AC Machines* - Peter Vas
- **应用手册**：[Texas Instruments - SVPWM Application Note](https://www.ti.com/lit/pdf/sprabs0)

---

**最后更新**：2026-01-31  
**维护者**：VectorFOC Project Team