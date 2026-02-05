# HAL Module (Hardware Abstraction Layer)

**Status**: ✅ Active - Production Ready

## 概述

HAL模块提供硬件抽象层，将电机控制核心逻辑与具体硬件实现解耦。支持不同MCU平台的移植。

## 目录结构

```
hal/
├── hal_abstraction.h/c    # HAL抽象层接口
├── hal_pwm.h/c            # PWM抽象接口
├── hal_adc.h/c            # ADC抽象接口
└── hal_encoder.h/c        # 编码器抽象接口
```

## 架构设计

```
┌─────────────────────────────┐
│   Motor Control Logic      │  motor/core/motor.c
│   (Platform Independent)   │
└─────────────────────────────┘
            ↓ 调用
┌─────────────────────────────┐
│    HAL Abstract Layer       │  hal/hal_*.h
│  (Generic Interfaces)       │
└─────────────────────────────┘
            ↓ 实现
┌─────────────────────────────┐
│  Platform-Specific Drivers  │  motor/hal/drivers/*.c
│      (STM32G4 specific)     │
└─────────────────────────────┘
            ↓ 使用
┌─────────────────────────────┐
│    Hardware Peripherals     │  BSP/*.c, STM32 HAL
└─────────────────────────────┘
```

---

## 模块说明

### 1. HAL抽象层 (`hal_abstraction.h/c`)

**功能**: 提供统一的硬件访问接口

**核心结构**:
```c
typedef struct {
    void (*pwm_set_duty)(uint8_t channel, float duty);
    float (*adc_read_current)(uint8_t phase);
    float (*encoder_read_angle)(void);
    // ...
} HAL_Interface_t;
```

**主要API**:
```c
void HAL_Init(const HAL_Interface_t *interface);
const HAL_Interface_t* HAL_GetInterface(void);
```

---

### 2. PWM抽象接口 (`hal_pwm.h/c`)

**功能**: 三相PWM输出控制

**接口定义**:
```c
// 初始化PWM
void HAL_PWM_Init(uint32_t frequency, uint16_t deadtime);

// 设置占空比 (0.0 ~ 1.0)
void HAL_PWM_SetDuty(float duty_a, float duty_b, float duty_c);

// 使能/禁用PWM输出
void HAL_PWM_Enable(void);
void HAL_PWM_Disable(void);

// 紧急停止
void HAL_PWM_EmergencyStop(void);
```

**使用示例**:
```c
// 初始化20kHz PWM，死区时间200ns
HAL_PWM_Init(20000, 200);

// 设置三相占空比
HAL_PWM_SetDuty(0.5f, 0.3f, 0.7f);

// 使能输出
HAL_PWM_Enable();

// 故障时紧急停止
if (fault_detected) {
    HAL_PWM_EmergencyStop();
}
```

---

### 3. ADC抽象接口 (`hal_adc.h/c`)

**功能**: 电流/电压采样

**接口定义**:
```c
// 初始化ADC
void HAL_ADC_Init(void);

// 读取相电流 [A]
float HAL_ADC_ReadCurrent_A(void);
float HAL_ADC_ReadCurrent_B(void);
float HAL_ADC_ReadCurrent_C(void);

// 读取母线电压 [V]
float HAL_ADC_ReadBusVoltage(void);

// 读取温度 [°C]
float HAL_ADC_ReadTemperature(void);

// 触发ADC采样
void HAL_ADC_TriggerSampling(void);

// 检查采样完成
bool HAL_ADC_IsSamplingComplete(void);
```

**使用示例**:
```c
// 在PWM中断中触发ADC
void TIM1_UP_IRQHandler(void) {
    HAL_ADC_TriggerSampling();
}

// 在ADC完成中断中读取
void ADC1_2_IRQHandler(void) {
    if (HAL_ADC_IsSamplingComplete()) {
        float Ia = HAL_ADC_ReadCurrent_A();
        float Ib = HAL_ADC_ReadCurrent_B();
        float Ic = HAL_ADC_ReadCurrent_C();
        float Vbus = HAL_ADC_ReadBusVoltage();
        
        // 执行FOC控制
        FOC_CurrentLoop(Ia, Ib, Ic, theta, Id_ref, Iq_ref);
    }
}
```

---

### 4. 编码器抽象接口 (`hal_encoder.h/c`)

**功能**: 位置/速度反馈

**接口定义**:
```c
// 初始化编码器
void HAL_Encoder_Init(void);

// 读取机械角度 [rad]
float HAL_Encoder_ReadMechanicalAngle(void);

// 读取电角度 [rad]
float HAL_Encoder_ReadElectricalAngle(uint8_t pole_pairs);

// 读取速度 [rad/s]
float HAL_Encoder_ReadVelocity(void);

// 校准编码器零点
int HAL_Encoder_Calibrate(uint8_t pole_pairs);

// 重置编码器
void HAL_Encoder_Reset(void);
```

**使用示例**:
```c
// 初始化并校准
HAL_Encoder_Init();
HAL_Encoder_Calibrate(7);  // 7极对电机

// 在控制循环中读取
float theta = HAL_Encoder_ReadElectricalAngle(7);
float omega = HAL_Encoder_ReadVelocity();

// 用于FOC和速度环
FOC_CurrentLoop(..., theta, ...);
VelocityController_Update(omega_ref, omega);
```

---

## 平台移植指南

### 移植到新平台的步骤

#### 1. 实现驱动层

在`motor/hal/drivers/`创建平台特定实现：

```c
// motor/hal/drivers/pwm_stm32f4.c
void PWM_STM32F4_Init(uint32_t freq, uint16_t deadtime) {
    // STM32F4特定的PWM初始化
}

void PWM_STM32F4_SetDuty(float a, float b, float c) {
    // STM32F4特定的占空比设置
}
```

#### 2. 注册HAL接口

```c
// motor/hal/motor_hal_stm32f4.c
static const HAL_Interface_t stm32f4_hal = {
    .pwm_init = PWM_STM32F4_Init,
    .pwm_set_duty = PWM_STM32F4_SetDuty,
    .adc_read_current = ADC_STM32F4_ReadCurrent,
    .encoder_read_angle = Encoder_STM32F4_ReadAngle,
    // ...
};

void MotorHAL_STM32F4_Init(void) {
    HAL_Init(&stm32f4_hal);
}
```

#### 3. 更新构建系统

在`CMakeLists.txt`中添加条件编译：

```cmake
if(PLATFORM STREQUAL "STM32F4")
    set(HAL_SOURCES
        motor/hal/motor_hal_stm32f4.c
        motor/hal/drivers/pwm_stm32f4.c
        motor/hal/drivers/adc_stm32f4.c
    )
endif()
```

---

## 当前平台支持

### STM32G431 (当前实现)

**文件**: `motor/hal/motor_hal_g431.c`

**硬件资源**:
- **PWM**: TIM1 (Advanced Timer)
  - CH1/CH1N: Phase A
  - CH2/CH2N: Phase B
  - CH3/CH3N: Phase C
  - 频率: 20kHz
  - 死区: 200ns

- **ADC**: ADC1 + ADC2 (Dual Mode)
  - IN1: Phase A Current
  - IN2: Phase B Current
  - IN3: Phase C Current
  - IN4: Bus Voltage
  - 采样: 20kHz (PWM同步)

- **编码器**: SPI3
  - MT6816磁编码器
  - 时钟: 10MHz

**性能指标**:
| 指标 | 值 |
|------|-----|
| PWM频率 | 20kHz |
| ADC采样率 | 20kHz |
| 编码器更新 | 20kHz |
| 控制延迟 | <50μs |

---

## 设计原则

### 1. 平台无关性

核心控制逻辑不应包含任何硬件特定代码：

✅ **正确**:
```c
// motor/core/motor.c
float theta = HAL_Encoder_ReadElectricalAngle(pole_pairs);
```

❌ **错误**:
```c
// motor/core/motor.c
float theta = MT6816_ReadElectricalAngle(pole_pairs);  // 硬件特定
```

### 2. 接口一致性

所有平台实现必须提供相同的接口：

```c
// 所有平台都必须实现
void HAL_PWM_SetDuty(float a, float b, float c);
```

### 3. 性能优化

HAL层应尽量轻量，避免不必要的开销：

```c
// 内联小函数
static inline void HAL_PWM_SetDuty_Fast(float a, float b, float c) {
    TIM1->CCR1 = (uint16_t)(a * PWM_ARR);
    TIM1->CCR2 = (uint16_t)(b * PWM_ARR);
    TIM1->CCR3 = (uint16_t)(c * PWM_ARR);
}
```

---

## 故障排查

### 问题1: PWM输出异常

**检查清单**:
- [ ] PWM引脚配置正确
- [ ] 死区时间设置合理
- [ ] 互补输出极性正确
- [ ] 刹车功能配置

### 问题2: ADC采样不准确

**检查清单**:
- [ ] ADC参考电压稳定
- [ ] 采样时间足够
- [ ] 运放增益正确
- [ ] 偏置电压校准

### 问题3: 编码器读取错误

**检查清单**:
- [ ] SPI时钟频率合适
- [ ] SPI模式配置正确
- [ ] 编码器供电正常
- [ ] 磁铁气隙合适

---

## 性能优化建议

### 1. 使用DMA

```c
// 使用DMA传输ADC数据
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 4);
```

### 2. 中断优先级

```c
// 确保PWM/ADC中断优先级最高
HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 1);
```

### 3. 内联关键函数

```c
__attribute__((always_inline))
static inline void HAL_PWM_SetDuty_Inline(...) {
    // 关键路径代码
}
```

---

## 与其他模块的关系

```
motor/core/motor.c (控制逻辑)
    ↓ 调用
hal/hal_*.h (抽象接口)
    ↓ 实现
motor/hal/motor_hal_g431.c (平台适配)
    ↓ 调用
motor/hal/drivers/*.c (驱动实现)
    ↓ 使用
BSP/*.c (硬件外设)
```

---

## 注意事项

> [!IMPORTANT]
> **中断安全**: HAL函数可能在中断中调用，必须保证线程安全

> [!WARNING]
> **时序要求**: PWM和ADC必须严格同步，采样点应在PWM中心

> [!TIP]
> **移植建议**: 先实现基本功能，验证后再优化性能
