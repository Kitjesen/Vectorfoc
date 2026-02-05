# Encoder Module

**Status**: ✅ Active - Production Ready

## 1. 概述 (Overview)

本模块实现了磁编码器驱动接口，核心支持 **MT6816**（14-bit 磁绝对位置传感器）。驱动包含完整的 **SPI通信**、**奇偶校验**、**线性化补偿** 以及 **PLL（锁相环）位置/速度估算**，专为高性能 FOC 控制设计。

## 2. 目录结构 (Directory Structure)

```
encoder/
├── MT6816/
│   ├── mt6816_encoder.h  // 核心头文件：定义句柄、API、配置宏
│   └── mt6816_encoder.c  // 核心实现：SPI读取、PLL滤波、角度计算
└── README.md             // 说明文档
```

## 3. MT6816 驱动详解

### 3.1 核心特性

- **分辨率**: 14-bit (0~16383 count)
- **绝对精度**: ±0.1° (取决于磁铁安装)
- **通信接口**: standard SPI (Mode 3, CPOL=1, CPHA=1 usually supported), 读取耗时 < 10us
- **信号处理**:
  - 集成 PLL 估算器（默认带宽 1000Hz），提供平滑的速度和插值位置
  - 支持多圈计数 (64-bit Shadow Counter)
  - 支持 128 点线性化 LUT 补偿

### 3.2 数据结构 (`MT6816_Handle_t`)

驱动使用面向对象风格封装，通过 `MT6816_Handle_t` 句柄管理所有状态。主要成员如下：

```c
typedef struct MT6816_Handle_t {
    // --- 硬件配置 ---
    SPI_HandleTypeDef *hspi;    // HAL SPI 句柄
    GPIO_TypeDef *cs_port;      // 片选 Port
    uint16_t cs_pin;            // 片选 Pin

    // --- 用户配置 ---
    uint8_t pole_pairs;         // 电机极对数 (用于电角度计算)
    MT6816_Direction_t dir;     // 方向 (MT6816_DIR_CW / MT6816_DIR_CCW)
    int32_t offset_counts;      // 零点偏移值 (机械角度校准用)
    float pll_bandwidth;        // PLL带宽 (Hz), 控制速度响应与噪声抑制

    // --- 实时输出 (只读) ---
    uint16_t raw_angle;         // 原始SPI数值 (0-16383)
    float mec_angle_rad;        // 机械角度 [0, 2π)
    float elec_angle_rad;       // 电角度 [-π, π] (用于FOC Park变换)
    float velocity_rad_s;       // 机械角速度 [rad/s] (PLL估算)
    int64_t shadow_count;       // 多圈累积计数值
    MT6816_Status_t last_status;// 通信状态 (OK/SPI_ERR/PARITY_ERR)
  
    // ... (内部PLL状态变量)
} MT6816_Handle_t;
```

### 3.3 关键 API

#### 3.3.1 初始化

```c
void MT6816_Init(MT6816_Handle_t *encoder, SPI_HandleTypeDef *hspi, 
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);
```

- **配置**: 绑定 SPI 句柄和 CS 引脚，初始化内部状态。
- **默认值**: 极对数默认 7，PLL 带宽默认 1000Hz (可通过宏 `ENCODER_PLL_BANDWIDTH` 修改)。

#### 3.3.2 周期性更新

```c
MT6816_Status_t MT6816_Update(MT6816_Handle_t *encoder, float dt);
```

- **调用位置**: 必须在电机控制的中断服务函数 (ISR) 中调用。
- **参数 `dt`**: 采样周期（秒），例如 20kHz 频率下为 0.00005f。
- **功能**:
  1. 触发 SPI 读取并校验数据。
  2. 运行 PLL 估算速度和插值位置。
  3. 计算机械角度和电角度。

---

## 4. 快速使用指南 (Quick Start)

以下代码展示如何在 STM32 HAL 环境下集成该模块。

### Step 1: 变量定义

在 `main.c` 或电机任务文件中：

```c
#include "encoder/MT6816/mt6816_encoder.h"

// 定义编码器句柄
MT6816_Handle_t h_encoder;

// 引入生成的 SPI 句柄 (通常在 main.c 或 spi.c)
extern SPI_HandleTypeDef hspi1; 
```

### Step 2: 初始化

在 `main()` 函数的初始化部分：

```c
void System_Init() {
    // ... HAL_Init, SystemClock_Config, MX_SPI1_Init ...

    // 初始化编码器
    MT6816_Init(&h_encoder, &hspi1, GPIOB, GPIO_PIN_0); // 假设 CS 接在 PB0

    // 配置电机参数
    h_encoder.pole_pairs = 11;       // 例如：11对极电机
    h_encoder.dir = MT6816_DIR_CW;  // 设置正方向
}
```

### Step 3: 高频控制循环

在定时器中断或 FOC 任务中 (e.g., 20kHz):

```c
void FOC_Loop() {
    float dt = 50e-6f; // 50us

    // 1. 更新编码器状态
    MT6816_Status_t status = MT6816_Update(&h_encoder, dt);

    if (status != MT6816_OK) {
        // 错误处理: 比如通信失败连续发生时停止电机
        Error_Handler(); 
    }

    // 2. 获取角度用于 FOC 计算
    float theta_elec = h_encoder.elec_angle_rad;
    float speed = h_encoder.velocity_rad_s;

    // 3. 执行 FOC 算法
    FOC_Run(theta_elec, speed);
}
```

---

## 5. 校准 (Calibration)

实际应用中，编码器的 0 刻度通常不对齐电机的电角度 0 位置。需要进行**对齐校准**。

### 手动校准原理

1. 给电机强行注入 D 轴电流（电压矢量定位），将转子锁定在电角度 0 位置。
2. 读取此时编码器的 `raw_angle` 或内部计数值。
3. 将该值设为 `offset_counts`。

### 代码实现建议

```c
void Calibrate_Encoder_Offset() {
    // 1. 强行定位由 FOC 模块完成 (设置 Id=2A, Iq=0)
    FOC_Lock_Rotor(); 
    HAL_Delay(1000); // 等待稳定

    // 2. 读取当前原始物理值
    // 注意：这里我们读取 raw_angle，它是 0-16383 的值
    h_encoder.offset_counts = h_encoder.raw_angle;

    // 3. 保存 offset_counts 到 Flash，下次上电读取
    Save_To_Flash(h_encoder.offset_counts);
}
```

## 6. 性能优化与调试 (Tips)

1. **PLL 带宽调整**:

   - 如果速度读数噪声大，**减小** `pll_bandwidth` (e.g., 500Hz).
   - 如果需要更快的动态响应，**增加** `pll_bandwidth` (e.g., 4000Hz).
   - 修改宏 `ENCODER_PLL_BANDWIDTH` 或直接修改 `h_encoder.pll_bandwidth`.
2. **SPI 速率**:

   - MT6816 支持高达 25MHz SPI，但建议使用 5MHz - 10MHz 以保证信号完整性，减少干扰。
3. **异常处理**:

   - 检查 `h_encoder.rx_err_count` 和 `check_err_count` 确认是否存在 SPI 通信干扰。
   - 常见问题：连线过长导致数据位错误，建议使用双绞线或降低 SPI 频率。
