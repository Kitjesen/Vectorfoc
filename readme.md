# VectorFOC 电机控制器固件

![VectorFOC](fig/foc.png)

> 高性能无刷电机 FOC 驱动器，适用于机器人关节、工业伺服、协作机器人等场景。

[![Platform](https://img.shields.io/badge/MCU-STM32G431-blue?style=flat-square)]()
[![Language](https://img.shields.io/badge/language-C-orange?style=flat-square)]()
[![License](https://img.shields.io/badge/license-MIT-green?style=flat-square)]()
[![Control](https://img.shields.io/badge/电流环-20kHz-red?style=flat-square)]()
[![Encoder](https://img.shields.io/badge/编码器-14bit-blueviolet?style=flat-square)]()

---

## 功能概览

- **FOC 矢量控制** — Clarke / Park / SVPWM 全链路，20kHz 电流环
- **7 种控制模式** — 开环、力矩、速度、位置、速度斜坡、位置斜坡、MIT 阻抗
- **3 种通信协议** — Inovxio（主推）、MIT Cheetah、CANopen DS402
- **11 种故障保护** — 过压/欠压/过流/过温/堵转/超速/通信看门狗
- **自动校准** — 电流零点、电阻电感、极对数、编码器 LUT、磁链、齿槽补偿
- **参数管理** — Flash 持久化、CAN 远程读写、结构化参数表

---

## 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | STM32G431 (Cortex-M4F @ 170MHz) |
| 编码器 | MT6816 14-bit SPI 磁编码器 |
| 电流环频率 | 20kHz |
| CAN 速率 | 1Mbps |
| 电压范围 | 12V – 60V |
| 电流采样 | 0.02Ω 采样电阻，50 倍运放增益 |

---

## 快速开始

### 编译

```bash
git clone https://github.com/Kitjesen/Vectorfoc.git
cd Vectorfoc

mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build . -j8
```

也可用 Keil MDK 打开 `MDK-ARM/VectorFOC.uvprojx` 直接编译。

### 烧录

使用 STM32CubeProgrammer 或 OpenOCD 烧录 `VectorFoc.elf`。

### 运行

```bash
# 触发校准
cansend can0 008#00

# 保存参数
cansend can0 016#

# 使能电机
cansend can0 003#
```

---

## 控制模式

| 模式 | 说明 | 典型场景 |
|------|------|----------|
| 开环 | 直接电压控制 | 调试 |
| 力矩 | 电流闭环 | 力控、张力控制 |
| 速度 | 速度闭环 | 风机、输送带 |
| 位置 | 位置闭环 | 精密定位 |
| 速度斜坡 | 带加减速限制 | 平滑启停 |
| 位置斜坡 | 梯形轨迹规划 | 点到点运动 |
| MIT 阻抗 | Kp + Kd + 前馈力矩 | 机器人关节 |

---

## 项目结构

```
VectorFOC/
├── Src/                    # 自定义代码
│   ├── APP/                #   应用层（初始化、ISR、RTOS 任务）
│   ├── ALGO/               #   算法层（FOC、PID、电机状态机、观测器）
│   ├── HAL/                #   硬件抽象（ADC、PWM、编码器、BSP）
│   ├── COMM/               #   通信层（协议管理、命令执行、传输抽象）
│   └── UI/                 #   用户接口（参数系统、错误管理、LED、VOFA+）
├── Lib/                    # 第三方 / CubeMX 生成代码
│   ├── Core/               #   HAL 外设初始化
│   ├── Drivers/            #   ST HAL + CMSIS
│   ├── Middlewares/        #   FreeRTOS、USB
│   └── USB_Device/         #   USB CDC
├── MDK-ARM/                # Keil 工程
├── test/                   # 单元测试
└── CMakeLists.txt          # CMake 构建配置
```

---

## 配置

电机参数在 `Src/ALGO/motor/config.h`：

```c
#define MOTOR_POLE_PAIRS    7       // 极对数
#define MOTOR_RESISTANCE    0.5f    // 相电阻 [Ω]
#define MOTOR_INDUCTANCE    0.001f  // 相电感 [H]
#define MOTOR_FLUX          0.08f   // 磁链 [Wb]
```

首次使用建议设置 `PRE_CALIBRATED 0` 触发自动校准，校准完成后改为 `1`。

---

## 许可证

MIT License

## 致谢

- STM32 HAL 库
- MIT Cheetah 开源项目
