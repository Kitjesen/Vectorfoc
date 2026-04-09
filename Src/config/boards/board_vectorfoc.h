// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file    boards/board_vectorfoc.h
 * @brief   VectorFOC G431 (STM32G431CBU6, 168MHz, MT6816) 全部硬件引脚定义
 *
 * ===  如何换板 ===
 * 不要改此文件。在 board_config.h 的路由处切换 include 目标，
 * 或在构建系统添加 -DBOARD_XSTAR 编译标志即可。
 *
 * === 宏命名规则 ===
 * 所有宏名称与 board_xstar.h 完全相同，只有值不同。
 * 上层代码应 #include "board_config.h"，永远不直接包含本文件。
 */
#ifndef BOARD_VECTORFOC_H
#define BOARD_VECTORFOC_H

#include "stm32g4xx_hal.h"

/* ==========================================================================
   板子标识
   ========================================================================== */
#define BOARD_NAME              "VectorFOC_G431"
#define BOARD_MCU               "STM32G431CBU6"
#define BOARD_REVISION          "Rev1.0"

/* ==========================================================================
   能力标志 — 上层用 #if HW_xxx_ENABLED 而非 #ifdef BOARD_xxx
   ========================================================================== */
#define HW_USB_ENABLED          1   /**< 有 USB CDC（PA11/PA12）            */
#define HW_DRV_ENABLED          0   /**< DRV8323 已定义引脚但驱动未实现     */

/* ==========================================================================
   1. 时钟
   ========================================================================== */
#define SYS_CLOCK_MHZ           168
#define SYS_CLOCK_HZ            (SYS_CLOCK_MHZ * 1000000UL)

/* ==========================================================================
   2. 电机 PWM — TIM1（中心对齐，死区互补）
   ========================================================================== */
#define HW_PWM_TIMER            htim1
#define HW_PWM_TIM_INSTANCE     TIM1
#define HW_PWM_FREQ_HZ          20000
#define HW_PWM_DEADTIME_CLKS    20

#define HW_PWM_CH_U             TIM_CHANNEL_1   /* PA8  */
#define HW_PWM_CH_V             TIM_CHANNEL_2   /* PA9  */
#define HW_PWM_CH_W             TIM_CHANNEL_3   /* PA10 */
#define HW_PWM_CH_TRIG          TIM_CHANNEL_4   /* ADC 注入触发 */

/* 上桥 */
#define HW_PWM_U_H_PIN          GPIO_PIN_8      /* PA8  TIM1_CH1  */
#define HW_PWM_U_H_PORT         GPIOA
#define HW_PWM_V_H_PIN          GPIO_PIN_9      /* PA9  TIM1_CH2  */
#define HW_PWM_V_H_PORT         GPIOA
#define HW_PWM_W_H_PIN          GPIO_PIN_10     /* PA10 TIM1_CH3  */
#define HW_PWM_W_H_PORT         GPIOA
/* 下桥 */
#define HW_PWM_U_L_PIN          GPIO_PIN_13     /* PB13 TIM1_CH1N */
#define HW_PWM_U_L_PORT         GPIOB
#define HW_PWM_V_L_PIN          GPIO_PIN_14     /* PB14 TIM1_CH2N */
#define HW_PWM_V_L_PORT         GPIOB
#define HW_PWM_W_L_PIN          GPIO_PIN_15     /* PB15 TIM1_CH3N */
#define HW_PWM_W_L_PORT         GPIOB

/* ==========================================================================
   3. 电流采样 — ADC1 注入组（TIM1_CC4 触发）
      Rank1(JDR1)=Ic  Rank2(JDR2)=Ib  Rank3(JDR3)=Ia  Rank4(JDR4)=Vbus
   ========================================================================== */
#define HW_ADC_CURRENT          hadc1
#define HW_ADC_CURRENT_INST     ADC1

/* 原始 JDR 偏移（供 motor_hal_g431.c 使用） */
#define HW_ADC_JDR_IC           JDR1    /* PA0 IN1  → 相C */
#define HW_ADC_JDR_IB           JDR2    /* PA1 IN2  → 相B */
#define HW_ADC_JDR_IA           JDR3    /* PA2 IN3  → 相A */
#define HW_ADC_JDR_VBUS         JDR4    /* PA3 IN4  → 母线电压 */

/**
 * 统一电流 ADC 访问宏 —— 与 board_xstar.h 宏名完全相同。
 * 用法：HW_ADC_IA_HANDLE.Instance->HW_ADC_IA_JDR
 * 本板三相均在 ADC1；X-STAR 的 IB 在 ADC2，通过 HW_ADC_IB_HANDLE 体现。
 */
#define HW_ADC_IA_HANDLE        HW_ADC_CURRENT  /* hadc1 → 相A */
#define HW_ADC_IA_JDR           HW_ADC_JDR_IA   /* JDR3 */
#define HW_ADC_IB_HANDLE        HW_ADC_CURRENT  /* hadc1 → 相B */
#define HW_ADC_IB_JDR           HW_ADC_JDR_IB   /* JDR2 */
#define HW_ADC_IC_HANDLE        HW_ADC_CURRENT  /* hadc1 → 相C */
#define HW_ADC_IC_JDR           HW_ADC_JDR_IC   /* JDR1 */
#define HW_ADC_VBUS_HANDLE      HW_ADC_CURRENT  /* hadc1 → 母线 */
#define HW_ADC_VBUS_JDR         HW_ADC_JDR_VBUS /* JDR4 */

/* 电流传感器 GPIO */
#define HW_ADC_IA_PIN           GPIO_PIN_2      /* PA2 */
#define HW_ADC_IA_PORT          GPIOA
#define HW_ADC_IA_CH            ADC_CHANNEL_3
#define HW_ADC_IB_PIN           GPIO_PIN_1      /* PA1 */
#define HW_ADC_IB_PORT          GPIOA
#define HW_ADC_IB_CH            ADC_CHANNEL_2
#define HW_ADC_IC_PIN           GPIO_PIN_0      /* PA0 */
#define HW_ADC_IC_PORT          GPIOA
#define HW_ADC_IC_CH            ADC_CHANNEL_1
#define HW_ADC_VBUS_PIN         GPIO_PIN_3      /* PA3 */
#define HW_ADC_VBUS_PORT        GPIOA
#define HW_ADC_VBUS_CH          ADC_CHANNEL_4

/* 模拟前端参数 */
#define HW_SHUNT_RESISTANCE     0.02f           /* 采样电阻 [Ω]        */
#define HW_OPAMP_GAIN           50.0f           /* 运放增益 [V/V]      */
#define HW_ADC_VREF             3.3f            /* ADC 参考电压 [V]    */
#define HW_ADC_RESOLUTION       4095            /* 12-bit 最大值       */
#define HW_ADC_MIDPOINT         1.65f           /* Vref/2 零点偏置 [V] */

/* 电流转换系数 [A/LSB]，由上面参数自动推导 */
#define HW_FAC_CURRENT  (HW_ADC_VREF / (HW_ADC_RESOLUTION * HW_SHUNT_RESISTANCE * HW_OPAMP_GAIN))

/* ==========================================================================
   4. 温度采样 — ADC2 DMA
   ========================================================================== */
#define HW_ADC_TEMP             hadc2
#define HW_ADC_TEMP_INST        ADC2
#define HW_TEMP_PIN             GPIO_PIN_2      /* PB2  */
#define HW_TEMP_PORT            GPIOB
#define HW_TEMP_CH              ADC_CHANNEL_12  /* ADC2_IN12 */
#define HW_NTC_R25              10000.0f        /* 10kΩ @25°C */
#define HW_NTC_B_VALUE          3950.0f
#define HW_NTC_PULLUP           10000.0f        /* 上拉电阻 */

/* ==========================================================================
   5. 母线电压分压
   ========================================================================== */
#define HW_VBUS_R_HIGH          10000.0f        /* 上分压 [Ω] */
#define HW_VBUS_R_LOW            1000.0f        /* 下分压 [Ω] */
#define HW_VBUS_DIVIDER_RATIO   ((HW_VBUS_R_HIGH + HW_VBUS_R_LOW) / HW_VBUS_R_LOW)
/* 电压转换系数 [V/LSB] */
#define HW_VOLTAGE_FACTOR       (HW_VBUS_DIVIDER_RATIO * (HW_ADC_VREF / HW_ADC_RESOLUTION))

/* ==========================================================================
   6. 位置传感器 — SPI1 磁编码器
      可选编码器型号（取消注释所需行，注释掉其余行）：
        HW_POSITION_SENSOR_MT6816  — AMR 14-bit，出厂默认
        HW_POSITION_SENSOR_TMR3109 — TMR 23-bit，推荐升级（同 SOP8 封装）
   ========================================================================== */
#define HW_POSITION_SENSOR_HALL     1u
#define HW_POSITION_SENSOR_ABZ      2u
#define HW_POSITION_SENSOR_MT6816   3u
#define HW_POSITION_SENSOR_TMR3109  4u   /**< MDT TMR3109，23-bit TMR */

/* ---- 选择编码器型号（修改此行即可切换） ---- */
#define HW_POSITION_SENSOR_MODE     HW_POSITION_SENSOR_MT6816
/* #define HW_POSITION_SENSOR_MODE  HW_POSITION_SENSOR_TMR3109 */

/* SPI 接口（MT6816 与 TMR3109 共用同一 SOP8 引脚，无需改线） */
#define HW_ENC_SPI              hspi1
#define HW_ENC_SPI_INST         SPI1
#define HW_ENC_CS_PIN           GPIO_PIN_4      /* PC4 */
#define HW_ENC_CS_PORT          GPIOC
#define HW_ENC_SCK_PIN          GPIO_PIN_5      /* PA5 */
#define HW_ENC_MISO_PIN         GPIO_PIN_6      /* PA6 */
#define HW_ENC_MOSI_PIN         GPIO_PIN_7      /* PA7 */

/* CPR 根据所选编码器自动选择 */
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
#  define HW_ENC_CPR            8388608u        /* 23-bit TMR3109 */
#else
#  define HW_ENC_CPR            16384u          /* 14-bit MT6816  */
#endif

/* HAL 与编码器实例名（供 motor_data.c 用，类型已在各驱动头文件中声明） */
#define HW_MOTOR_HAL_HANDLE     g431_hal_handle
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
#  define HW_ENCODER_DATA_SYMBOL  tmr3109_encoder_data
#else
#  define HW_ENCODER_DATA_SYMBOL  encoder_data
#endif

/* ==========================================================================
   7. 预驱动 SPI3 — DRV8323（已定义引脚，驱动未实现，HW_DRV_ENABLED=0）
   ========================================================================== */
#define HW_DRV_SPI              hspi3
#define HW_DRV_SPI_INST         SPI3
#define HW_DRV_CS_PIN           GPIO_PIN_12     /* PB12 */
#define HW_DRV_CS_PORT          GPIOB
#define HW_DRV_SCK_PIN          GPIO_PIN_3      /* PB3 */
#define HW_DRV_MISO_PIN         GPIO_PIN_4      /* PB4 */
#define HW_DRV_MOSI_PIN         GPIO_PIN_5      /* PB5 */

/* ==========================================================================
   8. CAN — FDCAN1
   ========================================================================== */
#define HW_CAN                  hfdcan1
#define HW_CAN_INST             FDCAN1
#define HW_CAN_RX_PIN           GPIO_PIN_8      /* PB8 */
#define HW_CAN_RX_PORT          GPIOB
#define HW_CAN_TX_PIN           GPIO_PIN_9      /* PB9 */
#define HW_CAN_TX_PORT          GPIOB
#define HW_CAN_BAUDRATE         1000000

/* ==========================================================================
   9. 调试串口 — USART1
   ========================================================================== */
#define HW_UART_DEBUG           huart1
#define HW_UART_DEBUG_INST      USART1
#define HW_UART_TX_PIN          GPIO_PIN_6      /* PB6 */
#define HW_UART_TX_PORT         GPIOB
#define HW_UART_RX_PIN          GPIO_PIN_7      /* PB7 */
#define HW_UART_RX_PORT         GPIOB
#define HW_UART_BAUDRATE        921600

/* ==========================================================================
   10. USB CDC — PA11/PA12
   ========================================================================== */
#define HW_USB_DM_PIN           GPIO_PIN_11     /* PA11 D- */
#define HW_USB_DP_PIN           GPIO_PIN_12     /* PA12 D+ */
#define HW_USB_PORT             GPIOA

/* ==========================================================================
   11. RGB LED — WS2812 via TIM3_CH2 DMA
   ========================================================================== */
#define HW_LED_TIMER            htim3
#define HW_LED_TIM_INSTANCE     TIM3
#define HW_LED_CHANNEL          TIM_CHANNEL_2
#define HW_LED_PIN              GPIO_PIN_4      /* PA4 */
#define HW_LED_PORT             GPIOA

/* ==========================================================================
   12. 状态 LED
   ========================================================================== */
#define HW_STATUS_LED_PIN       GPIO_PIN_13     /* PC13 */
#define HW_STATUS_LED_PORT      GPIOC

/* ==========================================================================
   外设句柄声明（CubeMX 生成）
   ========================================================================== */
extern TIM_HandleTypeDef    htim1;
extern TIM_HandleTypeDef    htim3;
extern ADC_HandleTypeDef    hadc1;
extern ADC_HandleTypeDef    hadc2;
extern SPI_HandleTypeDef    hspi1;
extern SPI_HandleTypeDef    hspi3;
extern FDCAN_HandleTypeDef  hfdcan1;
extern UART_HandleTypeDef   huart1;

#endif /* BOARD_VECTORFOC_H */
