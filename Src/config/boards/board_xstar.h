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
 * @file    boards/board_xstar.h
 * @brief   X-STAR-S G431 (STM32G431RBT6, 170MHz, Hall/ABZ) 全部硬件引脚定义
 *
 * 与 VectorFOC 主要差异：
 *   - PWM V下桥：PA12（而非 PB14），PA12 同时被 VectorFOC 用作 USB D+
 *   - 电流采样：3路内部 OPAMP 差分放大，ADC1+ADC2 双路注入读取
 *   - 采样电阻：3mΩ（原版 20mΩ）
 *   - 母线分压：75kΩ/3kΩ（原版 10kΩ/1kΩ）
 *   - NTC：47kΩ@25°C，下拉拓扑（原版上拉拓扑）
 *   - 位置传感器：Hall（TIM3）或 ABZ，无 MT6816
 *   - 调试串口：USART2 PB3/PB4（原版 USART1 PB6/PB7）
 *   - 无 USB，PA12 已被 PWM 占用
 *
 * === 宏命名规则 ===
 * 所有宏名称与 board_vectorfoc.h 完全相同，只有值不同。
 * 上层代码应 #include "board_config.h"，永远不直接包含本文件。
 */
#ifndef BOARD_XSTAR_H
#define BOARD_XSTAR_H

#include "stm32g4xx_hal.h"

/* ==========================================================================
   板子标识
   ========================================================================== */
#define BOARD_NAME              "XSTAR_S_G431"
#define BOARD_MCU               "STM32G431RBT6"
#define BOARD_REVISION          "V1.03"

/* ==========================================================================
   能力标志 — 上层用 #if HW_xxx_ENABLED 而非 #ifdef BOARD_xxx
   ========================================================================== */
#define HW_USB_ENABLED          0   /**< 无 USB：PA12 已被 PWM V下桥占用   */
#define HW_DRV_ENABLED          0   /**< 无外部预驱芯片，内置门极驱动       */

/* ==========================================================================
   1. 时钟
   ========================================================================== */
#define SYS_CLOCK_MHZ           170
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

/* 上桥（与 VectorFOC 相同） */
#define HW_PWM_U_H_PIN          GPIO_PIN_8      /* PA8  TIM1_CH1  */
#define HW_PWM_U_H_PORT         GPIOA
#define HW_PWM_V_H_PIN          GPIO_PIN_9      /* PA9  TIM1_CH2  */
#define HW_PWM_V_H_PORT         GPIOA
#define HW_PWM_W_H_PIN          GPIO_PIN_10     /* PA10 TIM1_CH3  */
#define HW_PWM_W_H_PORT         GPIOA
/* 下桥（注意：V相 PA12，与 VectorFOC 的 PB14 不同） */
#define HW_PWM_U_L_PIN          GPIO_PIN_13     /* PB13 TIM1_CH1N */
#define HW_PWM_U_L_PORT         GPIOB
#define HW_PWM_V_L_PIN          GPIO_PIN_12     /* PA12 TIM1_CH2N ← 与 VectorFOC 不同 */
#define HW_PWM_V_L_PORT         GPIOA
#define HW_PWM_W_L_PIN          GPIO_PIN_15     /* PB15 TIM1_CH3N */
#define HW_PWM_W_L_PORT         GPIOB

/* ==========================================================================
   3. 电流采样 — ADC1+ADC2 双路注入组（TIM1_CC4 触发）
      ADC1: Rank1(JDR1)=Iu  Rank2(JDR2)=Iw  Rank3(JDR3)=Vbus  Rank4(JDR4)=Temp
      ADC2: Rank1(JDR1)=Iv
   ========================================================================== */
#define HW_ADC_CURRENT          hadc1           /* 主 ADC 句柄（ADC1） */
#define HW_ADC_CURRENT_INST     ADC1
#define HW_ADC2_CURRENT         hadc2           /* Iv 相专用（ADC2）   */
#define HW_ADC2_CURRENT_INST    ADC2

/* ADC1 注入通道 */
#define HW_ADC1_CH_IU           ADC_CHANNEL_3   /* PA2  OPAMP1_VOUT → Iu */
#define HW_ADC1_CH_IW           ADC_CHANNEL_12  /* PB1  OPAMP3_VOUT → Iw */
#define HW_ADC1_CH_VBUS         ADC_CHANNEL_14  /* PB11 母线电压          */
#define HW_ADC1_CH_TEMP         ADC_CHANNEL_11  /* PB12 NTC温度           */

/* ADC1 JDR 偏移（供 motor_hal_xstar.c 使用） */
#define HW_ADC1_JDR_IU          JDR1
#define HW_ADC1_JDR_IW          JDR2
#define HW_ADC1_JDR_VBUS        JDR3
#define HW_ADC1_JDR_TEMP        JDR4

/* ADC2 注入通道 */
#define HW_ADC2_CH_IV           ADC_CHANNEL_3   /* PA6  OPAMP2_VOUT → Iv */
#define HW_ADC2_JDR_IV          JDR1

/**
 * 统一电流 ADC 访问宏 —— 与 board_vectorfoc.h 宏名完全相同。
 * 用法：HW_ADC_IA_HANDLE.Instance->HW_ADC_IA_JDR
 * 本板 IB（Iv相）在 ADC2；VectorFOC 三相均在 ADC1。
 */
#define HW_ADC_IA_HANDLE        HW_ADC_CURRENT  /* hadc1 → Iu（相A） */
#define HW_ADC_IA_JDR           HW_ADC1_JDR_IU  /* JDR1 */
#define HW_ADC_IB_HANDLE        HW_ADC2_CURRENT /* hadc2 → Iv（相B）← 与 VectorFOC 不同 */
#define HW_ADC_IB_JDR           HW_ADC2_JDR_IV  /* JDR1 */
#define HW_ADC_IC_HANDLE        HW_ADC_CURRENT  /* hadc1 → Iw（相C） */
#define HW_ADC_IC_JDR           HW_ADC1_JDR_IW  /* JDR2 */
#define HW_ADC_VBUS_HANDLE      HW_ADC_CURRENT  /* hadc1 → 母线 */
#define HW_ADC_VBUS_JDR         HW_ADC1_JDR_VBUS /* JDR3 */

/* 模拟前端参数 */
#define HW_SHUNT_RESISTANCE     0.003f          /* 采样电阻 [Ω]           */
#define HW_OPAMP_GAIN           10.0f           /* 差分运放增益 [V/V]     */
#define HW_ADC_VREF             3.3f
#define HW_ADC_RESOLUTION       4095
#define HW_ADC_MIDPOINT         1.65f           /* 差分放大器零点偏置 [V] */

/* 电流转换系数 [A/LSB] */
#define HW_FAC_CURRENT  (HW_ADC_VREF / (HW_ADC_RESOLUTION * HW_SHUNT_RESISTANCE * HW_OPAMP_GAIN))

/* ==========================================================================
   4. 母线电压 — PB11, ADC1 注入 Rank3
      分压：R54=75kΩ（上）, R14=3kΩ（下），最大可测 85.8V
   ========================================================================== */
#define HW_VBUS_R_HIGH          75000.0f
#define HW_VBUS_R_LOW            3000.0f
#define HW_VBUS_DIVIDER_RATIO   ((HW_VBUS_R_HIGH + HW_VBUS_R_LOW) / HW_VBUS_R_LOW)
#define HW_VOLTAGE_FACTOR       (HW_VBUS_DIVIDER_RATIO * (HW_ADC_VREF / HW_ADC_RESOLUTION))

/* ==========================================================================
   5. 温度 — PB12, ADC1 注入 Rank4
      NCP18WB473J03RB：47kΩ@25°C，下拉 10kΩ（+3.3V → NTC → R69 → GND）
   ========================================================================== */
#define HW_TEMP_PIN             GPIO_PIN_12     /* PB12 */
#define HW_TEMP_PORT            GPIOB
#define HW_NTC_R25              47000.0f
#define HW_NTC_B_VALUE          3950.0f
#define HW_NTC_PULLDOWN         10000.0f        /* 与 VectorFOC 拓扑相反，为下拉 */

/* ==========================================================================
   6. 位置传感器 — TIM3 Hall 或 ABZ（PC6/PC7/PC8）
   ========================================================================== */
#define HW_POSITION_SENSOR_HALL     1u
#define HW_POSITION_SENSOR_ABZ      2u
#define HW_POSITION_SENSOR_MT6816   3u

#ifndef HW_POSITION_SENSOR_MODE
#define HW_POSITION_SENSOR_MODE     HW_POSITION_SENSOR_HALL
#endif

#define HW_SENSOR_TIMER         htim3
#define HW_HALL_TIMER           HW_SENSOR_TIMER
#define HW_HALL_PORT            GPIOC
#define HW_HALL_HA_PIN          GPIO_PIN_6      /* PC6 TIM3_CH1 */
#define HW_HALL_HB_PIN          GPIO_PIN_7      /* PC7 TIM3_CH2 */
#define HW_HALL_HC_PIN          GPIO_PIN_8      /* PC8 TIM3_CH3 */

#define HW_ABZ_TIMER            HW_SENSOR_TIMER
#define HW_ABZ_PORT             GPIOC
#define HW_ABZ_A_PIN            GPIO_PIN_6      /* PC6 TIM3_CH1 */
#define HW_ABZ_B_PIN            GPIO_PIN_7      /* PC7 TIM3_CH2 */
#define HW_ABZ_Z_PIN            GPIO_PIN_8      /* PC8 */
#define HW_ABZ_Z_PORT           GPIOC
#ifndef HW_ABZ_CPR
#define HW_ABZ_CPR              4096u
#endif

/* HAL 与编码器实例名（供 motor_data.c 用） */
#define HW_MOTOR_HAL_HANDLE     xstar_hal_handle
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
#define HW_ENCODER_DATA_SYMBOL  hall_data
#else
#define HW_ENCODER_DATA_SYMBOL  abz_data
#endif

/* ==========================================================================
   7. CAN — FDCAN1（与 VectorFOC 相同）
   ========================================================================== */
#define HW_CAN                  hfdcan1
#define HW_CAN_INST             FDCAN1
#define HW_CAN_RX_PIN           GPIO_PIN_8      /* PB8 */
#define HW_CAN_RX_PORT          GPIOB
#define HW_CAN_TX_PIN           GPIO_PIN_9      /* PB9 */
#define HW_CAN_TX_PORT          GPIOB
#define HW_CAN_BAUDRATE         1000000

/* ==========================================================================
   8. 调试串口 — USART2（与 VectorFOC 的 USART1 不同）
   ========================================================================== */
#define HW_UART_DEBUG           huart2
#define HW_UART_DEBUG_INST      USART2
#define HW_UART_TX_PIN          GPIO_PIN_3      /* PB3 */
#define HW_UART_TX_PORT         GPIOB
#define HW_UART_RX_PIN          GPIO_PIN_4      /* PB4 */
#define HW_UART_RX_PORT         GPIOB
#define HW_UART_BAUDRATE        921600

/* ==========================================================================
   9. 状态 LED（与 VectorFOC 的 PC13 不同）
   ========================================================================== */
#define HW_STATUS_LED_PIN       GPIO_PIN_5      /* PB5 */
#define HW_STATUS_LED_PORT      GPIOB

/* ==========================================================================
   外设句柄声明（xstar_bsp.c / CubeMX 生成）
   ========================================================================== */
extern TIM_HandleTypeDef    htim1;
extern TIM_HandleTypeDef    htim3;
extern ADC_HandleTypeDef    hadc1;
extern ADC_HandleTypeDef    hadc2;
extern FDCAN_HandleTypeDef  hfdcan1;
extern UART_HandleTypeDef   huart2;

#endif /* BOARD_XSTAR_H */
