/**
 * @file board_config_xstar.h
 * @brief X-STAR-S STM32G431RBT6 FOC&BLDC 开发板硬件配置
 *
 * 使用方法：编译时加 -DBOARD_XSTAR，即可替换默认的 board_config.h
 *
 * 与 VectorFOC 原版板的主要差异：
 *   1. PWM V下桥：PA12 (CH2N)，而非 PB14
 *   2. 电流采样：3路内部OPAMP差分放大（增益10x），ADC1+ADC2双ADC读取
 *   3. 采样电阻：3mΩ（原版 20mΩ）
 *   4. 母线电压分压：75kΩ/3kΩ（原版 10kΩ/1kΩ）
 *   5. NTC：NCP18WB473J03RB，47kΩ@25°C，下拉10kΩ（拓扑与原版相反）
 *   6. 位置传感器：霍尔传感器（TIM3，PC6/PC7/PC8），无磁编码器
 *
 * ADC 注入组分配（需与 CubeMX 配置一致）：
 *   ADC1 Rank1: OPAMP1_VOUT → Iu相电流
 *   ADC1 Rank2: OPAMP3_VOUT → Iw相电流
 *   ADC1 Rank3: PB11 (IN14)  → 母线电压
 *   ADC1 Rank4: PB12 (IN11)  → NTC温度
 *   ADC2 Rank1: OPAMP2_VOUT → Iv相电流
 *   触发源：TIM1_CC4（与PWM同步）
 */
#ifndef BOARD_CONFIG_XSTAR_H
#define BOARD_CONFIG_XSTAR_H

#include "stm32g4xx_hal.h"

/* ==========================================================================
   板子标识
   ========================================================================== */
#define BOARD_NAME          "XSTAR_S_G431"
#define BOARD_MCU           "STM32G431RBT6"
#define BOARD_REVISION      "V1.03"

/* ==========================================================================
   1. 时钟
   ========================================================================== */
#define SYS_CLOCK_MHZ       170
#ifdef SYS_CLOCK_HZ
#undef SYS_CLOCK_HZ
#endif
#define SYS_CLOCK_HZ        (SYS_CLOCK_MHZ * 1000000UL)

/* ==========================================================================
   2. 电机PWM — TIM1（中心对齐，死区互补）
   ========================================================================== */
#define HW_PWM_TIMER            htim1
#define HW_PWM_TIM_INSTANCE     TIM1
#define HW_PWM_FREQ_HZ          20000
#define HW_PWM_DEADTIME_CLKS    20

/* 相序：CH1=U, CH2=V, CH3=W */
#define HW_PWM_CH_U         TIM_CHANNEL_1
#define HW_PWM_CH_V         TIM_CHANNEL_2
#define HW_PWM_CH_W         TIM_CHANNEL_3
#define HW_PWM_CH_TRIG      TIM_CHANNEL_4   /* ADC注入触发 */

/* 上桥引脚（与VectorFOC相同） */
#define HW_PWM_U_H_PIN      GPIO_PIN_8      /* PA8  TIM1_CH1  */
#define HW_PWM_U_H_PORT     GPIOA
#define HW_PWM_V_H_PIN      GPIO_PIN_9      /* PA9  TIM1_CH2  */
#define HW_PWM_V_H_PORT     GPIOA
#define HW_PWM_W_H_PIN      GPIO_PIN_10     /* PA10 TIM1_CH3  */
#define HW_PWM_W_H_PORT     GPIOA

/* 下桥引脚（注意：V相 PA12，与VectorFOC的PB14不同！） */
#define HW_PWM_U_L_PIN      GPIO_PIN_13     /* PB13 TIM1_CH1N */
#define HW_PWM_U_L_PORT     GPIOB
#define HW_PWM_V_L_PIN      GPIO_PIN_12     /* PA12 TIM1_CH2N ← 不同于VectorFOC */
#define HW_PWM_V_L_PORT     GPIOA
#define HW_PWM_W_L_PIN      GPIO_PIN_15     /* PB15 TIM1_CH3N */
#define HW_PWM_W_L_PORT     GPIOB

/* ==========================================================================
   3. 电流采样 — ADC1 注入组（OPAMP1/3输出 + 母线 + 温度）
   ========================================================================== */
#define HW_ADC_CURRENT          hadc1
#define HW_ADC_CURRENT_INST     ADC1

/* ADC1 注入序列通道（与 xstar_bsp.c 中 Rank 配置一致）
 *   Rank1: PA2  = ADC1_IN3  = OPAMP1_VOUT → Iu
 *   Rank2: PB1  = ADC1_IN12 = OPAMP3_VOUT → Iw
 *   Rank3: PB11 = ADC1_IN14               → Vbus
 *   Rank4: PB12 = ADC1_IN11               → NTC温度
 */
#define HW_ADC1_CH_IU       ADC_CHANNEL_3   /* PA2  OPAMP1_VOUT */
#define HW_ADC1_CH_IW       ADC_CHANNEL_12  /* PB1  OPAMP3_VOUT */
#define HW_ADC1_CH_VBUS     ADC_CHANNEL_14  /* PB11 母线电压     */
#define HW_ADC1_CH_TEMP     ADC_CHANNEL_11  /* PB12 NTC温度      */

/* JDR 寄存器偏移宏（用于 motor_hal_xstar.c 直接寄存器读取） */
#define HW_ADC1_JDR_IU      JDR1
#define HW_ADC1_JDR_IW      JDR2
#define HW_ADC1_JDR_VBUS    JDR3
#define HW_ADC1_JDR_TEMP    JDR4

/* ADC2 注入序列通道
 *   Rank1: PA6 = ADC2_IN3 = OPAMP2_VOUT → Iv
 */
#define HW_ADC2_CURRENT         hadc2
#define HW_ADC2_CURRENT_INST    ADC2
#define HW_ADC2_CH_IV       ADC_CHANNEL_3   /* PA6  OPAMP2_VOUT */
#define HW_ADC2_JDR_IV      JDR1

/* 模拟前端参数 */
#define HW_SHUNT_RESISTANCE 0.003f  /* 采样电阻：3mΩ */
#define HW_OPAMP_GAIN       10.0f   /* 差分运放增益：10倍（R_fb/R_in = 10kΩ/1kΩ） */
#define HW_ADC_VREF         3.3f
#define HW_ADC_RESOLUTION   4095
#define HW_ADC_MIDPOINT     1.65f   /* 偏置电压（差分放大器零点） */

/* 电流转换系数：A/LSB = Vref / (Resolution × R_shunt × Gain) */
#define HW_FAC_CURRENT      (HW_ADC_VREF / (HW_ADC_RESOLUTION * HW_SHUNT_RESISTANCE * HW_OPAMP_GAIN))

/* ==========================================================================
   4. 母线电压 — PB11, ADC1_IN14（注入组Rank3）
      分压：R54=75kΩ（上）, R14=3kΩ（下），最大可测85.8V
   ========================================================================== */
#define HW_VBUS_R_HIGH      75000.0f    /* R54 */
#define HW_VBUS_R_LOW        3000.0f    /* R14 */
#define HW_VBUS_DIVIDER_RATIO   ((HW_VBUS_R_HIGH + HW_VBUS_R_LOW) / HW_VBUS_R_LOW)  /* = 26 */

/* 电压转换系数：V/LSB */
#define HW_VOLTAGE_FACTOR   (HW_VBUS_DIVIDER_RATIO * (HW_ADC_VREF / HW_ADC_RESOLUTION))

/* ==========================================================================
   5. NTC温度 — PB12, ADC1_IN11（注入组Rank4）
      型号：NCP18WB473J03RB，R25=47kΩ，B=3950
      电路：+3.3V → NTC → TEMP → R69(10kΩ) → GND
      注意：NTC在上方（与VectorFOC的下方拓扑相反）
   ========================================================================== */
#define HW_TEMP_PIN         GPIO_PIN_12     /* PB12 */
#define HW_TEMP_PORT        GPIOB
#define HW_NTC_R25          47000.0f        /* 47kΩ @25°C */
#define HW_NTC_B_VALUE      3950.0f         /* B值 */
#define HW_NTC_PULLDOWN     10000.0f        /* R69 下拉电阻 */
/* ADC读数 → NTC阻值：R_ntc = R_down × (ADC_MAX - raw) / raw */
/* 温度计算用Steinhart-Hart简化公式（math.h logf）*/

/* ==========================================================================
   6. 霍尔传感器 / 编码器 — TIM3（PC6/PC7/PC8）
   ========================================================================== */
#define HW_POSITION_SENSOR_HALL 1u
#define HW_POSITION_SENSOR_ABZ  2u

#ifndef HW_POSITION_SENSOR_MODE
#define HW_POSITION_SENSOR_MODE HW_POSITION_SENSOR_HALL
#endif

#define HW_SENSOR_TIMER     htim3

#define HW_HALL_TIMER       HW_SENSOR_TIMER
#define HW_HALL_PORT        GPIOC
#define HW_HALL_HA_PIN      GPIO_PIN_6      /* PC6 TIM3_CH1 */
#define HW_HALL_HB_PIN      GPIO_PIN_7      /* PC7 TIM3_CH2 */
#define HW_HALL_HC_PIN      GPIO_PIN_8      /* PC8 TIM3_CH3 */

#define HW_ABZ_TIMER        HW_SENSOR_TIMER
#define HW_ABZ_PORT         GPIOC
#define HW_ABZ_A_PIN        GPIO_PIN_6      /* PC6 TIM3_CH1 */
#define HW_ABZ_B_PIN        GPIO_PIN_7      /* PC7 TIM3_CH2 */
#define HW_ABZ_Z_PIN        GPIO_PIN_8      /* PC8 GPIO input */
#define HW_ABZ_Z_PORT       GPIOC

#ifndef HW_ABZ_CPR
#define HW_ABZ_CPR          4096u
#endif

/* ==========================================================================
   7. CAN — FDCAN1（与VectorFOC相同）
   ========================================================================== */
#define HW_CAN              hfdcan1
#define HW_CAN_INST         FDCAN1
#define HW_CAN_RX_PIN       GPIO_PIN_8      /* PB8  FDCAN1_RX AF9 */
#define HW_CAN_RX_PORT      GPIOB
#define HW_CAN_TX_PIN       GPIO_PIN_9      /* PB9  FDCAN1_TX AF9 */
#define HW_CAN_TX_PORT      GPIOB
#define HW_CAN_BAUDRATE     1000000

/* ==========================================================================
   8. 调试串口 — USART2（X-STAR-S 使用 USART2，PB3/PB4）
      注意：VectorFOC 使用 USART1（PB6/PB7），此处不同
   ========================================================================== */
#define HW_UART_DEBUG       huart2
#define HW_UART_DEBUG_INST  USART2
#define HW_UART_TX_PIN      GPIO_PIN_3      /* PB3 */
#define HW_UART_TX_PORT     GPIOB
#define HW_UART_RX_PIN      GPIO_PIN_4      /* PB4 */
#define HW_UART_RX_PORT     GPIOB
#define HW_UART_BAUDRATE    921600

/* ==========================================================================
   9. 状态LED
   ========================================================================== */
#define HW_STATUS_LED_PIN   GPIO_PIN_5      /* PB5 LED2 */
#define HW_STATUS_LED_PORT  GPIOB

/* ==========================================================================
   外部句柄声明（CubeMX生成）
   ========================================================================== */
extern TIM_HandleTypeDef    htim1;
extern TIM_HandleTypeDef    htim3;
extern ADC_HandleTypeDef    hadc1;
extern ADC_HandleTypeDef    hadc2;
extern FDCAN_HandleTypeDef  hfdcan1;
extern UART_HandleTypeDef   huart2;

#endif /* BOARD_CONFIG_XSTAR_H */
