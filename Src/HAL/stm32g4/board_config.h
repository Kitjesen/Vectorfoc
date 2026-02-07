/**
 * @file    board_config.h
 * @brief   Centralized board hardware configuration
 * @note    All pin assignments, peripheral mappings, and board-specific constants
 *          are defined here. Modify THIS FILE when porting to a new board.
 *
 * === How to port to a new board ===
 * 1. Copy this file as board_config_<your_board>.h
 * 2. Update pin/peripheral assignments
 * 3. Update CubeMX .ioc accordingly
 * 4. Include your board config via build system or rename
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32g4xx_hal.h"

/* ==========================================================================
   Board Identification
   ========================================================================== */
#define BOARD_NAME              "VectorFOC_G431"
#define BOARD_MCU               "STM32G431CBU6"
#define BOARD_REVISION          "Rev1.0"

/* ==========================================================================
   1. Clock Configuration
   ========================================================================== */
#define SYS_CLOCK_MHZ           168
#define SYS_CLOCK_HZ            (SYS_CLOCK_MHZ * 1000000UL)

/* ==========================================================================
   2. Motor PWM - TIM1 (Advanced Timer, center-aligned)
   ========================================================================== */
#define HW_PWM_TIMER            htim1           // Timer handle
#define HW_PWM_TIM_INSTANCE     TIM1            // Timer peripheral
#define HW_PWM_FREQ_HZ         20000           // PWM frequency [Hz]
#define HW_PWM_DEADTIME_CLKS   20              // Dead time [clock cycles]

/* PWM Channel Assignment (TIM1) */
#define HW_PWM_CH_U            TIM_CHANNEL_1   // Phase U (PA8)
#define HW_PWM_CH_V            TIM_CHANNEL_2   // Phase V (PA9)
#define HW_PWM_CH_W            TIM_CHANNEL_3   // Phase W (PA10)
#define HW_PWM_CH_TRIG         TIM_CHANNEL_4   // ADC trigger

/* PWM GPIO Pins */
#define HW_PWM_U_H_PIN         GPIO_PIN_8      // PA8  - U high side
#define HW_PWM_U_H_PORT        GPIOA
#define HW_PWM_V_H_PIN         GPIO_PIN_9      // PA9  - V high side
#define HW_PWM_V_H_PORT        GPIOA
#define HW_PWM_W_H_PIN         GPIO_PIN_10     // PA10 - W high side
#define HW_PWM_W_H_PORT        GPIOA
#define HW_PWM_U_L_PIN         GPIO_PIN_13     // PB13 - U low side
#define HW_PWM_U_L_PORT        GPIOB
#define HW_PWM_V_L_PIN         GPIO_PIN_14     // PB14 - V low side
#define HW_PWM_V_L_PORT        GPIOB
#define HW_PWM_W_L_PIN         GPIO_PIN_15     // PB15 - W low side
#define HW_PWM_W_L_PORT        GPIOB

/* ==========================================================================
   3. Current Sensing - ADC1 (Injected mode, TIM1_CC4 triggered)
   ========================================================================== */
#define HW_ADC_CURRENT          hadc1           // ADC handle
#define HW_ADC_CURRENT_INST     ADC1            // ADC peripheral

/* ADC Channel -> Phase Mapping
 * IMPORTANT: JDR register order is set by CubeMX injected rank config.
 *   JDR1 = Rank 1, JDR2 = Rank 2, etc.
 *   Verify this matches your CubeMX ADC injected sequence!
 */
#define HW_ADC_JDR_IC          JDR1            // Phase C -> JDR1 (PA0, IN1)
#define HW_ADC_JDR_IB          JDR2            // Phase B -> JDR2 (PA1, IN2)
#define HW_ADC_JDR_IA          JDR3            // Phase A -> JDR3 (PA2, IN3)
#define HW_ADC_JDR_VBUS        JDR4            // VBUS    -> JDR4 (PA3, IN4)

/* Current Sensor GPIO */
#define HW_ADC_IA_PIN          GPIO_PIN_2      // PA2 - Phase A
#define HW_ADC_IA_PORT         GPIOA
#define HW_ADC_IA_CH           ADC_CHANNEL_3
#define HW_ADC_IB_PIN          GPIO_PIN_1      // PA1 - Phase B
#define HW_ADC_IB_PORT         GPIOA
#define HW_ADC_IB_CH           ADC_CHANNEL_2
#define HW_ADC_IC_PIN          GPIO_PIN_0      // PA0 - Phase C
#define HW_ADC_IC_PORT         GPIOA
#define HW_ADC_IC_CH           ADC_CHANNEL_1
#define HW_ADC_VBUS_PIN        GPIO_PIN_3      // PA3 - Bus voltage/current
#define HW_ADC_VBUS_PORT       GPIOA
#define HW_ADC_VBUS_CH         ADC_CHANNEL_4

/* Current Sensing Analog Front End */
#define HW_SHUNT_RESISTANCE    0.02f           // Shunt resistor [Ohm]
#define HW_OPAMP_GAIN          50.0f           // Op-amp gain [V/V]
#define HW_ADC_VREF            3.3f            // ADC reference voltage [V]
#define HW_ADC_RESOLUTION      4095            // 12-bit ADC max value
#define HW_ADC_MIDPOINT        1.65f           // Vref/2 midpoint [V]

/* ==========================================================================
   4. Temperature Sensing - ADC2 (DMA mode)
   ========================================================================== */
#define HW_ADC_TEMP            hadc2           // ADC handle
#define HW_ADC_TEMP_INST       ADC2            // ADC peripheral
#define HW_TEMP_PIN            GPIO_PIN_2      // PB2
#define HW_TEMP_PORT           GPIOB
#define HW_TEMP_CH             ADC_CHANNEL_12  // ADC2_IN12

/* NTC Thermistor */
#define HW_NTC_R25             10000.0f        // NTC resistance at 25°C [Ohm]
#define HW_NTC_B_VALUE         3950            // NTC B-value
#define HW_NTC_PULLUP          10000.0f        // Pull-up resistor [Ohm]

/* ==========================================================================
   5. Bus Voltage Sensing
   ========================================================================== */
#define HW_VBUS_R_HIGH         10000.0f        // Upper divider resistor [Ohm]
#define HW_VBUS_R_LOW          1000.0f         // Lower divider resistor [Ohm]
#define HW_VBUS_DIVIDER_RATIO  ((HW_VBUS_R_HIGH + HW_VBUS_R_LOW) / HW_VBUS_R_LOW)

/* ==========================================================================
   6. Encoder - SPI3 (MT6816)
   ========================================================================== */
#define HW_ENC_SPI             hspi3           // SPI handle
#define HW_ENC_SPI_INST        SPI3            // SPI peripheral
#define HW_ENC_CS_PIN          GPIO_PIN_12     // PB12 - Chip select
#define HW_ENC_CS_PORT         GPIOB
#define HW_ENC_SCK_PIN         GPIO_PIN_3      // PB3
#define HW_ENC_MISO_PIN        GPIO_PIN_4      // PB4
#define HW_ENC_MOSI_PIN        GPIO_PIN_5      // PB5
#define HW_ENC_CPR             16384           // Counts per revolution (14-bit)

/* ==========================================================================
   7. Pre-Driver - SPI1 (DRV8323 or similar)
   ========================================================================== */
#define HW_DRV_SPI             hspi1           // SPI handle
#define HW_DRV_SPI_INST        SPI1            // SPI peripheral
#define HW_DRV_CS_PIN          GPIO_PIN_4      // PC4 - Chip select
#define HW_DRV_CS_PORT         GPIOC
#define HW_DRV_SCK_PIN         GPIO_PIN_5      // PA5
#define HW_DRV_MISO_PIN        GPIO_PIN_6      // PA6
#define HW_DRV_MOSI_PIN        GPIO_PIN_7      // PA7

/* ==========================================================================
   8. CAN Bus - FDCAN1
   ========================================================================== */
#define HW_CAN                 hfdcan1         // CAN handle
#define HW_CAN_INST            FDCAN1          // CAN peripheral
#define HW_CAN_RX_PIN          GPIO_PIN_8      // PB8
#define HW_CAN_RX_PORT         GPIOB
#define HW_CAN_TX_PIN          GPIO_PIN_9      // PB9
#define HW_CAN_TX_PORT         GPIOB
#define HW_CAN_BAUDRATE        1000000         // Default 1Mbps

/* ==========================================================================
   9. Debug UART - USART1
   ========================================================================== */
#define HW_UART_DEBUG          huart1          // UART handle
#define HW_UART_DEBUG_INST     USART1          // UART peripheral
#define HW_UART_TX_PIN         GPIO_PIN_6      // PB6
#define HW_UART_TX_PORT        GPIOB
#define HW_UART_RX_PIN         GPIO_PIN_7      // PB7
#define HW_UART_RX_PORT        GPIOB
#define HW_UART_BAUDRATE       921600          // Debug baud rate

/* ==========================================================================
   10. USB - CDC Virtual COM
   ========================================================================== */
#define HW_USB_DM_PIN          GPIO_PIN_11     // PA11 - USB D-
#define HW_USB_DP_PIN          GPIO_PIN_12     // PA12 - USB D+
#define HW_USB_PORT            GPIOA

/* ==========================================================================
   11. RGB LED - WS2812 via TIM3_CH2 (DMA)
   ========================================================================== */
#define HW_LED_TIMER           htim3           // Timer handle
#define HW_LED_TIM_INSTANCE    TIM3            // Timer peripheral
#define HW_LED_CHANNEL         TIM_CHANNEL_2   // PA4
#define HW_LED_PIN             GPIO_PIN_4      // PA4
#define HW_LED_PORT            GPIOA

/* ==========================================================================
   12. Status LED - GPIO
   ========================================================================== */
#define HW_STATUS_LED_PIN      GPIO_PIN_13     // PC13
#define HW_STATUS_LED_PORT     GPIOC

/* ==========================================================================
   External handle declarations (defined in CubeMX-generated code)
   ========================================================================== */
extern TIM_HandleTypeDef       htim1;
extern TIM_HandleTypeDef       htim3;
extern ADC_HandleTypeDef       hadc1;
extern ADC_HandleTypeDef       hadc2;
extern SPI_HandleTypeDef       hspi1;
extern SPI_HandleTypeDef       hspi3;
extern FDCAN_HandleTypeDef     hfdcan1;
extern UART_HandleTypeDef      huart1;

#endif /* BOARD_CONFIG_H */
