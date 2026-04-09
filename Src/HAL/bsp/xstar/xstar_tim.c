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
 * @file xstar_tim.c
 * @brief X-STAR-S 定时器初始化
 *
 *  TIM1：中心对齐 PWM，20kHz，CH1-3 三相输出 + CH4 ADC 触发
 *         170MHz / (2×4250) = 20kHz
 *         引脚：PA8(CH1), PA9(CH2), PA10(CH3) — 上桥 AF6
 *               PB13(CH1N) AF6, PA12(CH2N) AF6, PB15(CH3N) AF4 — 下桥
 *
 *  TIM3：Hall 传感器模式，PC6(CH1)/PC7(CH2)/PC8(CH3) — AF2
 *         TI1 = XOR(CH1, CH2, CH3)，每次霍尔跳变触发 CC1 中断
 */
#ifdef BOARD_XSTAR

#include "xstar_bsp.h"
#include "board_config_xstar.h"
#include "hall_encoder.h"
#include "main.h"

/* TIM1 ARR for 20kHz center-aligned: 170MHz / 2 / 20000 = 4250 */
#define XSTAR_TIM1_ARR       4250U
/* ADC 触发脉冲：靠近计数峰值以对齐电流采样（ARR - 10） */
#define XSTAR_TIM1_TRIG_PULSE (XSTAR_TIM1_ARR - 10U)

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle);

/* ==========================================================================
   TIM1 — 三相中心对齐 PWM（20kHz，死区20周期≈118ns@170MHz）
   ========================================================================== */
void XStar_TIM1_Init(void) {
    TIM_MasterConfigTypeDef        sMasterConfig       = {0};
    TIM_OC_InitTypeDef             sConfigOC           = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period            = XSTAR_TIM1_ARR;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;   /* 每个 PWM 周期产生一次更新事件 */
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

    /* CH4 的 OC4REF 作为 ADC 注入组触发源（TRGO） */
    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_OC4REF;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* CH1~3：三相 PWM，初始占空比为 0 */
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, HW_PWM_CH_U) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, HW_PWM_CH_V) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, HW_PWM_CH_W) != HAL_OK) {
        Error_Handler();
    }

    /* CH4：ADC 触发脉冲，靠近峰值（电流采样在 PWM 中点电流纹波最小） */
    sConfigOC.Pulse = XSTAR_TIM1_TRIG_PULSE;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, HW_PWM_CH_TRIG) != HAL_OK) {
        Error_Handler();
    }

    /* 死区与刹车配置 */
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = HW_PWM_DEADTIME_CLKS; /* 20 clks ≈ 118ns */
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }

    /* 配置 GPIO（通过 HAL_TIM_MspPostInit） */
    HAL_TIM_MspPostInit(&htim1);
}

/* ==========================================================================
   TIM3 — Hall 传感器模式
      TI1 = XOR(CH1, CH2, CH3)，任意跳变触发 CC1 中断 → Hall_UpdateFromISR()
      计数器自由运行（0xFFFF），可用于计算两次跳变间的时间（速度估算）
   ========================================================================== */
void XStar_TIM3_HallSensor_Init(void) {
    TIM_HallSensor_InitTypeDef sConfig = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;          /* 170MHz 计数时钟，分辨率约5.9ns */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 0xFFFF;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.IC1Polarity    = TIM_ICPOLARITY_BOTHEDGE; /* 上升沿+下降沿均触发 */
    sConfig.IC1Prescaler   = TIM_ICPSC_DIV1;
    sConfig.IC1Filter      = 0x08;                    /* 滤波防抖：8个时钟周期 */
    sConfig.Commutation_Delay = 0;

    if (HAL_TIMEx_HallSensor_Init(&htim3, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

void XStar_TIM3_Encoder_Init(void) {
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = HW_ABZ_CPR - 1u;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode          = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC1Filter            = 4;
    sConfig.IC2Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC2Filter            = 4;

    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

void XStar_TIM3_Sensor_Init(void) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    XStar_TIM3_HallSensor_Init();
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
    XStar_TIM3_Encoder_Init();
#else
#error "Unsupported X-STAR position sensor mode"
#endif
}

/* ==========================================================================
   HAL MspInit 回调：TIM1 PWM（GPIO + 时钟）
   ========================================================================== */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle) {
    if (tim_pwmHandle->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();

        /* TIM1 CC IRQ 用于 FOC（优先级最高） */
        HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
        HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 15, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    }
}

/* ==========================================================================
   HAL MspPostInit 回调：TIM1 GPIO（PWM 引脚 AF 配置）
   X-STAR-S 与 VectorFOC 的差异：V 下桥为 PA12（AF6），原版为 PB14（AF4）
   ========================================================================== */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (timHandle->Instance == TIM1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* 上桥：PA8(CH1), PA9(CH2), PA10(CH3) — AF6 */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* 下桥 V：PA12(CH2N) — AF6（X-STAR-S 特有，非 PB14！） */
        GPIO_InitStruct.Pin       = GPIO_PIN_12;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* 下桥 U：PB13(CH1N) — AF6 */
        GPIO_InitStruct.Pin       = GPIO_PIN_13;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* 下桥 W：PB15(CH3N) — AF4（STM32G431 上 PB15=TIM1_CH3N 对应 AF4，非 AF6）*/
        GPIO_InitStruct.Pin       = GPIO_PIN_15;
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle) {
    if (tim_pwmHandle->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_15);
        HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
        HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    }
}

/* ==========================================================================
   HAL MspInit 回调：TIM3 Hall 传感器模式
   HAL_TIMEx_HallSensor_Init() 内部调用 HAL_TIMEx_HallSensor_MspInit()
   ========================================================================== */
void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* PC6(TIM3_CH1=HA), PC7(TIM3_CH2=HB), PC8(TIM3_CH3=HC) — AF2 */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;  /* 上拉，防止悬空 */
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* TIM3 全局中断（CC1 用于霍尔跳变检测） */
        HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
#else
    (void)htim;
#endif
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim) {
    (void)htim; /* Not used for Hall sensor — see HAL_TIMEx_HallSensor_MspInit */
}

void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8);
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
    }
#else
    (void)htim;
#endif
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim) {
    (void)htim;
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
    if (htim->Instance == TIM3) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_InitStruct.Pin       = HW_ABZ_A_PIN | HW_ABZ_B_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(HW_ABZ_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = HW_ABZ_Z_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = 0;
        HAL_GPIO_Init(HW_ABZ_Z_PORT, &GPIO_InitStruct);
    }
#else
    (void)htim;
#endif
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
    if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_DISABLE();
        HAL_GPIO_DeInit(HW_ABZ_PORT, HW_ABZ_A_PIN | HW_ABZ_B_PIN);
        HAL_GPIO_DeInit(HW_ABZ_Z_PORT, HW_ABZ_Z_PIN);
    }
#else
    (void)htim;
#endif
}

/* ==========================================================================
   HAL_TIM_IC_CaptureCallback：霍尔跳变处理
   由 TIM3_IRQHandler → HAL_TIM_IRQHandler → 此回调 调用
   ========================================================================== */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    if (htim->Instance == TIM3) {
        Hall_UpdateFromISR();
    }
#else
    (void)htim;
#endif
}

#endif /* BOARD_XSTAR */
