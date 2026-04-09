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
 * @file xstar_bsp.c
 * @brief X-STAR-S STM32G431RBT6 BSP 硬件初始化实现
 *
 * 包含：
 *   - 系统时钟：8MHz HSE → PLL → 170MHz
 *   - GPIO：LED、ADC 模拟引脚
 *   - OPAMP1/2/3：Standalone 模式（外部增益10x）
 *   - ADC1：注入组4通道（Iu, Iw, Vbus, Temp），TIM1_CC4 触发
 *   - ADC2：注入组1通道（Iv），TIM1_CC4 触发
 *   - FDCAN1：1Mbps，PB8(RX)/PB9(TX)
 *   - USART2：921600 baud，PB3(TX)/PB4(RX)
 *   - 所有 HAL MspInit/MspDeInit 回调（仅在 BOARD_XSTAR 时编译）
 */
#ifdef BOARD_XSTAR

#include "xstar_bsp.h"
#include "main.h"
#include <math.h>

/* ==========================================================================
   外设句柄定义（htim1/3/hadc1/2/hfdcan1 已在 Lib/Core/Src 中定义）
   OPAMP 使用直接寄存器访问，不依赖 HAL OPAMP 模块
   ========================================================================== */
UART_HandleTypeDef   huart2;

/* ADC12 时钟使能引用计数（ADC1 和 ADC2 共享同一个时钟） */
static uint32_t s_adc12_clk_enabled = 0;

/* ==========================================================================
   1. 系统时钟配置
      8MHz HSE → PLLM=2(→4MHz) → PLLN=85(→340MHz VCO) → PLLR=2 → 170MHz
      Flash 延迟：Boost 模式 4WS（136~170MHz 范围）
   ========================================================================== */
void XStar_SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Boost 模式（1.28V）：支持 170MHz */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /* 配置 HSE + PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV2;   /* 8MHz/2 = 4MHz  */
    RCC_OscInitStruct.PLL.PLLN       = 85;               /* 4×85 = 340MHz VCO */
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;   /* 不使用 */
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;   /* 不使用 */
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;   /* 340/2 = 170MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* 配置总线分频：HCLK=PCLK1=PCLK2=170MHz */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    /* 170MHz 需要 4 个 Flash 等待周期（Boost 模式，136~170MHz） */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* ==========================================================================
   2. GPIO 初始化
      - PB5: LED（推挽输出）
      - 模拟引脚（ADC 输入）：PA2, PA6, PB1, PB11, PB12
      注意：PWM、Hall、CAN、UART 的 GPIO 在各外设 MspInit 中配置
   ========================================================================== */
void XStar_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* LED2: PB5 默认低电平（熄灭） */
    HAL_GPIO_WritePin(HW_STATUS_LED_PORT, HW_STATUS_LED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = HW_STATUS_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HW_STATUS_LED_PORT, &GPIO_InitStruct);

    /* ADC 模拟输入引脚（配置为 Analog，无需 AF） */
    /* GPIOA: PA2(OPAMP1_VOUT→ADC1_IN3 Iu), PA6(OPAMP2_VOUT→ADC2_IN3 Iv) */
    GPIO_InitStruct.Pin  = GPIO_PIN_2 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* GPIOB: PB1(OPAMP3_VOUT→ADC1_IN12 Iw), PB11(ADC1_IN14 Vbus),
     *        PB12(ADC1_IN11 Temp) */
    GPIO_InitStruct.Pin  = GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ==========================================================================
   3. OPAMP 初始化（直接寄存器，不依赖 HAL OPAMP 模块）
      Standalone 模式：外部增益网络 R_in=1kΩ / R_fb=10kΩ → 增益10x
      引脚（对照原理图确认，以下为 STM32G431 典型接法）：
        OPAMP1: VINP=PA1(IO0), VINM=PA3(IO0), VOUT=PA2
        OPAMP2: VINP=PA7(IO0), VINM=PA5(IO0), VOUT=PA6
        OPAMP3: VINP=PB0(IO0), VINM=PB2(IO0), VOUT=PB1
      CSR 寄存器：VPSEL=0(IO0), VMSEL=0b00(IO0), OPAMPxEN=1
   ========================================================================== */
void XStar_OPAMP_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 配置 OPAMP 输入引脚为模拟模式 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA1(OPAMP1_VINP), PA3(OPAMP1_VINM)
     * PA5(OPAMP2_VINM), PA7(OPAMP2_VINP) */
    GPIO_InitStruct.Pin  = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB0(OPAMP3_VINP), PB2(OPAMP3_VINM) */
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 启用 OPAMPs：Standalone 模式（VMSEL=00=IO0, VPSEL=00=IO0），仅置 EN 位
     * VOUT 引脚（PA2, PA6, PB1）由 ADC MspInit 配置为 Analog */
    OPAMP1->CSR = OPAMP_CSR_OPAMPxEN;
    OPAMP2->CSR = OPAMP_CSR_OPAMPxEN;
    OPAMP3->CSR = OPAMP_CSR_OPAMPxEN;
}

/* ==========================================================================
   4. ADC1 初始化
      注入组 4 通道，TIM1_CC4 上升沿触发，ADC 时钟 = PLL/4 ≈ 42.5MHz
        Rank1: ADC1_IN3  (PA2  OPAMP1_VOUT) → Iu
        Rank2: ADC1_IN12 (PB1  OPAMP3_VOUT) → Iw
        Rank3: ADC1_IN14 (PB11)             → Vbus
        Rank4: ADC1_IN11 (PB12)             → Temp
   ========================================================================== */
void XStar_ADC1_Init(void) {
    ADC_MultiModeTypeDef       multimode      = {0};
    ADC_InjectionConfTypeDef   sConfigInjected = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    /* 通用注入配置 */
    sConfigInjected.InjectedSamplingTime        = ADC_SAMPLETIME_6CYCLES_5;
    sConfigInjected.InjectedSingleDiff          = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber        = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset              = 0;
    sConfigInjected.InjectedNbrOfConversion     = 4;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv            = DISABLE;
    sConfigInjected.QueueInjectedContext        = DISABLE;
    sConfigInjected.ExternalTrigInjecConv       = ADC_EXTERNALTRIGINJEC_T1_CC4;
    sConfigInjected.ExternalTrigInjecConvEdge   = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjected.InjecOversamplingMode       = DISABLE;

    /* Rank1: ADC1_IN3 → Iu（OPAMP1_VOUT / PA2） */
    sConfigInjected.InjectedChannel = HW_ADC1_CH_IU;
    sConfigInjected.InjectedRank    = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }

    /* Rank2: ADC1_IN12 → Iw（OPAMP3_VOUT / PB1） */
    sConfigInjected.InjectedChannel = HW_ADC1_CH_IW;
    sConfigInjected.InjectedRank    = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }

    /* Rank3: ADC1_IN14 → Vbus（PB11） */
    sConfigInjected.InjectedChannel = HW_ADC1_CH_VBUS;
    sConfigInjected.InjectedRank    = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }

    /* Rank4: ADC1_IN11 → Temp（PB12 NTC） */
    sConfigInjected.InjectedChannel = HW_ADC1_CH_TEMP;
    sConfigInjected.InjectedRank    = ADC_INJECTED_RANK_4;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }
}

/* ==========================================================================
   5. ADC2 初始化
      注入组 1 通道，TIM1_CC4 触发（与 ADC1 同步）
        Rank1: ADC2_IN3 (PA6 OPAMP2_VOUT) → Iv
   ========================================================================== */
void XStar_ADC2_Init(void) {
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    hadc2.Instance                   = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation      = 0;
    hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait      = DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.NbrOfConversion       = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) { Error_Handler(); }

    /* Rank1: ADC2_IN3 → Iv（OPAMP2_VOUT / PA6） */
    sConfigInjected.InjectedChannel            = HW_ADC2_CH_IV;
    sConfigInjected.InjectedRank               = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime       = ADC_SAMPLETIME_6CYCLES_5;
    sConfigInjected.InjectedSingleDiff         = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber       = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset             = 0;
    sConfigInjected.InjectedNbrOfConversion    = 1;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv           = DISABLE;
    sConfigInjected.QueueInjectedContext       = DISABLE;
    sConfigInjected.ExternalTrigInjecConv      = ADC_EXTERNALTRIGINJEC_T1_CC4;
    sConfigInjected.ExternalTrigInjecConvEdge  = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjected.InjecOversamplingMode      = DISABLE;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }
}

/* ==========================================================================
   6. FDCAN1 初始化
      1Mbps Classic CAN，PB8(RX)/PB9(TX)
      位时序：170MHz / 10 / (1+13+3) = 1Mbps
   ========================================================================== */
void XStar_FDCAN1_Init(void) {
    hfdcan1.Instance                = FDCAN1;
    hfdcan1.Init.ClockDivider       = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat        = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode               = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause      = DISABLE;
    hfdcan1.Init.ProtocolException  = ENABLE;
    /* 标称段：170MHz/10/17 = 1Mbps */
    hfdcan1.Init.NominalPrescaler      = 10;
    hfdcan1.Init.NominalSyncJumpWidth  = 3;
    hfdcan1.Init.NominalTimeSeg1       = 13;
    hfdcan1.Init.NominalTimeSeg2       = 3;
    /* 数据段（Classic 模式下同标称段即可） */
    hfdcan1.Init.DataPrescaler         = 10;
    hfdcan1.Init.DataSyncJumpWidth     = 3;
    hfdcan1.Init.DataTimeSeg1          = 13;
    hfdcan1.Init.DataTimeSeg2          = 3;
    hfdcan1.Init.StdFiltersNbr         = 0;
    hfdcan1.Init.ExtFiltersNbr         = 1;  /* Protocol uses 29-bit extended IDs */
    hfdcan1.Init.TxFifoQueueMode       = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) { Error_Handler(); }
}

/* ==========================================================================
   7. USART2 初始化
      921600 baud，8N1，PB3(TX)/PB4(RX)，中断模式（用于调试日志）
   ========================================================================== */
void XStar_USART2_Init(void) {
    huart2.Instance            = USART2;
    huart2.Init.BaudRate       = HW_UART_BAUDRATE;
    huart2.Init.WordLength     = UART_WORDLENGTH_8B;
    huart2.Init.StopBits       = UART_STOPBITS_1;
    huart2.Init.Parity         = UART_PARITY_NONE;
    huart2.Init.Mode           = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) { Error_Handler(); }
}

/* ==========================================================================
   8. 主初始化入口（供 main.c 调用）
   ========================================================================== */
void XStar_BSP_Init(void) {
    XStar_SystemClock_Config();
    XStar_GPIO_Init();
    XStar_OPAMP_Init();
    XStar_ADC1_Init();
    XStar_ADC2_Init();
    XStar_FDCAN1_Init();
    XStar_USART2_Init();
    /* TIM1 和 TIM3 在 xstar_tim.c 中，由 XStar_BSP_Init 一并调用 */
    XStar_TIM1_Init();
    XStar_TIM3_Sensor_Init();
}

/* ==========================================================================
   HAL MspInit / MspDeInit 回调（仅在 BOARD_XSTAR 下编译，替代原版实现）
   ========================================================================== */

/* ---- ADC MspInit/MspDeInit ---- */
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {
    GPIO_InitTypeDef         GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit  = {0};

    /* ADC12 共享同一个 PLL 时钟源，用引用计数避免重复初始化 */
    PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection   = RCC_ADC12CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
    s_adc12_clk_enabled++;
    if (s_adc12_clk_enabled == 1) {
        __HAL_RCC_ADC12_CLK_ENABLE();
    }

    if (adcHandle->Instance == ADC1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PA2: OPAMP1_VOUT (ADC1_IN3, Iu) */
        GPIO_InitStruct.Pin  = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PB1: OPAMP3_VOUT (ADC1_IN12, Iw)
         * PB11: Vbus       (ADC1_IN14)
         * PB12: NTC Temp   (ADC1_IN11) */
        GPIO_InitStruct.Pin  = GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    } else if (adcHandle->Instance == ADC2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA6: OPAMP2_VOUT (ADC2_IN3, Iv) */
        GPIO_InitStruct.Pin  = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC2 不需要独立中断，由 ADC1 ISR 同步读取 JDR */
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle) {
    s_adc12_clk_enabled--;
    if (s_adc12_clk_enabled == 0) {
        __HAL_RCC_ADC12_CLK_DISABLE();
    }
    if (adcHandle->Instance == ADC1) {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    } else if (adcHandle->Instance == ADC2) {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    }
}

/* ---- FDCAN MspInit/MspDeInit ---- */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle) {
    GPIO_InitTypeDef         GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit  = {0};

    if (fdcanHandle->Instance == FDCAN1) {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB8=FDCAN1_RX (AF9), PB9=FDCAN1_TX (AF9) */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle) {
    if (fdcanHandle->Instance == FDCAN1) {
        __HAL_RCC_FDCAN_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
    }
}

/* ---- UART MspInit/MspDeInit（USART2 on PB3/PB4） ---- */
void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle) {
    GPIO_InitTypeDef         GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit  = {0};

    if (uartHandle->Instance == USART2) {
        PeriphClkInit.PeriphClockSelection   = RCC_PERIPHCLK_USART2;
        PeriphClkInit.Usart2ClockSelection   = RCC_USART2CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB3=USART2_TX (AF7), PB4=USART2_RX (AF7) */
        GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_4;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle) {
    if (uartHandle->Instance == USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
}

#endif /* BOARD_XSTAR */
