/**
 * @file boot_main.c
 * @brief Bootloader 入口点
 *
 * 此文件仅用于 Bootloader 构建
 */
#include "stm32g4xx_hal.h"
#include "bootloader.h"

/* USB Device */
extern void MX_USB_Device_Init(void);

/**
 * @brief  System clock configuration (simplified for bootloader)
 *         HSE 24MHz -> PLL -> 168MHz SYSCLK
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /* Configure HSE + HSI48 + PLL */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 42;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Configure HCLK, SYSCLK, PCLK1, PCLK2 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/**
 * @brief  GPIO initialization (minimal for bootloader)
 */
static void MX_GPIO_Init(void)
{
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
}

/**
 * @brief Bootloader 主函数
 */
int main(void)
{
    /* HAL 初始化 (包括 SysTick) */
    HAL_Init();
    
    /* 配置系统时钟 */
    SystemClock_Config();
    
    /* 初始化 GPIO */
    MX_GPIO_Init();
    
    /* 初始化 USB */
    MX_USB_Device_Init();
    
    /* 进入 Bootloader 主逻辑 */
    Boot_Main();
    
    /* 不应该到达这里 */
    while (1) {
    }
}

/**
 * @brief  SysTick 中断处理 (HAL 需要)
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief  Error handler
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
