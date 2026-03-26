/**
 * @file xstar_bsp.h
 * @brief X-STAR-S STM32G431RBT6 BSP 初始化接口声明
 *
 * 用法：在 main.c 中 #ifdef BOARD_XSTAR 时调用 XStar_BSP_Init()
 *       代替 CubeMX 生成的 MX_*_Init() 系列函数。
 */
#ifndef XSTAR_BSP_H
#define XSTAR_BSP_H

#ifdef BOARD_XSTAR

#include "board_config_xstar.h"

/* ------------------------------------------------------------------
   外设句柄声明（定义于 xstar_bsp.c / xstar_tim.c）
   htim1, htim3, hadc1, hadc2, hfdcan1 已在原 Lib/Core/Src 文件中定义
   OPAMP 使用直接寄存器访问，无需 HAL OPAMP 模块
   ------------------------------------------------------------------ */
extern UART_HandleTypeDef   huart2;

/* ------------------------------------------------------------------
   主初始化入口（替代所有 MX_*_Init 调用）
   ------------------------------------------------------------------ */
void XStar_BSP_Init(void);

/* ------------------------------------------------------------------
   分项初始化（如需单独调用）
   ------------------------------------------------------------------ */
void XStar_SystemClock_Config(void);
void XStar_GPIO_Init(void);
void XStar_OPAMP_Init(void);
void XStar_ADC1_Init(void);
void XStar_ADC2_Init(void);
void XStar_FDCAN1_Init(void);
void XStar_USART2_Init(void);
void XStar_TIM1_Init(void);
void XStar_TIM3_HallSensor_Init(void);
void XStar_TIM3_Encoder_Init(void);
void XStar_TIM3_Sensor_Init(void);

#endif /* BOARD_XSTAR */
#endif /* XSTAR_BSP_H */
