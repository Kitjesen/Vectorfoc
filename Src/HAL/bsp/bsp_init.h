#ifndef BSP_INIT_H
#define BSP_INIT_H
#include "bsp_dwt.h"
/**
 * @brief bspinit,initbsp,init
 *        start,RobotoInit()
 *
 * @note CANinit,init
 */
/* 根据实际时钟初始化 DWT，支持 168MHz（VectorFOC）和 170MHz（X-STAR-S） */
void BSPInit()
{
  DWT_Init(SystemCoreClock / 1000000UL);
}
#endif
