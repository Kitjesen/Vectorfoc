#ifndef TEST_APP_INIT_BINDING_MOCKS_BOARD_CONFIG_H
#define TEST_APP_INIT_BINDING_MOCKS_BOARD_CONFIG_H
#define __disable_irq() Test_DisableIrq()
#define __enable_irq() Test_EnableIrq()
#define __DMB() Test_DataMemoryBarrier()
void Test_DisableIrq(void);
void Test_EnableIrq(void);
void Test_DataMemoryBarrier(void);
void DWT_Delay(float seconds);
void Error_Handler(void);
#endif
#ifndef M_2PI
#define M_2PI 6.28318530717958647692f
#endif
