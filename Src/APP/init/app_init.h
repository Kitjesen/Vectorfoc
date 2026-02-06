/**
 * @file app_init.h
 * @brief 应用层统一初始化接口
 */

#ifndef APP_INIT_H
#define APP_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 应用层初始化 (在 main() 中调用)
 * @note 初始化顺序:
 *   1. BSP (DWT, Log, ADC, PWM)
 *   2. Safety & Detection
 *   3. Parameter System
 *   4. CAN & Protocol
 *   5. State Machine
 *   6. Motor
 */
void App_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_INIT_H */
