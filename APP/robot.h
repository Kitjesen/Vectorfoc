/**
 * @file robot.h
 * @brief 应用层入口 (兼容旧接口)
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "init/app_init.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用 (调用 App_Init)
 * @deprecated 使用 App_Init() 代替
 */
static inline void RobotInit(void) { App_Init(); }

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_H */
