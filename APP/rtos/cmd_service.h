/**
 * @file cmd_service.h
 * @brief 命令服务接口
 */

#ifndef CMD_SERVICE_H
#define CMD_SERVICE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void CmdService_Init(void);                   // 命令服务初始化
void CmdService_Process(void);                // 命令服务处理
void CmdService_SetReportEnable(bool enable); // 设置是否上报状态

#ifdef __cplusplus
}
#endif

#endif /* CMD_SERVICE_H */
