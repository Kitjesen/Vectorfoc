/**
 * @file cmd_service.h
 * @brief
 */
#ifndef CMD_SERVICE_H
#define CMD_SERVICE_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void CmdService_Init(void);                   // init
void CmdService_Process(void);                //
void CmdService_SetReportEnable(bool enable); // setstate
#ifdef __cplusplus
}
#endif
#endif /* CMD_SERVICE_H */
