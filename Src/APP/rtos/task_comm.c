/**
 * @file task_comm.c
 * @brief 通信任务 - CAN命令处理和状态上报 (500Hz)
 */

#include "FreeRTOS.h"
#include "cmd_service.h"
#include "cmsis_os.h"
#include "manager.h"
#include "rtos_tasks.h"


__attribute__((noreturn)) void StartCustomTask(void const *argument) {
  (void)argument;
  CmdService_Init();

  for (;;) {
    Protocol_ProcessQueuedFrames();
    CmdService_Process();
    osDelay(2);
  }
}
