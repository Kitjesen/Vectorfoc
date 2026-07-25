/**
 * @file app_freertos.h
 * @brief Runtime FreeRTOS diagnostics exported by app_freertos.c.
 */
#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool scheduler_running;
  bool task_handles_ready;
  uint32_t default_stack_high_water_words;
  uint32_t guard_stack_high_water_words;
  uint32_t comm_stack_high_water_words;
} AppFreertosRuntimeStats_t;

/**
 * @brief Capture task stack high-water marks for the core runtime tasks.
 *
 * Call from task/thread diagnostic context only; this API is not ISR-facing.
 * When the scheduler is not running, or any task handle is unavailable, all
 * high-water fields remain zero and the return value is false.
 *
 * @param stats Output diagnostics. Fields are cleared before sampling.
 * @return true when the scheduler is running and all task handles were ready.
 */
bool AppFreertos_GetRuntimeStats(AppFreertosRuntimeStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* APP_FREERTOS_H */
