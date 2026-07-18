#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_freertos.h"
#include "cmsis_os.h"
#include "task.h"

extern osThreadId defaultTaskHandle;
extern osThreadId guardtaskHandle;
extern osThreadId customtaskHandle;

static BaseType_t s_scheduler_state = taskSCHEDULER_NOT_STARTED;
static const osThreadDef_t *s_created_threads[3];
static uint32_t s_created_count = 0u;

BaseType_t xTaskGetSchedulerState(void) { return s_scheduler_state; }

UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask) {
  if (xTask == defaultTaskHandle) {
    return 111u;
  }
  if (xTask == guardtaskHandle) {
    return 222u;
  }
  if (xTask == customtaskHandle) {
    return 333u;
  }
  return 0u;
}

osThreadId osThreadCreate(const osThreadDef_t *thread_def, void *argument) {
  (void)argument;
  assert(s_created_count < 3u);
  s_created_threads[s_created_count++] = thread_def;
  return (osThreadId)thread_def;
}

void StartDefaultTask(void const *argument) { (void)argument; }
void StartGuardTask(void const *argument) { (void)argument; }
void StartCustomTask(void const *argument) { (void)argument; }
void Error_Handler(void) { assert(!"Error_Handler called"); }

void Emergency_DisableBridgeOutputs(void) {}
void Emergency_Shutdown(void) {}

void MX_FREERTOS_Init(void);

static void reset_task_handles(void) {
  defaultTaskHandle = NULL;
  guardtaskHandle = NULL;
  customtaskHandle = NULL;
  s_created_count = 0u;
}

static void test_null_output_is_safe(void) {
  assert(AppFreertos_GetRuntimeStats(NULL) == false);
}

static void test_scheduler_not_running_returns_cleared_unavailable_stats(void) {
  AppFreertosRuntimeStats_t stats = {
      .scheduler_running = true,
      .task_handles_ready = true,
      .default_stack_high_water_words = 1u,
      .guard_stack_high_water_words = 2u,
      .comm_stack_high_water_words = 3u,
  };

  reset_task_handles();
  s_scheduler_state = taskSCHEDULER_NOT_STARTED;

  assert(AppFreertos_GetRuntimeStats(&stats) == false);
  assert(stats.scheduler_running == false);
  assert(stats.task_handles_ready == false);
  assert(stats.default_stack_high_water_words == 0u);
  assert(stats.guard_stack_high_water_words == 0u);
  assert(stats.comm_stack_high_water_words == 0u);
}

static void test_running_scheduler_without_handles_is_unavailable(void) {
  AppFreertosRuntimeStats_t stats = {0};

  reset_task_handles();
  s_scheduler_state = taskSCHEDULER_RUNNING;

  assert(AppFreertos_GetRuntimeStats(&stats) == false);
  assert(stats.scheduler_running == true);
  assert(stats.task_handles_ready == false);
  assert(stats.default_stack_high_water_words == 0u);
  assert(stats.guard_stack_high_water_words == 0u);
  assert(stats.comm_stack_high_water_words == 0u);
}

static void test_initialized_tasks_report_high_water_marks(void) {
  AppFreertosRuntimeStats_t stats = {0};

  reset_task_handles();
  MX_FREERTOS_Init();
  s_scheduler_state = taskSCHEDULER_RUNNING;

  assert(s_created_count == 3u);
  assert(defaultTaskHandle == (osThreadId)s_created_threads[0]);
  assert(guardtaskHandle == (osThreadId)s_created_threads[1]);
  assert(customtaskHandle == (osThreadId)s_created_threads[2]);

  assert(AppFreertos_GetRuntimeStats(&stats) == true);
  assert(stats.scheduler_running == true);
  assert(stats.task_handles_ready == true);
  assert(stats.default_stack_high_water_words == 111u);
  assert(stats.guard_stack_high_water_words == 222u);
  assert(stats.comm_stack_high_water_words == 333u);
}

int main(void) {
  test_null_output_is_safe();
  test_scheduler_not_running_returns_cleared_unavailable_stats();
  test_running_scheduler_without_handles_is_unavailable();
  test_initialized_tasks_report_high_water_marks();
  return 0;
}
