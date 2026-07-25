#include "app_init.h"
#include "board_config.h"
#include "bsp_log.h"
#include "error_types.h"
#include "fault_def.h"
#include "motor.h"
#include "param_table.h"

#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);              \
      return 1;                                                                \
    }                                                                          \
  } while (0)

typedef enum {
  EVENT_NONE = 0,
  EVENT_BSP_INIT,
  EVENT_ERROR_MANAGER_INIT,
  EVENT_ADC_BIND,
  EVENT_PWM_BIND,
  EVENT_PARAM_BINDINGS_INSTALL,
  EVENT_ENCODER_INIT,
  EVENT_PARAM_SYSTEM_INIT,
  EVENT_ADC_BSP_INIT,
  EVENT_PWM_START_SAMPLING,
  EVENT_ADC_CALIBRATE_CURRENT,
  EVENT_PWM_DISABLE,
  EVENT_ENCODER_OFFSET_APPLY,
  EVENT_DETECTION_SLOW,
  EVENT_APP_COMM_BOOTSTRAP,
  EVENT_INIT_MOTOR_NO_CALIB,
  EVENT_RUNTIME_SETTINGS_INSTALL,
  EVENT_PARAM_APPLY_RUNTIME,
  EVENT_DMB,
  EVENT_ERROR_HANDLER,
} TestEvent;

static TestEvent s_events[64];
static uint32_t s_event_count;
static jmp_buf s_error_handler_jump;
static uint32_t s_last_error_code;
static const char *s_last_error_message;
static bool s_error_handler_armed;
static bool s_runtime_ready_at_dmb;
static bool s_runtime_ready_at_motor_init;
static bool s_runtime_ready_at_runtime_apply;

static int s_adc_bind_result;
static int s_pwm_bind_result;
static ParamResult s_parameter_bindings_result;
static int s_encoder_init_result;
static bool s_adc_bsp_init_result;
static int s_pwm_start_sampling_result;
static int s_adc_calibrate_result;
static int s_encoder_update_result;
static int s_encoder_offset_result;
static uint32_t s_detection_slow_result;
static bool s_app_comm_bootstrap_result;
static bool s_safety_has_active_fault;
static uint8_t s_precalib_pass_mask;
static const Motor_HAL_AdcInterface_t *s_bound_adc;
static const Motor_HAL_PwmInterface_t *s_bound_pwm;

static void Record(TestEvent event) {
  if (s_event_count < sizeof(s_events) / sizeof(s_events[0])) {
    s_events[s_event_count++] = event;
  }
}

static bool EventWasRecorded(TestEvent event) {
  for (uint32_t i = 0u; i < s_event_count; ++i) {
    if (s_events[i] == event) {
      return true;
    }
  }
  return false;
}

static uint32_t EventIndex(TestEvent event) {
  for (uint32_t i = 0u; i < s_event_count; ++i) {
    if (s_events[i] == event) {
      return i;
    }
  }
  return UINT32_MAX;
}

MOTOR_DATA motor_data;
StateMachine g_ds402_state_machine;
UART_HandleTypeDef HW_UART_DEBUG;

static void FakeAdcUpdate(Motor_HAL_SensorData_t *data) {
  data->i_a = 1.0f;
  data->i_b = 2.0f;
  data->i_c = 3.0f;
  data->v_bus = 24.0f;
  data->temp = 35.0f;
}

static void FakeEncoderGetData(Motor_HAL_EncoderData_t *data) {
  data->position_rad = 3.1415926f;
  data->velocity_rad = 6.2831853f;
  data->elec_angle = 1.25f;
}

static const Motor_HAL_AdcInterface_t k_adc = {.update = FakeAdcUpdate};
static const Motor_HAL_PwmInterface_t k_pwm = {0};
static const Motor_HAL_EncoderInterface_t k_encoder = {
    .get_data = FakeEncoderGetData,
};
static const Motor_HAL_Handle_t k_hal = {
    .pwm = &k_pwm,
    .adc = &k_adc,
    .encoder = &k_encoder,
};

static void ResetState(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  memset(&g_ds402_state_machine, 0, sizeof(g_ds402_state_machine));
  memset(s_events, 0, sizeof(s_events));
  s_event_count = 0u;
  s_last_error_code = ERROR_NONE;
  s_last_error_message = NULL;
  s_error_handler_armed = false;
  s_runtime_ready_at_dmb = true;
  s_runtime_ready_at_motor_init = true;
  s_runtime_ready_at_runtime_apply = true;
  s_adc_bind_result = 0;
  s_pwm_bind_result = 0;
  s_parameter_bindings_result = PARAM_OK;
  s_encoder_init_result = 0;
  s_adc_bsp_init_result = true;
  s_pwm_start_sampling_result = 0;
  s_adc_calibrate_result = 0;
  s_encoder_update_result = 0;
  s_encoder_offset_result = 0;
  s_detection_slow_result = FAULT_NONE;
  s_app_comm_bootstrap_result = true;
  s_safety_has_active_fault = false;
  s_precalib_pass_mask = 0x0Fu;
  s_bound_adc = NULL;
  s_bound_pwm = NULL;
  motor_data.components.hal = &k_hal;
  motor_data.parameters.pole_pairs = 7;
}

static int RunAppInitExpectingError(void) {
  s_error_handler_armed = true;
  if (setjmp(s_error_handler_jump) == 0) {
    App_Init();
    return 1;
  }
  s_error_handler_armed = false;
  return 0;
}

void Test_DisableIrq(void) {}
void Test_EnableIrq(void) {}
void Test_DataMemoryBarrier(void) {
  s_runtime_ready_at_dmb = App_IsFocRuntimeReady();
  Record(EVENT_DMB);
}
void DWT_Delay(float seconds) { (void)seconds; }
void HAL_WatchdogFeed(void) {}
void BSPInit(void) { Record(EVENT_BSP_INIT); }
void LogInit(UART_HandleTypeDef *log_config) { (void)log_config; }
void RGB_DisplayColorById(uint8_t color_id) { (void)color_id; }
void ErrorManager_Init(void) { Record(EVENT_ERROR_MANAGER_INIT); }
void ErrorManager_ReportFull(uint32_t error_code, const char *message,
                             const char *file, uint32_t line) {
  (void)file;
  (void)line;
  s_last_error_code = error_code;
  s_last_error_message = message;
}
void Error_Handler(void) {
  Record(EVENT_ERROR_HANDLER);
  if (s_error_handler_armed) {
    longjmp(s_error_handler_jump, 1);
  }
}
void StateMachine_Init(StateMachine *sm) { sm->pre_check_callback = NULL; }
void StateMachine_SetPreCheckCallback(StateMachine *sm,
                                      bool (*callback)(MotorState to_state)) {
  sm->pre_check_callback = callback;
}
void Detection_Init(const DetectionConfig *config) { (void)config; }
uint32_t Detection_Check_Slow(void *motor) {
  (void)motor;
  Record(EVENT_DETECTION_SLOW);
  return s_detection_slow_result;
}
void Safety_Init(const SafetyConfig *config) { (void)config; }
void Safety_TriggerFault(uint32_t fault_bits, MOTOR_DATA *motor,
                         StateMachine *fsm) {
  (void)fault_bits;
  (void)motor;
  (void)fsm;
}
bool Safety_HasActiveFault(void) { return s_safety_has_active_fault; }
uint8_t Motor_PreCalibCheck(MOTOR_DATA *motor, uint8_t *fail_mask) {
  (void)motor;
  if (fail_mask != NULL) {
    *fail_mask = (uint8_t)~s_precalib_pass_mask;
  }
  return s_precalib_pass_mask;
}
void Init_Motor_No_Calib(MOTOR_DATA *motor) {
  (void)motor;
  s_runtime_ready_at_motor_init = App_IsFocRuntimeReady();
  Record(EVENT_INIT_MOTOR_NO_CALIB);
}
int MHAL_ADC_Bind(const Motor_HAL_AdcInterface_t *interface) {
  s_bound_adc = interface;
  Record(EVENT_ADC_BIND);
  return s_adc_bind_result;
}
int MHAL_PWM_Bind(const Motor_HAL_PwmInterface_t *interface) {
  s_bound_pwm = interface;
  Record(EVENT_PWM_BIND);
  return s_pwm_bind_result;
}
ParamResult ParameterBindingsSettings_Install(void) {
  Record(EVENT_PARAM_BINDINGS_INSTALL);
  return s_parameter_bindings_result;
}
void EncoderCalibrationSettings_InstallAdapter(void) {}
int MHAL_Encoder_Init(void) {
  Record(EVENT_ENCODER_INIT);
  return s_encoder_init_result;
}
ParamResult Param_SystemInitOnce(void) {
  Record(EVENT_PARAM_SYSTEM_INIT);
  return PARAM_OK;
}
bool adc_bsp_init(void) {
  Record(EVENT_ADC_BSP_INIT);
  return s_adc_bsp_init_result;
}
int MHAL_PWM_StartSampling(void) {
  Record(EVENT_PWM_START_SAMPLING);
  return s_pwm_start_sampling_result;
}
int MHAL_ADC_CalibrateCurrent(void) {
  Record(EVENT_ADC_CALIBRATE_CURRENT);
  return s_adc_calibrate_result;
}
int MHAL_PWM_Disable(void) {
  Record(EVENT_PWM_DISABLE);
  return 0;
}
int RuntimeSettings_ApplyEncoderOffset(void) {
  Record(EVENT_ENCODER_OFFSET_APPLY);
  return s_encoder_offset_result;
}
int MHAL_Encoder_Update(uint8_t pole_pairs) {
  (void)pole_pairs;
  return s_encoder_update_result;
}
bool AppComm_Bootstrap(void) {
  Record(EVENT_APP_COMM_BOOTSTRAP);
  return s_app_comm_bootstrap_result;
}
void RuntimeSettings_InstallAdapter(void) {
  Record(EVENT_RUNTIME_SETTINGS_INSTALL);
}
void Param_ApplyRuntimeState(void) {
  s_runtime_ready_at_runtime_apply = App_IsFocRuntimeReady();
  Record(EVENT_PARAM_APPLY_RUNTIME);
}

static int test_reports_adc_error_and_stops_when_motor_hal_is_missing(void) {
  ResetState();
  motor_data.components.hal = NULL;

  CHECK(RunAppInitExpectingError() == 0);

  CHECK(s_last_error_code == ERROR_HW_ADC_INIT);
  CHECK(strcmp(s_last_error_message, "Motor ADC port binding failed") == 0);
  CHECK(!EventWasRecorded(EVENT_ADC_BIND));
  CHECK(!EventWasRecorded(EVENT_PWM_BIND));
  CHECK(!App_IsFocRuntimeReady());
  return 0;
}

static int test_reports_adc_error_and_stops_when_adc_bind_fails(void) {
  ResetState();
  s_adc_bind_result = -1;

  CHECK(RunAppInitExpectingError() == 0);

  CHECK(s_last_error_code == ERROR_HW_ADC_INIT);
  CHECK(strcmp(s_last_error_message, "Motor ADC port binding failed") == 0);
  CHECK(s_bound_adc == &k_adc);
  CHECK(!EventWasRecorded(EVENT_PWM_BIND));
  CHECK(!App_IsFocRuntimeReady());
  return 0;
}

static int test_reports_pwm_error_and_stops_when_pwm_bind_fails(void) {
  ResetState();
  s_pwm_bind_result = -1;

  CHECK(RunAppInitExpectingError() == 0);

  CHECK(s_last_error_code == ERROR_HW_PWM_INIT);
  CHECK(strcmp(s_last_error_message, "Motor PWM port binding failed") == 0);
  CHECK(s_bound_pwm == &k_pwm);
  CHECK(!EventWasRecorded(EVENT_PARAM_BINDINGS_INSTALL));
  CHECK(!App_IsFocRuntimeReady());
  return 0;
}

static int test_reports_pwm_error_and_stops_when_sampling_start_fails(void) {
  ResetState();
  s_pwm_start_sampling_result = -1;

  CHECK(RunAppInitExpectingError() == 0);

  CHECK(s_last_error_code == ERROR_HW_PWM_INIT);
  CHECK(strcmp(s_last_error_message, "PWM sampling trigger start failed") == 0);
  CHECK(EventIndex(EVENT_ADC_BSP_INIT) < EventIndex(EVENT_PWM_START_SAMPLING));
  CHECK(!EventWasRecorded(EVENT_ADC_CALIBRATE_CURRENT));
  CHECK(!App_IsFocRuntimeReady());
  return 0;
}

static int
test_reports_adc_error_and_stops_when_current_calibration_fails(void) {
  ResetState();
  s_adc_calibrate_result = -1;

  CHECK(RunAppInitExpectingError() == 0);

  CHECK(s_last_error_code == ERROR_HW_ADC_INIT);
  CHECK(strcmp(s_last_error_message, "Current offset calibration failed") == 0);
  CHECK(EventIndex(EVENT_PWM_START_SAMPLING) <
        EventIndex(EVENT_ADC_CALIBRATE_CURRENT));
  CHECK(!EventWasRecorded(EVENT_ENCODER_OFFSET_APPLY));
  CHECK(!App_IsFocRuntimeReady());
  return 0;
}

static int
test_keeps_runtime_readiness_false_until_successful_boot_completes(void) {
  ResetState();

  CHECK(!App_IsFocRuntimeReady());
  App_Init();

  CHECK(EventWasRecorded(EVENT_INIT_MOTOR_NO_CALIB));
  CHECK(EventWasRecorded(EVENT_PARAM_APPLY_RUNTIME));
  CHECK(EventWasRecorded(EVENT_DMB));
  CHECK(!s_runtime_ready_at_motor_init);
  CHECK(!s_runtime_ready_at_runtime_apply);
  CHECK(!s_runtime_ready_at_dmb);
  CHECK(App_IsFocRuntimeReady());
  return 0;
}

int main(void) {
  CHECK(test_reports_adc_error_and_stops_when_motor_hal_is_missing() == 0);
  CHECK(test_reports_adc_error_and_stops_when_adc_bind_fails() == 0);
  CHECK(test_reports_pwm_error_and_stops_when_pwm_bind_fails() == 0);
  CHECK(test_reports_pwm_error_and_stops_when_sampling_start_fails() == 0);
  CHECK(test_reports_adc_error_and_stops_when_current_calibration_fails() == 0);
  CHECK(test_keeps_runtime_readiness_false_until_successful_boot_completes() ==
        0);

  puts("App init binding tests PASSED");
  return 0;
}
