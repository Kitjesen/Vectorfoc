#include "startup_test_support.h"
#include "motor_sampling.h"
#include "error_manager.h"
#include "hardware_interface.h"
#include "parameter_access.h"
#include "board_configuration.h"
#include <string.h>
MOTOR_DATA motor_data;
StateMachine g_ds402_state_machine;
static StartupADCRegisters adc1_regs, adc2_regs;
ADC_HandleTypeDef hadc1 = {&adc1_regs}, hadc2 = {&adc2_regs};
float startup_test_offsets[3];
unsigned test_enable_calls, test_sample_count, test_control_calls;
unsigned test_adc_calls, test_adc_fail_call;
unsigned test_enable_fail_call, test_stop_calls, test_stop_fail_call;
int test_enable_result, test_sampling_result;
bool test_power_on, test_sampling_on;
uint32_t test_irq_mask;
PositionSensorHealth_t test_sensor_health;
void (*test_enable_hook)(void);
unsigned test_configured_gain_calls, test_recomputed_gain_calls;
TIM_HandleTypeDef htim1;
unsigned TestHardware_PhaseMask(void) {
  unsigned mask = 0;
  for (unsigned i = 0; i < 3; ++i) {
    if (htim1.main_state[i] == HAL_TIM_CHANNEL_STATE_BUSY) mask |= 1u << (2*i);
    if (htim1.complement_state[i] == HAL_TIM_CHANNEL_STATE_BUSY) mask |= 2u << (2*i);
  }
  return mask;
}
static HAL_StatusTypeDef phase_start(unsigned *state) {
  ++test_enable_calls;
  if (test_enable_hook) {
    void (*hook)(void) = test_enable_hook;
    test_enable_hook = NULL;
    hook();
  }
  if (test_enable_result || test_enable_calls == test_enable_fail_call) return HAL_ERROR;
  *state = HAL_TIM_CHANNEL_STATE_BUSY;
  test_power_on = TestHardware_PhaseMask() != 0;
  return HAL_OK;
}
static HAL_StatusTypeDef phase_stop(unsigned *state) {
  ++test_stop_calls;
  if (test_stop_calls == test_stop_fail_call) return HAL_ERROR;
  *state = HAL_TIM_CHANNEL_STATE_READY;
  test_power_on = TestHardware_PhaseMask() != 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
  if (channel == TIM_CHANNEL_4) {
    if (test_sampling_result) return HAL_ERROR;
    timer->main_state[channel] = HAL_TIM_CHANNEL_STATE_BUSY;
    test_sampling_on = true;
    return HAL_OK;
  }
  return phase_start(&timer->main_state[channel]);
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel) {
  if (channel == TIM_CHANNEL_4) test_sampling_on = false;
  return phase_stop(&timer->main_state[channel]);
}
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
  return phase_start(&timer->complement_state[channel]);
}
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *timer, uint32_t channel) {
  return phase_stop(&timer->complement_state[channel]);
}
void TestHardware_Reset(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  test_enable_calls = test_sample_count = test_control_calls = 0;
  test_enable_hook = NULL;
  test_configured_gain_calls = test_recomputed_gain_calls = 0;
  test_enable_fail_call = test_stop_calls = test_stop_fail_call = 0;
  memset(&htim1, 0, sizeof(htim1));
  htim1.arr = 1000;
  test_adc_calls = test_adc_fail_call = 0;
  test_enable_result = test_sampling_result = 0;
  test_power_on = test_sampling_on = false;
  test_irq_mask = 0;
  memset(startup_test_offsets, 0, sizeof(startup_test_offsets));
  adc1_regs = (StartupADCRegisters){2048,2040,2048,2500};
  adc2_regs = adc1_regs;
  test_sensor_health = (PositionSensorHealth_t){
      .valid = true,
      .calibrated = true,
  };
}
void ADC_SetCurrentOffsets(float a, float b, float c) {
  startup_test_offsets[0]=a; startup_test_offsets[1]=b; startup_test_offsets[2]=c;
}
static HAL_StatusTypeDef adc_call(void) {
  return ++test_adc_calls == test_adc_fail_call ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *a, uint32_t mode) {
  (void)a; (void)mode; return adc_call();
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *a) { (void)a; return adc_call(); }
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *a) { (void)a; return adc_call(); }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *a, uint32_t *p, uint32_t n) {
  (void)a; (void)p; (void)n; return adc_call();
}
void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_GetSystemTick(void) { return test_sample_count / 20; }
uint32_t HAL_GetTick(void) { return HAL_GetSystemTick(); }
uint32_t HAL_EnterCritical(void) { uint32_t previous=test_irq_mask; test_irq_mask=1; return previous; }
void HAL_ExitCritical(uint32_t previous) { test_irq_mask=previous; }
void ErrorManager_Init(void) {}
void ErrorManager_ReportFull(uint32_t c,const char *m,const char *f,uint32_t l) {
  (void)c; (void)m; (void)f; (void)l;
}
void ErrorManager_ClearDomain(ErrorDomain domain) { (void)domain; }

/* Control algorithms and driven motor identification are outside this test.
 * Their boundaries stay inert; startup, FSM, protection and cogging remain real. */
bool Control_RunCurrentCycle(MOTOR_DATA *motor) {
  (void)motor;
  ++test_control_calls;
  return true;
}
void FOC_Algorithm_ResetState(FOC_AlgorithmState_t *state) { memset(state,0,sizeof(*state)); }
CalibResult RSLSCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx, float dt) {
  (void)motor; (void)ctx; (void)dt; return CALIB_FAILED_INVALID_PARAMS;
}
CalibResult FluxCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx) {
  (void)motor; (void)ctx; return CALIB_FAILED_INVALID_PARAMS;
}
void Param_ScheduleSave(void) {}

const PositionSensorDescriptor_t *PositionSensor_GetDescriptor(void) {
  static const PositionSensorDescriptor_t descriptor = {
      .name = "startup-test",
      .capabilities = POSITION_SENSOR_CAP_ABSOLUTE | POSITION_SENSOR_CAP_HEALTH,
  };
  return &descriptor;
}
bool PositionSensor_IsInitialized(void) { return true; }
PositionSensorStatus_t PositionSensor_GetHealth(PositionSensorHealth_t *health) {
  if (health == NULL)
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  *health = test_sensor_health;
  return POSITION_SENSOR_STATUS_OK;
}

void RGB_DisplayColorById(uint8_t color) { (void)color; }
ParamResult Param_SystemInitOnce(void) { return (ParamResult)0; }
void Control_Initialize(MOTOR_DATA *motor) { (void)motor; }
void Control_UpdateCurrentGains(MOTOR_DATA *motor) {
  (void)motor; ++test_recomputed_gain_calls;
}
void Control_ApplyConfiguredCurrentGains(MOTOR_DATA *motor) {
  (void)motor; ++test_configured_gain_calls;
}
CalibResult RSLSCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx) {
  (void)motor; (void)ctx; return CALIB_IN_PROGRESS;
}
uint8_t RSLSCalib_GetProgress(CalibrationContext *ctx) { (void)ctx; return 0; }
