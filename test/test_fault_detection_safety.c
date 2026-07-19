// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_detection.h"
#include "config.h"
#include "error_types.h"
#include "motor.h"
#include "position_sensor.h"
#include "fsm.h"
#include "mock_hal_types.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_tick_ms;
static float s_velocity;
static uint32_t s_last_error_code;
static uint32_t s_error_report_count;
static uint32_t s_error_clear_domain_count;
static uint32_t s_fault_callback_count;
static uint32_t s_last_callback_fault_bits;
static bool s_fault_callback_succeeds;
static bool s_position_sensor_present = true;
static bool s_position_sensor_initialized = true;
static PositionSensorStatus_t s_position_sensor_health_status =
    POSITION_SENSOR_STATUS_OK;
static PositionSensorHealth_t s_position_sensor_health = {.valid = true};
static const PositionSensorDescriptor_t s_position_sensor_descriptor = {
    .name = "test-position-sensor",
    .capabilities = POSITION_SENSOR_CAP_HEALTH,
};

#undef HAL_GetSystemTick
uint32_t HAL_GetSystemTick(void) { return s_tick_ms; }
uint32_t HAL_GetTick(void) { return s_tick_ms; }
float MHAL_Encoder_GetVelocity(void) { return s_velocity; }
const PositionSensorDescriptor_t *PositionSensor_GetDescriptor(void) {
  return s_position_sensor_present ? &s_position_sensor_descriptor : NULL;
}
bool PositionSensor_IsInitialized(void) {
  return s_position_sensor_initialized;
}
PositionSensorStatus_t
PositionSensor_GetHealth(PositionSensorHealth_t *health) {
  if (health != NULL &&
      s_position_sensor_health_status == POSITION_SENSOR_STATUS_OK) {
    *health = s_position_sensor_health;
  }
  return s_position_sensor_health_status;
}
uint32_t HAL_EnterCritical(void) { return 0u; }
void HAL_ExitCritical(uint32_t previous_state) { (void)previous_state; }
int MHAL_PWM_Enable(void) { return 0; }
int MHAL_PWM_Disable(void) { return 0; }
int MHAL_PWM_Brake(void) { return 0; }

void ErrorManager_ReportFull(uint32_t error_code, const char *message,
                             const char *file, uint32_t line) {
  (void)message;
  (void)file;
  (void)line;
  s_last_error_code = error_code;
  s_error_report_count++;
}

void ErrorManager_ClearDomain(ErrorDomain domain) {
  assert(domain == ERROR_DOMAIN_SAFETY);
  s_error_clear_domain_count++;
}

static bool test_fault_callback(uint32_t fault_bits, void *motor) {
  (void)motor;
  s_fault_callback_count++;
  s_last_callback_fault_bits = fault_bits;
  return s_fault_callback_succeeds;
}

#include "../Src/ALGO/motor/fsm.c"
#include "../Src/SAFE/safety_control.c"

static MOTOR_DATA make_motor(CONTROL_MODE mode) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  s_position_sensor_present = true;
  s_position_sensor_initialized = true;
  s_position_sensor_health_status = POSITION_SENSOR_STATUS_OK;
  memset(&s_position_sensor_health, 0, sizeof(s_position_sensor_health));
  s_position_sensor_health.valid = true;
  motor.state.State_Mode = STATE_MODE_IDLE;
  motor.state.Control_Mode = mode;
  motor.algo_input.Vbus = 24.0f;
  motor.feedback.temperature = 25.0f;
  return motor;
}

static void disable_unrelated_slow_checks(void) {
  DetectionConfig *config = Detection_GetConfig();
  config->enable_voltage_protection = false;
  config->enable_temp_protection = false;
  config->enable_stall_protection = false;
  config->enable_can_timeout = false;
}

static void test_encoder_health_counters_are_consumed_without_recounting(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);
  disable_unrelated_slow_checks();

  s_position_sensor_health.valid = false;
  s_position_sensor_health.consecutive_failures =
      FAULT_ENCODER_ERR_CONSECUTIVE_MAX - 1u;
  s_position_sensor_health.total_failures = 41u;
  s_position_sensor_health.transport_error_score = 37u;

  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) == 0u);
  assert(Detection_GetState()->encoder_err_consecutive ==
         FAULT_ENCODER_ERR_CONSECUTIVE_MAX - 1u);
  assert(Detection_GetState()->encoder_err_count == 41u);

  /* Polling the same health frame must not add another safety-layer error. */
  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) == 0u);
  assert(Detection_GetState()->encoder_err_consecutive ==
         FAULT_ENCODER_ERR_CONSECUTIVE_MAX - 1u);
  assert(Detection_GetState()->encoder_err_count == 41u);

  s_position_sensor_health.consecutive_failures =
      FAULT_ENCODER_ERR_CONSECUTIVE_MAX;
  s_position_sensor_health.transport_error_score = 43u;
  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) != 0u);
  assert(Detection_GetState()->encoder_err_count == 43u);

  s_position_sensor_health.valid = true;
  s_position_sensor_health.consecutive_failures = 0u;
  s_position_sensor_health.total_failures = 44u;
  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) == 0u);
  assert(Detection_GetState()->encoder_err_consecutive == 0u);
  assert(Detection_GetState()->encoder_err_count == 44u);
}

static void test_unavailable_encoder_health_uses_existing_fault_threshold(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);
  disable_unrelated_slow_checks();

  s_position_sensor_present = false;
  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) == 0u);
  assert(Detection_GetState()->encoder_err_consecutive == 1u);

  s_position_sensor_present = true;
  s_position_sensor_initialized = false;
  for (uint32_t i = 2u; i < FAULT_ENCODER_ERR_CONSECUTIVE_MAX; ++i) {
    assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) == 0u);
    assert(Detection_GetState()->encoder_err_consecutive == i);
  }

  s_position_sensor_initialized = true;
  s_position_sensor_health_status = POSITION_SENSOR_STATUS_IO_ERROR;
  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) != 0u);

  s_position_sensor_health_status = POSITION_SENSOR_STATUS_OK;
  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) == 0u);
  assert(Detection_GetState()->encoder_err_consecutive == 0u);
}

static void test_invalid_driver_health_without_transport_count_still_faults(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);
  disable_unrelated_slow_checks();

  /* Hall/commutation validity can fail without a transport/update error. */
  s_position_sensor_health.valid = false;
  s_position_sensor_health.consecutive_failures = 0u;
  s_position_sensor_health.total_failures = 0u;
  s_position_sensor_health.transport_error_score = 0u;

  for (uint32_t i = 1u; i < FAULT_ENCODER_ERR_CONSECUTIVE_MAX; ++i) {
    assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) == 0u);
    assert(Detection_GetState()->encoder_err_consecutive == i);
  }
  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) != 0u);

  s_position_sensor_health.valid = true;
  assert((Detection_Check_Slow(&motor) & FAULT_ENCODER_LOSS) == 0u);
  assert(Detection_GetState()->encoder_err_consecutive == 0u);
}

static void test_open_loop_keeps_electrical_and_thermal_protection(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_OPEN);
  Detection_Init(NULL);

  motor.algo_input.Ia = 100.0f;
  assert((Detection_Check_Fast(&motor) & FAULT_CURRENT_A) != 0u);

  Detection_Init(NULL);
  motor.algo_input.Ia = 0.0f;
  motor.algo_input.Vbus = 70.0f;
  assert((Detection_Check_Slow(&motor) & FAULT_OVER_VOLTAGE) != 0u);

  Detection_Init(NULL);
  motor.algo_input.Vbus = 24.0f;
  motor.feedback.temperature = 160.0f;
  assert((Detection_Check_Slow(&motor) & FAULT_OVER_TEMP) != 0u);
}

static void test_non_finite_sensor_values_fail_safe(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);

  motor.algo_input.Ia = NAN;
  motor.algo_input.Ib = INFINITY;
  motor.algo_input.Ic = -INFINITY;
  uint32_t fault = Detection_Check_Fast(&motor);
  assert((fault & FAULT_OVER_CURRENT) == FAULT_OVER_CURRENT);

  Detection_Init(NULL);
  motor.algo_input.Ia = motor.algo_input.Ib = motor.algo_input.Ic = 0.0f;
  motor.algo_input.Vbus = NAN;
  fault = Detection_Check_Slow(&motor);
  assert((fault & (FAULT_OVER_VOLTAGE | FAULT_UNDER_VOLTAGE)) != 0u);

  Detection_Init(NULL);
  motor.algo_input.Vbus = 24.0f;
  motor.feedback.temperature = NAN;
  assert((Detection_Check_Slow(&motor) & FAULT_OVER_TEMP) != 0u);
}

static void test_stall_timeout_uses_elapsed_milliseconds(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);
  DetectionConfig *config = Detection_GetConfig();
  config->enable_voltage_protection = false;
  config->enable_temp_protection = false;
  config->enable_can_timeout = false;
  config->stall_current_threshold = 5.0f;
  config->stall_velocity_threshold = 1.0f;
  config->stall_detect_time_ms = 500u;
  motor.algo_input.Ia = 10.0f;
  motor.algo_input.Ib = 10.0f;
  motor.algo_input.Ic = 10.0f;
  s_velocity = 0.0f;

  s_tick_ms = 1000u;
  assert((Detection_Check_Slow(&motor) & FAULT_STALL_OVERLOAD) == 0u);
  s_tick_ms = 1499u;
  assert((Detection_Check_Slow(&motor) & FAULT_STALL_OVERLOAD) == 0u);
  s_tick_ms = 1500u;
  assert((Detection_Check_Slow(&motor) & FAULT_STALL_OVERLOAD) != 0u);
}

static void test_can_timeout_configuration_and_recovery(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  motor.state.State_Mode = STATE_MODE_RUNNING;
  Detection_Init(NULL);
  DetectionConfig *config = Detection_GetConfig();
  config->enable_voltage_protection = false;
  config->enable_temp_protection = false;
  config->enable_stall_protection = false;

  s_tick_ms = 2000u;
  Detection_SetCANTimeout(100u);
  assert(config->enable_can_timeout);
  assert(config->can_timeout_ms == 100u);
  Detection_FeedWatchdog(s_tick_ms);
  s_tick_ms = 2100u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);
  s_tick_ms = 2101u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) != 0u);

  Detection_FeedWatchdog(s_tick_ms);
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);

  Detection_SetCANTimeout(0u);
  assert(!config->enable_can_timeout);
  s_tick_ms += 10000u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);
}

static void test_default_can_timeout_is_nonzero_but_unarmed_until_can_frame(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);
  DetectionConfig *config = Detection_GetConfig();
  config->enable_voltage_protection = false;
  config->enable_temp_protection = false;
  config->enable_stall_protection = false;

  assert(DEFAULT_CAN_TIMEOUT_MS > 0u);

  s_tick_ms = 7000u;
  Detection_SetCANTimeout(DEFAULT_CAN_TIMEOUT_MS);
  assert(config->enable_can_timeout);
  assert(config->can_timeout_ms == DEFAULT_CAN_TIMEOUT_MS);

  s_tick_ms = 7000u + DEFAULT_CAN_TIMEOUT_MS + 1u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);

  motor.state.State_Mode = STATE_MODE_RUNNING;
  s_tick_ms += DEFAULT_CAN_TIMEOUT_MS + 1u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);

  Detection_FeedWatchdog(s_tick_ms);
  s_tick_ms += DEFAULT_CAN_TIMEOUT_MS;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);
  s_tick_ms++;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) != 0u);

  Detection_SetCANTimeout(0u);
  assert(!config->enable_can_timeout);
  s_tick_ms += DEFAULT_CAN_TIMEOUT_MS + 1u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);
}

static void test_can_timeout_ignores_idle_and_calibration_when_armed(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);
  DetectionConfig *config = Detection_GetConfig();
  config->enable_voltage_protection = false;
  config->enable_temp_protection = false;
  config->enable_stall_protection = false;

  s_tick_ms = 9000u;
  Detection_SetCANTimeout(100u);
  Detection_FeedWatchdog(s_tick_ms);

  s_tick_ms = 9101u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);

  motor.state.State_Mode = STATE_MODE_DETECTING;
  s_tick_ms = 9202u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);

  motor.state.State_Mode = STATE_MODE_RUNNING;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) != 0u);
}

static void test_clear_faults_reports_pending_fast_path_fault(void) {
  StateMachine fsm;
  StateMachine_Init(&fsm);
  s_last_error_code = ERROR_NONE;
  s_error_report_count = 0u;
  s_error_clear_domain_count = 0u;
  s_tick_ms = 3000u;

  Safety_Init(NULL);
  Safety_TriggerFault(FAULT_OVER_CURRENT, NULL, &fsm);

  assert(Safety_HasActiveFault());
  assert(s_error_report_count == 0u);
  assert(fsm.fault_history_index == 1u);
  assert(StateMachine_GetState(&fsm) == STATE_FAULT_REACTION_ACTIVE);

  StateMachine_Update(&fsm);
  assert(StateMachine_GetState(&fsm) == STATE_FAULT);

  assert(Safety_ClearFaults(&fsm));

  assert(s_error_report_count == 1u);
  assert(s_last_error_code == ERROR_SAFETY_OVERCURRENT);
  assert(s_error_clear_domain_count == 1u);
  assert(fsm.fault_history_index == 1u);
  assert(StateMachine_GetState(&fsm) == STATE_SWITCH_ON_DISABLED);
  assert(!Safety_HasActiveFault());
}

static void test_clear_faults_retries_failed_callback_without_duplicate_history(void) {
  StateMachine fsm;
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  SafetyConfig config = DEFAULT_SAFETY_CONFIG;
  StateMachine_Init(&fsm);
  s_last_error_code = ERROR_NONE;
  s_error_report_count = 0u;
  s_error_clear_domain_count = 0u;
  s_fault_callback_count = 0u;
  s_last_callback_fault_bits = FAULT_NONE;
  s_fault_callback_succeeds = false;
  s_tick_ms = 4000u;
  config.fault_callback = test_fault_callback;

  Safety_Init(&config);
  Safety_TriggerFault(FAULT_OVER_CURRENT, &motor, &fsm);
  assert(s_fault_callback_count == 0u);
  StateMachine_Update(&fsm);
  assert(Safety_ClearFaults(&fsm));

  assert(s_error_report_count == 1u);
  assert(fsm.fault_history_index == 1u);
  assert(s_fault_callback_count == 1u);
  assert(s_last_callback_fault_bits == FAULT_OVER_CURRENT);
  assert(!Safety_HasActiveFault());

  s_fault_callback_succeeds = true;
  Safety_Update_Slow(&motor, &fsm);

  assert(s_fault_callback_count == 2u);
  assert(s_last_callback_fault_bits == FAULT_OVER_CURRENT);
  assert(s_error_report_count == 1u);
  assert(fsm.fault_history_index == 1u);
  assert(!Safety_HasActiveFault());
}

static void test_slow_publish_does_not_repeat_fast_fsm_fault(void) {
  StateMachine fsm;
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  SafetyConfig config = DEFAULT_SAFETY_CONFIG;
  StateMachine_Init(&fsm);
  s_last_error_code = ERROR_NONE;
  s_error_report_count = 0u;
  s_error_clear_domain_count = 0u;
  s_fault_callback_count = 0u;
  s_last_callback_fault_bits = FAULT_NONE;
  s_fault_callback_succeeds = true;
  s_tick_ms = 5000u;
  config.fault_callback = test_fault_callback;

  Safety_Init(&config);
  Safety_TriggerFault(FAULT_OVER_CURRENT, &motor, &fsm);

  assert(s_error_report_count == 0u);
  assert(s_fault_callback_count == 0u);
  assert(fsm.fault_history_index == 1u);

  Safety_Update_Slow(&motor, &fsm);

  assert(s_error_report_count == 1u);
  assert(s_fault_callback_count == 1u);
  assert(fsm.fault_history_index == 1u);

  Safety_Update_Slow(&motor, &fsm);

  assert(s_error_report_count == 1u);
  assert(s_fault_callback_count == 1u);
  assert(fsm.fault_history_index == 1u);
}

static void test_clear_faults_waits_for_final_fsm_fault_state(void) {
  StateMachine fsm;
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  StateMachine_Init(&fsm);
  s_last_error_code = ERROR_NONE;
  s_error_report_count = 0u;
  s_error_clear_domain_count = 0u;
  s_tick_ms = 6000u;

  Safety_Init(NULL);
  Safety_TriggerFault(FAULT_OVER_CURRENT, &motor, &fsm);

  assert(StateMachine_GetState(&fsm) == STATE_FAULT_REACTION_ACTIVE);
  assert(Safety_HasActiveFault());
  assert(!Safety_ClearFaults(&fsm));
  assert(StateMachine_GetState(&fsm) == STATE_FAULT_REACTION_ACTIVE);
  assert(Safety_HasActiveFault());
  assert(s_error_report_count == 0u);
  assert(s_error_clear_domain_count == 0u);

  StateMachine_Update(&fsm);
  assert(StateMachine_GetState(&fsm) == STATE_FAULT);
  assert(Safety_ClearFaults(&fsm));

  assert(StateMachine_GetState(&fsm) == STATE_SWITCH_ON_DISABLED);
  assert(!Safety_HasActiveFault());
  assert(s_error_report_count == 1u);
  assert(s_error_clear_domain_count == 1u);
}

int main(void) {
  test_encoder_health_counters_are_consumed_without_recounting();
  test_unavailable_encoder_health_uses_existing_fault_threshold();
  test_invalid_driver_health_without_transport_count_still_faults();
  test_open_loop_keeps_electrical_and_thermal_protection();
  test_non_finite_sensor_values_fail_safe();
  test_stall_timeout_uses_elapsed_milliseconds();
  test_can_timeout_configuration_and_recovery();
  test_default_can_timeout_is_nonzero_but_unarmed_until_can_frame();
  test_can_timeout_ignores_idle_and_calibration_when_armed();
  test_clear_faults_reports_pending_fast_path_fault();
  test_clear_faults_retries_failed_callback_without_duplicate_history();
  test_slow_publish_does_not_repeat_fast_fsm_fault();
  test_clear_faults_waits_for_final_fsm_fault_state();
  puts("Fault detection safety tests: PASS");
  return 0;
}
