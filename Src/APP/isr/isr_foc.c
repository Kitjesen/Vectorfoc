/**
 * @file isr_foc.c
 * @brief FOCinterrupt (20kHz)
 * @note  motor_task.c
 */
#include "isr_foc.h"
#include "adc.h"
#include "board_config.h"
#include "hal_abstraction.h" // For HAL_WatchdogFeed()
#include "main.h"
#include "motor.h"
#include "motor_api.h"
#include "fsm.h"
#include "motor_adc.h"
#include "safety_control.h"
#include "vofa.h"
#define ADJUST_EN 0
#define FSM_UPDATE_DIV MOTOR_DECIM_DIV(CONTROL_FREQ_HZ, FSM_UPDATE_HZ)
#define ADV_CONTROL_DIV MOTOR_DECIM_DIV(CONTROL_FREQ_HZ, ADV_CONTROL_HZ)
#define HS_LOG_DIV MOTOR_DECIM_DIV(CONTROL_FREQ_HZ, HS_LOG_HZ)
/* ==========  ========== */
static inline void ISR_UpdateSensors(MOTOR_DATA *motor) {
  Motor_HAL_SensorData_t sensor_data;
  motor->components.hal->adc->update(&sensor_data);
  motor->algo_input.Ia = sensor_data.i_a;
  motor->algo_input.Ib = sensor_data.i_b;
  motor->algo_input.Ic = sensor_data.i_c;
  motor->algo_input.Vbus = sensor_data.v_bus;
  motor->feedback.temperature = sensor_data.temp;
}
static inline void ISR_UpdateEncoder(MOTOR_DATA *motor) {
  if (motor->components.hal && motor->components.hal->encoder) {
    Motor_HAL_EncoderData_t enc_data;
    motor->components.hal->encoder->update();
    motor->components.hal->encoder->get_data(&enc_data);
    motor->feedback.position = enc_data.angle_rad / M_2PI;
    motor->feedback.velocity = enc_data.velocity_rad / M_2PI;
    motor->feedback.phase_angle = enc_data.elec_angle;
    /* Open-loop: let control module accumulate theta_elec freely */
    if (motor->state.Control_Mode != CONTROL_MODE_OPEN) {
      motor->algo_input.theta_elec = enc_data.elec_angle;
    }
  }
}
static inline void ISR_RunAdvancedControl(MOTOR_DATA *motor) {
  Motor_API_Feedforward_Update(motor);
  Motor_API_Cogging_Update(motor);
}
/* ========== interrupt ========== */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != HW_ADC_CURRENT.Instance)
    return;
#if ADJUST_EN
  ISR_UpdateSensors(&motor_data);
#else
  static uint16_t s_fsm_counter = 0;
  static uint16_t s_adv_counter = 0;
#if HS_LOG_ENABLE
  static uint16_t s_log_counter = 0;
#endif
  // 1.
  HAL_WatchdogFeed();
  // 2. acquisition
  ISR_UpdateSensors(&motor_data);
  // 3. updateencoder
  ISR_UpdateEncoder(&motor_data);
  // 4. observerupdate
  Motor_API_Observer_Update(&motor_data);
  // 5. safety (20kHz)
  Safety_Update_Fast(&motor_data, &g_ds402_state_machine);
  // 6. stateupdate
  if (++s_fsm_counter >= FSM_UPDATE_DIV) {
    s_fsm_counter = 0;
    StateMachine_Update(&g_ds402_state_machine);
  }
  // 7.
  if (++s_adv_counter >= ADV_CONTROL_DIV) {
    s_adv_counter = 0;
    ISR_RunAdvancedControl(&motor_data);
  }
  // 8. FOC
  MotorStateTask(&motor_data);
  // 9.
#if HS_LOG_ENABLE
  if (++s_log_counter >= HS_LOG_DIV) {
    s_log_counter = 0;
    Scope_Update();
  }
#endif
#endif
}
