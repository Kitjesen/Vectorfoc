/**
 * @file isr_foc.c
 * @brief FOC实时控制中断 (20kHz)
 * @note 原 motor_task.c 内容迁移至此
 */

#include "isr_foc.h"

#include "adc.h"
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

/* ========== 内部函数 ========== */

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
    motor->algo_input.theta_elec = enc_data.elec_angle;
  }
}

static inline void ISR_RunAdvancedControl(MOTOR_DATA *motor) {
  Motor_API_Feedforward_Update(motor);
  Motor_API_Cogging_Update(motor);
}

/* ========== 中断回调 ========== */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != hadc1.Instance)
    return;

#if ADJUST_EN
  ISR_UpdateSensors(&motor_data);
#else
  static uint16_t s_fsm_counter = 0;
  static uint16_t s_adv_counter = 0;
#if HS_LOG_ENABLE
  static uint16_t s_log_counter = 0;
#endif

  // 1. 喂独立看门狗
  HAL_WatchdogFeed();

  // 2. 采集传感器数据
  ISR_UpdateSensors(&motor_data);

  // 3. 更新编码器数据
  ISR_UpdateEncoder(&motor_data);

  // 4. 观测器更新
  Motor_API_Observer_Update(&motor_data);

  // 5. 快速安全检测 (20kHz)
  Safety_Update_Fast(&motor_data, &g_ds402_state_machine);

  // 6. 状态机更新
  if (++s_fsm_counter >= FSM_UPDATE_DIV) {
    s_fsm_counter = 0;
    StateMachine_Update(&g_ds402_state_machine);
  }

  // 7. 高级控制算法
  if (++s_adv_counter >= ADV_CONTROL_DIV) {
    s_adv_counter = 0;
    ISR_RunAdvancedControl(&motor_data);
  }

  // 8. FOC控制
  MotorStateTask(&motor_data);

  // 9. 高速数据记录
#if HS_LOG_ENABLE
  if (++s_log_counter >= HS_LOG_DIV) {
    s_log_counter = 0;
    Scope_Update();
  }
#endif
#endif
}
