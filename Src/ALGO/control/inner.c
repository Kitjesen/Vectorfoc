#include "inner.h"
#include "config.h"
#include "control/cogging.h"
#include "control/field_weakening.h"
#include "foc/foc_algorithm.h"
#include "motor_hal_api.h"
#include <math.h>

/**
 * @brief 电流内环：使用 algo_* 与 HAL PWM，无 FOC_DATA 依赖
 */
void Control_InnerCurrentLoop(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  // 1. 配置参数 (仅在标记更新时同步)
  if (motor->params_updated) {
    motor->algo_config.Rs = motor->parameters.Rs;
    motor->algo_config.Ls = motor->parameters.Ls;
    motor->algo_config.flux = motor->parameters.flux;
    motor->algo_config.pole_pairs = motor->parameters.pole_pairs;

    motor->algo_config.Kp_current_d = motor->IdPID.Kp;
    motor->algo_config.Ki_current_d = motor->IdPID.Ki;
    motor->algo_config.Kp_current_q = motor->IqPID.Kp;
    motor->algo_config.Ki_current_q = motor->IqPID.Ki;

    motor->algo_config.Ts_current = CURRENT_MEASURE_PERIOD;
    motor->algo_config.voltage_limit = motor->Controller.voltage_limit;
    motor->algo_config.current_limit = motor->Controller.current_limit;
    motor->algo_config.enable_decoupling = true;

    motor->params_updated = false;
  }

  // 2. 输入由 Motor_UpdateSensors / Motor_UpdateEncoder 已写入 algo_input
  motor->algo_input.omega_elec =
      motor->feedback.velocity * motor->parameters.pole_pairs * M_2PI;
  motor->algo_input.enabled = true;
  motor->algo_input.Iq_ref += CoggingComp_GetCurrent(motor);

  FieldWeakening_Config_t fw_cfg;
  fw_cfg.max_weakening_current = motor->advanced.fw_max_current;
  fw_cfg.start_velocity = motor->advanced.fw_start_velocity;
  FieldWeakening_Update(motor, &fw_cfg);

  // 3. 执行 FOC 算法
  FOC_Algorithm_CurrentLoop(&motor->algo_input, &motor->algo_config,
                            &motor->algo_state, &motor->algo_output);

  // 4. 死区补偿后输出到 HAL PWM
  float Ta = motor->algo_output.Ta;
  float Tb = motor->algo_output.Tb;
  float Tc = motor->algo_output.Tc;

#ifdef DEADTIME_COMP
  float deadtime_duty = (float)DEADTIME_COMP / (float)PWM_ARR;
  if (motor->algo_input.Ia > 0.0f)
    Ta -= deadtime_duty;
  else
    Ta += deadtime_duty;
  if (motor->algo_input.Ib > 0.0f)
    Tb -= deadtime_duty;
  else
    Tb += deadtime_duty;
  if (motor->algo_input.Ic > 0.0f)
    Tc -= deadtime_duty;
  else
    Tc += deadtime_duty;
  Ta = CLAMP(Ta, 0.0f, 1.0f);
  Tb = CLAMP(Tb, 0.0f, 1.0f);
  Tc = CLAMP(Tc, 0.0f, 1.0f);
#endif

  if (motor->components.hal && motor->components.hal->pwm)
    motor->components.hal->pwm->set_duty(Ta, Tb, Tc);
}
