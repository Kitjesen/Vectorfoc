#include "inner.h"
#include "config.h"
#include "control/cogging.h"
#include "control/field_weakening.h"
#include "foc/foc_algorithm.h"
#include "foc/math_common.h"
#include "motor_hal_api.h"
#include <math.h>
/**
 * @brief currentinner loop： algo_*  HAL PWM， FOC_DATA
 */
void Control_InnerCurrentLoop(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  (void)ctx;  // 当前未使用，保留接口兼容性
  
  // [FIX] 添加空指针检查
  if (motor == NULL) {
    return;
  }
  
  // 1. configparam (update)
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
  // 2. input Motor_UpdateSensors / Motor_UpdateEncoder  algo_input
  motor->algo_input.omega_elec =
      motor->feedback.velocity * motor->parameters.pole_pairs * M_2PI;
  motor->algo_input.enabled = true;
  motor->algo_input.Iq_ref += CoggingComp_GetCurrent(motor);
  FieldWeakening_Config_t fw_cfg;
  fw_cfg.max_weakening_current = motor->advanced.fw_max_current;
  fw_cfg.start_velocity = motor->advanced.fw_start_velocity;
  FieldWeakening_Update(motor, &fw_cfg);
  // 3.  FOC
  FOC_Algorithm_CurrentLoop(&motor->algo_input, &motor->algo_config,
                            &motor->algo_state, &motor->algo_output);
  // 4. output HAL PWM (voltage + current)
  float Ta = motor->algo_output.Ta;
  float Tb = motor->algo_output.Tb;
  float Tc = motor->algo_output.Tc;
#ifdef DEADTIME_COMP
  float deadtime_duty = (float)DEADTIME_COMP / (float)PWM_ARR;
  
  // [FIX] 使用实际相电流方向判断死区补偿方向（而非电压方向）
  // 电流方向更准确，尤其在动态响应和电流过零时
  float Ia = motor->algo_input.Ia;
  float Ib = motor->algo_input.Ib;
  float Ic = motor->algo_input.Ic;
  
  // 电流过零阈值：应略大于 ADC 噪声的 2~3 倍
  // TODO: 从 config 读取，或通过标定自动估算
  float i_thresh = motor->algo_config.deadtime_i_thresh;
  if (i_thresh <= 0.0f) i_thresh = 0.2f;  // 默认 0.2A（假设 ADC 噪声 ~0.1A）
  
  // 根据电流方向确定补偿符号
  // 电流为正时，上管导通时间不足，需要增加占空比 (+)
  // 电流为负时，下管导通时间不足，需要减少占空比 (-)
  float s_a, s_b, s_c;
  
  // A 相：过零区间内线性插值
  if (fabsf(Ia) < i_thresh) {
    s_a = Ia / i_thresh;  // 平滑过渡 [-1, 1]
  } else {
    s_a = (Ia > 0.0f) ? 1.0f : -1.0f;
  }
  
  // B 相
  if (fabsf(Ib) < i_thresh) {
    s_b = Ib / i_thresh;
  } else {
    s_b = (Ib > 0.0f) ? 1.0f : -1.0f;
  }
  
  // C 相
  if (fabsf(Ic) < i_thresh) {
    s_c = Ic / i_thresh;
  } else {
    s_c = (Ic > 0.0f) ? 1.0f : -1.0f;
  }
  
  // 补偿量 = 死区时间比例 + 体二极管压降
  // Vdiode ≈ 0.7V (MOSFET 体二极管典型值，低压系统影响显著)
  float Vbus = motor->algo_input.Vbus;
  float Vdiode = motor->algo_config.deadtime_Vdiode;
  if (Vdiode <= 0.0f) Vdiode = 0.7f;  // 默认 0.7V
  float Vcomp = (Vbus > 1.0f) ? (deadtime_duty + Vdiode / Vbus) : deadtime_duty;
  
  Ta += Vcomp * s_a;
  Tb += Vcomp * s_b;
  Tc += Vcomp * s_c;
  Ta = CLAMP(Ta, 0.0f, 1.0f);
  Tb = CLAMP(Tb, 0.0f, 1.0f);
  Tc = CLAMP(Tc, 0.0f, 1.0f);
#endif
  if (motor->components.hal && motor->components.hal->pwm)
    motor->components.hal->pwm->set_duty(Ta, Tb, Tc);
}
