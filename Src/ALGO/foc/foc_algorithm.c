/**
 * @file    foc_algorithm.c
 * @brief   FOC算法实现（纯函数，硬件无关）
 */

#include "foc_algorithm.h"
#include "clarke.h"
#include "math_common.h"
#include "park.h"
#include "svpwm.h"
#include "trigonometry.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifndef MATH_PI
#define MATH_PI 3.14159265358979323846f
#endif

/* 私有函数声明 */

/* 公共函数实现 */

void FOC_Algorithm_InitState(FOC_AlgorithmState_t *state) {
  if (state == NULL)
    return;

  memset(state, 0, sizeof(FOC_AlgorithmState_t));
}

void FOC_Algorithm_CurrentLoop(const FOC_AlgorithmInput_t *input,
                               const FOC_AlgorithmConfig_t *config,
                               FOC_AlgorithmState_t *state,
                               FOC_AlgorithmOutput_t *output) {
  if (input == NULL || config == NULL || state == NULL || output == NULL) {
    return;
  }

  // 禁用时输出零
  if (!input->enabled) {
    output->Ta = 0.0f;
    output->Tb = 0.0f;
    output->Tc = 0.0f;
    output->overmodulation = false;
    output->voltage_saturated = false;
    output->current_limited = false;
    return;
  }

  // Step 1: Clarke变换 (abc → αβ)
  // 将三相静止坐标系电流 (Ia, Ib, Ic) 转换为两相静止坐标系电流 (Ialpha, Ibeta)
  // 公式: Ialpha = Ia, Ibeta = (Ia + 2*Ib) / sqrt(3)
  Clarke_Transform(input->Ia, input->Ib, input->Ic, &output->Ialpha,
                   &output->Ibeta);

  // Step 2: Park变换 (αβ → dq)
  // 将两相静止坐标系电流 (Ialpha, Ibeta) 转换为两相旋转坐标系电流 (Id, Iq)
  // 使用电角度 theta_elec 进行旋转变换
  Park_Transform(output->Ialpha, output->Ibeta, input->theta_elec, &output->Id,
                 &output->Iq);

  // Step 3: 电流反馈低通滤波
  // 对反馈电流 Id/Iq 进行一阶低通滤波，以抑制噪声
  // alpha = 2*pi*fc*Ts / (1 + 2*pi*fc*Ts) (Tustin 离散化近似)
  float omega_c = 2.0f * MATH_PI * config->current_filter_fc;
  float dt = config->Ts_current;
  float alpha_filter = (omega_c * dt) / (1.0f + omega_c * dt);

  if (config->current_filter_fc <= 0.0f)
    alpha_filter = 1.0f; // 如果截止频率 <= 0，则不滤波（直通）

  state->Id_filt += alpha_filter * (output->Id - state->Id_filt);
  state->Iq_filt += alpha_filter * (output->Iq - state->Iq_filt);

  // Step 4: 电流矢量限幅
  // 限制电流指令矢量的模长，防止过流
  // 如果 sqrt(Id^2 + Iq^2) > limit，则按比例缩小 Id 和 Iq
  float I_mag =
      sqrtf(input->Id_ref * input->Id_ref + input->Iq_ref * input->Iq_ref);
  float Id_cmd = input->Id_ref;
  float Iq_cmd = input->Iq_ref;

  if (I_mag > config->current_limit) {
    float scale = config->current_limit / I_mag;
    Id_cmd *= scale;
    Iq_cmd *= scale;
    output->current_limited = true;
  } else {
    output->current_limited = false;
  }

  // Step 5: PI电流控制器 (离散化带反向计算抗饱和)
  // V_pi = Kp*err + I
  // I += (Ki*err + Kb*(V_sat - V_raw)) * Ts
  // 实现了基本的 PI 控制，包含抗积分饱和逻辑 (Back-calculation)

  // D轴控制器 (控制磁通/弱磁)
  float error_d = Id_cmd - state->Id_filt; // 使用滤波后的反馈计算误差
  float Vd_raw = config->Kp_current_d * error_d + state->integral_d;

  // Q轴控制器 (控制转矩)
  float error_q = Iq_cmd - state->Iq_filt;
  float Vq_raw = config->Kp_current_q * error_q + state->integral_q;

  // Step 6: 前馈解耦
  // 补偿 D/Q 轴之间的交叉耦合项，提高动态响应
  // Vd_ff = -omega_e * Ls * Iq
  // Vq_ff = omega_e * (Ls * Id + flux)
  float Vd_ff = 0.0f;
  float Vq_ff = 0.0f;

  if (config->enable_decoupling) {
    float omega_e = input->omega_elec;
    Vd_ff = -omega_e * config->Ls * state->Iq_filt;
    Vq_ff = omega_e * (config->Ls * state->Id_filt + config->flux);

    Vd_ff *= config->decoupling_gain;
    Vq_ff *= config->decoupling_gain;
  }

  // 饱和前电压 (控制器输出 + 前馈补偿)
  output->Vd = Vd_raw + Vd_ff;
  output->Vq = Vq_raw + Vq_ff;

  float Vd_unlim = output->Vd;
  float Vq_unlim = output->Vq;

  // Step 7: 电压矢量限幅
  // 限制电压矢量在逆变器可输出的六边形内切圆内
  // V_max = (Vbus / sqrt(3)) * 0.95f (留出 5% 安全裕度)
  float V_max = (input->Vbus * 0.57735f) * 0.95f;
  float V_mag = sqrtf(output->Vd * output->Vd + output->Vq * output->Vq);

  if (V_mag > V_max) {
    float scale = V_max / V_mag;
    output->Vd *= scale;
    output->Vq *= scale;
    output->voltage_saturated = true;
  } else {
    output->voltage_saturated = false;
  }

  // 抗积分饱和更新 (Back-calculation anti-windup)
  // 当电压饱和时，停止积分或反向修正积分项，防止积分发散
  // Kb (Back-calculation gain) 通常取 1/Kp 或其他设计值
  float Kb = config->Kb_current;

  state->integral_d +=
      (config->Ki_current_d * error_d + Kb * (output->Vd - (Vd_raw + Vd_ff))) *
      dt;
  state->integral_q +=
      (config->Ki_current_q * error_q + Kb * (output->Vq - (Vq_raw + Vq_ff))) *
      dt;

  // Step 8: 逆Park变换 (dq → αβ)
  // 将旋转坐标系电压 (Vd, Vq) 变换回两相静止坐标系 (Valpha, Vbeta)
  Park_Inverse(output->Vd, output->Vq, input->theta_elec, &output->Valpha,
               &output->Vbeta);

  // Step 9: SVPWM调制 (αβ → abc PWM)
  // 使用空间矢量脉宽调制生成三相占空比 (Ta, Tb, Tc)
  // 包含扇区判断和矢量作用时间计算
  int ret = SVPWM_Modulate(output->Valpha, output->Vbeta, input->Vbus,
                           &output->Ta, &output->Tb, &output->Tc);

  output->overmodulation = (ret != 0);
}

void FOC_Algorithm_ResetState(FOC_AlgorithmState_t *state) {
  if (state == NULL)
    return;

  state->integral_d = 0.0f;
  state->integral_q = 0.0f;
  state->Id_filt = 0.0f;
  state->Iq_filt = 0.0f;
}

bool FOC_Algorithm_ValidateConfig(const FOC_AlgorithmConfig_t *config) {
  if (config == NULL)
    return false;

  // 检查电机参数
  if (config->Rs <= 0.0f || config->Rs > 100.0f)
    return false;
  if (config->Ls <= 0.0f || config->Ls > 1.0f)
    return false;
  if (config->flux < 0.0f || config->flux > 1.0f)
    return false;
  if (config->pole_pairs == 0 || config->pole_pairs > 50)
    return false;

  // 检查PI参数
  if (config->Kp_current_d < 0.0f || config->Ki_current_d < 0.0f)
    return false;
  if (config->Kp_current_q < 0.0f || config->Ki_current_q < 0.0f)
    return false;

  // 检查采样周期
  if (config->Ts_current <= 0.0f || config->Ts_current > 0.01f)
    return false;

  // 检查限制
  if (config->voltage_limit <= 0.0f)
    return false;
  if (config->current_limit <= 0.0f)
    return false;

  // 检查增益范围
  if (config->decoupling_gain < 0.0f || config->decoupling_gain > 1.0f)
    return false;

  return true;
}

void FOC_Algorithm_CalculateCurrentGains(float Rs, float Ls, float bandwidth,
                                         float *Kp, float *Ki) {
  if (Kp == NULL || Ki == NULL)
    return;

  // 电流环带宽设计 (内模控制 IMC 方法)
  float omega_c = 2.0f * MATH_PI * bandwidth;

  *Kp = Ls * omega_c;
  *Ki = Rs * omega_c;
}

/* Private function implementations */
