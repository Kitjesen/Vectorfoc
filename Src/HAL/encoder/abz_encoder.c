#include "abz_encoder.h"

#ifdef BOARD_XSTAR

#include "board_config_xstar.h"
#include "hal_abstraction.h"
#include <math.h>
#include <string.h>

#define ABZ_2PI 6.28318530717959f

static float Abz_NormalizeAngle(float angle) {
  while (angle >= ABZ_2PI) {
    angle -= ABZ_2PI;
  }
  while (angle < 0.0f) {
    angle += ABZ_2PI;
  }
  return angle;
}

static int32_t Abz_WrapDelta(int32_t delta, int32_t modulo) {
  int32_t half = modulo / 2;
  if (delta > half) {
    delta -= modulo;
  } else if (delta < -half) {
    delta += modulo;
  }
  return delta;
}

static bool Abz_ReadIndexPin(void) {
  return HAL_GPIO_ReadPin(HW_ABZ_Z_PORT, HW_ABZ_Z_PIN) == GPIO_PIN_SET;
}

Abz_Handle_t abz_data = {
    .cpr = HW_ABZ_CPR,
    .pole_pairs = 1,
    .direction = true,
    .calib_valid = true,
};

void Abz_Init(void) {
  float saved_offset = abz_data.offset_rad;
  uint8_t saved_pole_pairs = abz_data.pole_pairs == 0u ? 1u : abz_data.pole_pairs;
  bool saved_calib_valid = abz_data.calib_valid;

  memset(&abz_data, 0, sizeof(abz_data));
  abz_data.cpr = HW_ABZ_CPR;
  abz_data.offset_rad = saved_offset;
  abz_data.pole_pairs = saved_pole_pairs;
  abz_data.calib_valid = saved_calib_valid;
  abz_data.offset_counts =
      (int32_t)lroundf((saved_offset / ABZ_2PI) * (float)abz_data.cpr);

  __HAL_TIM_SET_COUNTER(&HW_ABZ_TIMER, 0);
  HAL_TIM_Encoder_Start(&HW_ABZ_TIMER, TIM_CHANNEL_ALL);

  abz_data.raw_count = (int32_t)__HAL_TIM_GET_COUNTER(&HW_ABZ_TIMER);
  abz_data.count_in_cpr = abz_data.raw_count;
  abz_data.last_count = abz_data.raw_count;
  abz_data.last_update_us = HAL_GetMicroseconds();
  abz_data.last_z_state = Abz_ReadIndexPin();
}

void Abz_SetPolePairs(uint8_t pp) {
  if (pp > 0u) {
    abz_data.pole_pairs = pp;
  }
}

void Abz_ResetCount(void) {
  __HAL_TIM_SET_COUNTER(&HW_ABZ_TIMER, 0);
  abz_data.raw_count = 0;
  abz_data.count_in_cpr = 0;
  abz_data.last_count = 0;
  abz_data.shadow_count = 0;
  abz_data.mec_angle_rad = 0.0f;
  abz_data.elec_angle_rad = Abz_NormalizeAngle(abz_data.offset_rad);
  abz_data.velocity_rad_s = 0.0f;
  abz_data.vel_estimate_ = 0.0f;
  abz_data.last_update_us = HAL_GetMicroseconds();
}

static void Abz_HAL_Update(void) {
  uint32_t now_us = HAL_GetMicroseconds();
  uint32_t elapsed_us = now_us - abz_data.last_update_us;
  int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(&HW_ABZ_TIMER);
  int32_t delta = Abz_WrapDelta(current - abz_data.last_count, (int32_t)abz_data.cpr);
  int32_t mechanical_counts = current - abz_data.offset_counts;

  if (mechanical_counts < 0) {
    mechanical_counts %= (int32_t)abz_data.cpr;
    mechanical_counts += (int32_t)abz_data.cpr;
  } else {
    mechanical_counts %= (int32_t)abz_data.cpr;
  }

  abz_data.raw_count = current;
  abz_data.count_in_cpr = current;
  abz_data.shadow_count += delta;
  abz_data.last_count = current;
  abz_data.direction = (delta >= 0);

  if (elapsed_us > 0u) {
    float dt = (float)elapsed_us * 1e-6f;
    abz_data.velocity_rad_s =
        ((float)delta * ABZ_2PI) / ((float)abz_data.cpr * dt);
    abz_data.vel_estimate_ = abz_data.velocity_rad_s / ABZ_2PI;
  } else {
    abz_data.velocity_rad_s = 0.0f;
    abz_data.vel_estimate_ = 0.0f;
  }

  abz_data.mec_angle_rad =
      Abz_NormalizeAngle(((float)mechanical_counts * ABZ_2PI) / (float)abz_data.cpr);
  abz_data.elec_angle_rad =
      Abz_NormalizeAngle(abz_data.mec_angle_rad * (float)abz_data.pole_pairs);

  {
    bool z_state = Abz_ReadIndexPin();
    if (z_state && !abz_data.last_z_state) {
      abz_data.index_seen = true;
    }
    abz_data.last_z_state = z_state;
  }

  abz_data.last_update_us = now_us;
}

static void Abz_HAL_GetData(Motor_HAL_EncoderData_t *data) {
  data->angle_rad = abz_data.mec_angle_rad;
  data->velocity_rad = abz_data.velocity_rad_s;
  data->elec_angle = abz_data.elec_angle_rad;
  data->raw_value = abz_data.raw_count;
}

static void Abz_HAL_SetOffset(float offset) {
  abz_data.offset_rad = offset;
  abz_data.offset_counts =
      (int32_t)lroundf((offset / ABZ_2PI) * (float)abz_data.cpr);
}

const Motor_HAL_EncoderInterface_t g_abz_encoder_interface = {
    .update = Abz_HAL_Update,
    .get_data = Abz_HAL_GetData,
    .set_offset = Abz_HAL_SetOffset,
};

#endif /* BOARD_XSTAR */
