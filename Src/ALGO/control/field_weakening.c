#include "field_weakening.h"
#include "common.h"

/** Field-weakening controller state. */
static float s_id_fw_integral = 0.0f;

/**
 * @brief Calculate linear field-weakening current from speed.
 */
static float FieldWeakening_CalcIdRef_Linear(const FieldWeakening_Config_t *cfg,
                                             float velocity) {
  if (cfg == NULL || cfg->max_weakening_current <= 0.0f ||
      cfg->start_velocity <= 0.0f) {
    return 0.0f;
  }

  float abs_vel = fabsf(velocity);
  if (abs_vel <= cfg->start_velocity) {
    return 0.0f;
  }

  float ratio = (abs_vel - cfg->start_velocity) / cfg->start_velocity;
  ratio = CLAMP(ratio, 0.0f, 1.0f);
  return -cfg->max_weakening_current * ratio;
}

/**
 * @brief Calculate field-weakening current from voltage saturation.
 */
static float FieldWeakening_CalcIdRef_VoltSat(MOTOR_DATA *motor,
                                              const FieldWeakening_Config_t *cfg,
                                              float dt) {
  if (motor == NULL || cfg == NULL || cfg->max_weakening_current <= 0.0f) {
    return 0.0f;
  }

  if (dt <= 0.0f || dt > 0.1f) {
    return s_id_fw_integral;
  }

  // Weakening current ramp rate [A/s].
  float fw_ramp_rate = 100.0f;

  // Ramp negative Id only while voltage saturation is active.
  if (motor->algo_output.voltage_saturated) {
    s_id_fw_integral -= fw_ramp_rate * dt;
  } else if (s_id_fw_integral < 0.0f) {
    // Recover back toward zero Id when saturation clears.
    s_id_fw_integral += fw_ramp_rate * 0.1f * dt;
    if (s_id_fw_integral > 0.0f) {
      s_id_fw_integral = 0.0f;
    }
  }

  s_id_fw_integral = CLAMP(s_id_fw_integral, -cfg->max_weakening_current, 0.0f);
  return s_id_fw_integral;
}

void FieldWeakening_Update(MOTOR_DATA *motor,
                           const FieldWeakening_Config_t *cfg) {
  if (motor == NULL || cfg == NULL) {
    return;
  }

  float id_fw_linear =
      FieldWeakening_CalcIdRef_Linear(cfg, motor->feedback.velocity);
  float id_fw_voltsat =
      FieldWeakening_CalcIdRef_VoltSat(motor, cfg, 0.0002f); // 5 kHz

  // Use the more aggressive of the two weakening requests.
  float id_fw = (id_fw_linear < id_fw_voltsat) ? id_fw_linear : id_fw_voltsat;

  if (id_fw == 0.0f) {
    return;
  }

  float id_ref = motor->algo_input.Id_ref + id_fw;
  motor->algo_input.Id_ref =
      CLAMP(id_ref, -cfg->max_weakening_current, cfg->max_weakening_current);
}

/**
 * @brief Reset field-weakening controller state.
 */
void FieldWeakening_Reset(void) {
  s_id_fw_integral = 0.0f;
}
