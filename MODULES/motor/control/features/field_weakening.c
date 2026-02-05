#include "field_weakening.h"
#include "common.h"

static float FieldWeakening_CalcIdRef(const FieldWeakening_Config_t *cfg,
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

void FieldWeakening_Update(MOTOR_DATA *motor,
                           const FieldWeakening_Config_t *cfg) {
  if (motor == NULL || cfg == NULL) {
    return;
  }

  float id_fw = FieldWeakening_CalcIdRef(cfg, motor->feedback.velocity);
  if (id_fw == 0.0f) {
    return;
  }

  float id_ref = motor->algo_input.Id_ref + id_fw;
  motor->algo_input.Id_ref =
      CLAMP(id_ref, -cfg->max_weakening_current, cfg->max_weakening_current);
}
