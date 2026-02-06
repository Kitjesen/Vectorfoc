#include "cogging.h"
#include "config.h"
#include <math.h>
#include <string.h>

typedef struct {
  bool active;
  uint16_t index;
  uint16_t hold_count;
  CONTROL_MODE prev_mode;
  float prev_input_velocity;
  float base_pos;
} CoggingCalibState_t;

static float s_cogging_map[COGGING_MAP_SIZE];
static bool s_map_valid = false;
static float s_phase_offset = 0.0f;
static CoggingCalibState_t s_calib = {0};

static float Wrap01(float x) {
  x -= floorf(x);
  if (x < 0.0f) {
    x += 1.0f;
  }
  return x;
}

static float CoggingComp_Lookup(float phase) {
  float idx_f = phase * (float)COGGING_MAP_SIZE;
  int idx0 = (int)idx_f;
  float frac = idx_f - (float)idx0;

  if (idx0 < 0) {
    idx0 = 0;
    frac = 0.0f;
  } else if (idx0 >= COGGING_MAP_SIZE) {
    idx0 = COGGING_MAP_SIZE - 1;
    frac = 0.0f;
  }

  int idx1 = idx0 + 1;
  if (idx1 >= COGGING_MAP_SIZE) {
    idx1 = 0;
  }

  float a = s_cogging_map[idx0];
  float b = s_cogging_map[idx1];
  return a + (b - a) * frac;
}

static void CoggingCalib_Finish(MOTOR_DATA *motor, bool success) {
  s_calib.active = false;
  s_calib.index = 0;
  s_calib.hold_count = 0;
  if (success) {
    s_map_valid = true;
  }

  if (motor) {
    motor->state.Control_Mode = s_calib.prev_mode;
    motor->Controller.input_velocity = s_calib.prev_input_velocity;
    motor->Controller.input_position = motor->feedback.position;
    motor->Controller.input_updated = true;
  }
}

void CoggingComp_StartCalibration(MOTOR_DATA *motor) {
  if (!motor) {
    return;
  }
  if (s_calib.active) {
    return;
  }

  memset(s_cogging_map, 0, sizeof(s_cogging_map));
  s_map_valid = false;

  s_calib.active = true;
  s_calib.index = 0;
  s_calib.hold_count = 0;
  s_calib.prev_mode = motor->state.Control_Mode;
  s_calib.prev_input_velocity = motor->Controller.input_velocity;
  s_calib.base_pos = motor->feedback.position;

  s_phase_offset = Wrap01(motor->feedback.position);
}

void CoggingComp_StopCalibration(MOTOR_DATA *motor) {
  if (!s_calib.active) {
    return;
  }
  CoggingCalib_Finish(motor, false);
}

bool CoggingComp_IsCalibrating(void) { return s_calib.active; }

bool CoggingComp_IsValid(void) { return s_map_valid; }

float CoggingComp_GetCurrent(const MOTOR_DATA *motor) {
  if (!motor) {
    return 0.0f;
  }
  if (!s_map_valid || s_calib.active) {
    return 0.0f;
  }
  if (motor->advanced.cogging_comp_enabled <= 0.5f) {
    return 0.0f;
  }

  float phase = Wrap01(motor->feedback.position - s_phase_offset);
  return CoggingComp_Lookup(phase);
}

void CoggingComp_Update(MOTOR_DATA *motor) {
  if (!motor || !s_calib.active) {
    return;
  }

  if (motor->state.State_Mode != STATE_MODE_RUNNING) {
    CoggingCalib_Finish(motor, false);
    return;
  }

  motor->state.Control_Mode = CONTROL_MODE_POSITION;
  motor->Controller.input_velocity = COGGING_CALIB_VEL_LIMIT;

  float step = 1.0f / (float)COGGING_MAP_SIZE;
  float target = s_calib.base_pos + (float)s_calib.index * step;
  motor->Controller.input_position = target;
  motor->Controller.input_updated = true;

  float pos_err = motor->feedback.position - target;
  if (fabsf(pos_err) <= COGGING_CALIB_POS_THRESH_TURN &&
      fabsf(motor->feedback.velocity) <= COGGING_CALIB_VEL_THRESH_TURN_S) {
    s_calib.hold_count++;
    if (s_calib.hold_count >= COGGING_CALIB_HOLD_CYCLES) {
      float iq_cmd = motor->algo_input.Iq_ref;
      float iq_clamped =
          CLAMP(iq_cmd, -motor->Controller.current_limit,
                motor->Controller.current_limit);

      if (s_calib.index < COGGING_MAP_SIZE) {
        s_cogging_map[s_calib.index] = iq_clamped;
      }

      s_calib.index++;
      s_calib.hold_count = 0;

      if (s_calib.index >= COGGING_MAP_SIZE) {
        CoggingCalib_Finish(motor, true);
      }
    }
  } else {
    s_calib.hold_count = 0;
  }
}
