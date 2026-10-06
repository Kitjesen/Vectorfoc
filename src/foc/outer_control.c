// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "control_context.h"
#include "motor_configuration.h"

#define VEL_FEEDBACK_FILTER_FC 100.0f

static inline float Control_CalculateVelocityOutput(const OuterLoopInput *input,
                                                    float vel_ref, float vel_fdb) {
  if (input->ladrc_enable >= 0.5f) {
    if (input->params_updated) {
      LADRC_UpdateGains(input->ladrc_state, input->ladrc_config);
    }
    return LADRC_Calc(input->ladrc_state, input->ladrc_config, vel_ref,
                      vel_fdb, input->ladrc_state->output, VELOCITY_LOOP_PERIOD_S);
  }
  return PID_CalcDt(input->velocity_pid, vel_fdb, vel_ref, VELOCITY_LOOP_PERIOD_S);
}

void Control_UpdatePositionLoop(const OuterLoopInput *input, MotorControlCtx *ctx) {
  if (input == NULL || ctx == NULL) {
    return;
  }

  if (input->mode == CONTROL_MODE_POSITION) {
    ctx->vel_set = PID_CalcDt(input->position_pid, input->feedback_position,
                              input->input_position, POSITION_LOOP_PERIOD_S);
    ctx->vel_set = CLAMP(ctx->vel_set, -input->input_velocity,
                         +input->input_velocity);
  } else if (input->mode == CONTROL_MODE_POSITION_RAMP) {
    ctx->vel_set = PID_CalcDt(input->position_pid, input->feedback_position,
                              input->pos_setpoint, POSITION_LOOP_PERIOD_S) +
                   input->vel_setpoint;
    ctx->vel_set = CLAMP(ctx->vel_set, -input->input_velocity,
                         +input->input_velocity);
  }
}

void Control_UpdateVelocityLoop(const OuterLoopInput *input,
                                OuterLoopOutput *output, MotorControlCtx *ctx) {
  if (input == NULL || output == NULL || ctx == NULL) {
    return;
  }

  float vel_feedback_filtered = input->vel_feedback_filtered;
  bool vel_filter_initialized = input->vel_filter_initialized;
  if (!vel_filter_initialized) {
    vel_feedback_filtered = input->feedback_velocity;
    vel_filter_initialized = true;
  }
  float alpha_vel = (M_2PI * VEL_FEEDBACK_FILTER_FC * VELOCITY_LOOP_PERIOD_S) /
                    (1.0f + M_2PI * VEL_FEEDBACK_FILTER_FC * VELOCITY_LOOP_PERIOD_S);
  vel_feedback_filtered +=
      alpha_vel * (input->feedback_velocity - vel_feedback_filtered);
  float vel_fdb = vel_feedback_filtered;

  if (input->mode >= CONTROL_MODE_VELOCITY &&
      input->mode <= CONTROL_MODE_POSITION) {
    if (input->mode == CONTROL_MODE_VELOCITY) {
      ctx->vel_set = input->input_velocity;
      ctx->vel_set = CLAMP(ctx->vel_set, -input->vel_limit,
                           +input->vel_limit);
    }
    output->iq_ref = Control_CalculateVelocityOutput(input, ctx->vel_set, vel_fdb);
    output->id_ref = 0.0f;
  } else if (input->mode >= CONTROL_MODE_VELOCITY_RAMP &&
             input->mode <= CONTROL_MODE_POSITION_RAMP) {
    if (input->mode == CONTROL_MODE_VELOCITY_RAMP) {
      ctx->vel_set = input->vel_setpoint;
      ctx->vel_set = CLAMP(ctx->vel_set, -input->vel_limit,
                           +input->vel_limit);
    }
    vel_fdb = CLAMP(vel_fdb, -input->vel_limit, +input->vel_limit);
    output->iq_ref = Control_CalculateVelocityOutput(input, ctx->vel_set, vel_fdb) +
                     input->torque_setpoint;
    output->id_ref = 0.0f;
  }

  output->vel_feedback_filtered = vel_feedback_filtered;
  output->vel_filter_initialized = vel_filter_initialized;
}
