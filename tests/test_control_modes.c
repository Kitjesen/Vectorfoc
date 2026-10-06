/* Behavioral checks and deterministic per-cycle traces for every control mode.
 * --trace emits named, exact IEEE float bits, excluding padding and addresses. */
#ifdef CONTROL_TEST_LEGACY_LAYOUT
#include "control/control.h"
#include "config.h"
#else
#include "foc/control_dispatcher.h"
#include "motor_configuration.h"
#endif
#include "control_support/control_probe.h"
#include "error_manager.h"
#include "pwm_interface.h"
#include "observer_adapter.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Also declared here so this test can compile against the pre-refactor header. */
void Control_InjectVoltage(MOTOR_DATA *motor, float Vd, float Vq, float angle);

#define CHECK(condition) do { if (!(condition)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static unsigned pwm_calls, brake_calls, error_calls;
static float duty[3];
static bool trace_enabled;
static bool hal_available = true;

static void RecordDuty(float a, float b, float c) {
  duty[0] = a; duty[1] = b; duty[2] = c;
  ++pwm_calls;
  CHECK(isfinite(a) && a >= 0.0f && a <= 1.0f);
  CHECK(isfinite(b) && b >= 0.0f && b <= 1.0f);
  CHECK(isfinite(c) && c >= 0.0f && c <= 1.0f);
}
#ifdef CONTROL_TEST_LEGACY_HAL
static void RecordBrake(void) { ++brake_calls; }
#else
static int RecordBrake(void) { ++brake_calls; return 0; }
#endif
int MHAL_PWM_SetDuty(float a, float b, float c) {
  if (!hal_available) return -1;
  RecordDuty(a, b, c);
  return 0;
}
#ifndef CONTROL_TEST_LEGACY_HAL
int MHAL_PWM_Brake(void) {
  if (!hal_available) return -1;
  return RecordBrake();
}
#endif
void ErrorManager_ReportFull(uint32_t code, const char *message,
                             const char *file, uint32_t line) {
  CHECK(code == ERROR_MOTOR_ENCODER_LOSS);
  CHECK(message && file && line);
  ++error_calls;
}

#if defined(CONTROL_TEST_LEGACY_VTABLE) || defined(CONTROL_TEST_LEGACY_HAL)
static const Motor_HAL_PwmInterface_t pwm = {
  .set_duty = RecordDuty, .brake = RecordBrake
};
static const Motor_HAL_Handle_t hal = {.pwm = &pwm};
#endif

static void TraceFloat(const char *name, float value) {
  CHECK(isfinite(value));
  if (trace_enabled) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    printf(" %s=%08" PRIx32, name, bits);
  }
}
static void TraceInt(const char *name, unsigned value) {
  if (trace_enabled) printf(" %s=%u", name, value);
}
#define F(field) TraceFloat(#field, (field))
#define U(field) TraceInt(#field, (unsigned)(field))

static void TracePID(const char *name, const PidTypeDef *p) {
  if (trace_enabled) printf(" %s", name);
  U(p->mode); F(p->Kp); F(p->Ki); F(p->Kd); F(p->max_out); F(p->max_iout);
  F(p->out); F(p->out_sat); F(p->Pout); F(p->Iout); F(p->Dout);
  for (unsigned i = 0; i < 3; ++i) { F(p->Dbuf[i]); F(p->error[i]); }
  F(p->prevMeasure); F(p->tau); F(p->fdb); F(p->set);
}

static void TraceCycle(unsigned pass, unsigned cycle, const MOTOR_DATA *m,
                       const ControlTestSnapshot *s, const SMO_Observer_t *o) {
  if (trace_enabled) printf("cycle pass=%u mode=%d n=%u", pass, m->state.Control_Mode, cycle);
  F(m->Controller.input_velocity); F(m->Controller.input_torque);
  F(m->Controller.input_current); F(m->Controller.pos_setpoint);
  F(m->Controller.vel_setpoint); F(m->Controller.torque_setpoint);
  F(m->Controller.current_ctrl_p_gain); F(m->Controller.current_ctrl_i_gain);
  U(m->Controller.input_updated); U(m->params_updated);
  TracePID("IdPID", &m->IdPID); TracePID("IqPID", &m->IqPID);
  TracePID("VelPID", &m->VelPID); TracePID("PosPID", &m->PosPID);
  F(m->algo_input.Ia); F(m->algo_input.Ib); F(m->algo_input.Ic);
  F(m->algo_input.Vbus); F(m->algo_input.theta_elec); F(m->algo_input.omega_elec);
  F(m->algo_input.Id_ref); F(m->algo_input.Iq_ref); U(m->algo_input.enabled);
  F(m->algo_output.Ta); F(m->algo_output.Tb); F(m->algo_output.Tc);
  F(m->algo_output.Ialpha); F(m->algo_output.Ibeta); F(m->algo_output.Id); F(m->algo_output.Iq);
  F(m->algo_output.Vd); F(m->algo_output.Vq); F(m->algo_output.Valpha); F(m->algo_output.Vbeta);
  U(m->algo_output.overmodulation); U(m->algo_output.voltage_saturated); U(m->algo_output.current_limited);
  F(m->algo_state.integral_d); F(m->algo_state.integral_q);
  F(m->algo_state.Id_filt); F(m->algo_state.Iq_filt);
  F(m->algo_config.Rs); F(m->algo_config.Ls); F(m->algo_config.flux); U(m->algo_config.pole_pairs);
  F(m->algo_config.Kp_current_d); F(m->algo_config.Ki_current_d);
  F(m->algo_config.Kp_current_q); F(m->algo_config.Ki_current_q);
  F(m->algo_config.Ts_current); F(m->algo_config.voltage_limit); F(m->algo_config.current_limit);
  U(m->algo_config.enable_decoupling); F(m->algo_config.decoupling_gain);
  F(m->algo_config.Kb_current); F(m->algo_config.current_filter_fc);
  F(m->algo_config.deadtime_i_thresh); F(m->algo_config.deadtime_Vdiode);
  F(m->vel_feedback_filtered); U(m->vel_filter_initialized);
  F(m->ladrc_state.z1); F(m->ladrc_state.z2); F(m->ladrc_state.output);
  F(m->ladrc_state.beta1); F(m->ladrc_state.beta2); F(m->ladrc_state.kp); U(m->ladrc_state.initialized);
  U(s->velocity_loop_count); U(s->position_loop_count); F(s->vel_set);
  F(s->traj.Y); F(s->traj.Yd); F(s->traj.Ydd); F(s->traj.Tf_); F(s->traj.t);
  U(s->traj.trajectory_done); F(s->traj.Xi_); F(s->traj.Xf_); F(s->traj.Vi_);
  F(s->traj.Ar_); F(s->traj.Vr_); F(s->traj.Dr_); F(s->traj.Ta_);
  F(s->traj.Tv_); F(s->traj.Td_); F(s->traj.yAccel_);
  F(s->velocity.last_value); F(s->velocity.max_rate); U(s->velocity.initialized);
  F(s->torque.last_value); F(s->torque.max_rate); U(s->torque.initialized);
  F(o->alpha); F(o->beta); F(o->est_i_alpha); F(o->est_i_beta);
  F(o->est_bemf_alpha); F(o->est_bemf_beta); F(o->est_angle); F(o->est_velocity);
  F(o->pll_angle); F(o->pll_velocity); F(o->pll_kp); F(o->pll_ki);
  F(m->feedback.observer_angle); F(m->feedback.observer_velocity);
  F(duty[0]); F(duty[1]); F(duty[2]); U(pwm_calls); U(brake_calls); U(error_calls);
  if (trace_enabled) putchar('\n');
}

static void InitMotor(MOTOR_DATA *m) {
  memset(m, 0, sizeof(*m));
#if defined(CONTROL_TEST_LEGACY_VTABLE) || defined(CONTROL_TEST_LEGACY_HAL)
  m->components.hal = &hal;
#endif
  hal_available = true;
  m->state.State_Mode = STATE_MODE_RUNNING;
  m->parameters.Rs = 0.3f; m->parameters.Ls = 0.0004f;
  m->parameters.flux = 0.002f; m->parameters.pole_pairs = 7;
  m->Controller.inertia = 0.012f;
  m->Controller.torque_ramp_rate = 30.0f; m->Controller.vel_ramp_rate = 40.0f;
  m->Controller.traj_vel = 2.0f; m->Controller.traj_accel = 100.0f;
  m->Controller.traj_decel = 80.0f; m->Controller.vel_limit = 20.0f;
  m->Controller.torque_const = 0.12f; m->Controller.torque_limit = 1.5f;
  m->Controller.current_limit = 8.0f; m->Controller.voltage_limit = 12.0f;
  const float current_gains[] = {0.4f, 3.0f, 0.0f};
  const float velocity_gains[] = {2.0f, 0.4f, 0.001f};
  const float position_gains[] = {5.0f, 0.2f, 0.0001f};
  PID_Init(&m->IdPID, PID_POSITION, current_gains, 12.0f, 12.0f);
  PID_Init(&m->IqPID, PID_POSITION, current_gains, 12.0f, 12.0f);
  PID_Init(&m->VelPID, PID_POSITION, velocity_gains, 8.0f, 8.0f);
  PID_Init(&m->PosPID, PID_POSITION, position_gains, 20.0f, 20.0f);
  m->ladrc_config = (LADRC_Config_t){120.0f, 30.0f, 25.0f, 8.0f};
  LADRC_Init(&m->ladrc_state, &m->ladrc_config);
  m->algo_config.decoupling_gain = 0.7f;
  m->algo_config.Kb_current = 2.0f; m->algo_config.current_filter_fc = 1000.0f;
  m->advanced.fw_max_current = 0.8f; m->advanced.fw_start_velocity = 6.0f;
  m->params_updated = true;
  Control_UpdateCurrentGains(m);
  Control_Initialize(m);
  m->algo_state.integral_d = 0.125f;
  Control_Initialize(m);
  CHECK(m->algo_state.integral_d == 0.125f); /* Reinitialization is a no-op. */
}

static void CheckConfiguredOuterLoopTiming(void) {
  CHECK(FOC_LOOP_FREQUENCY_HZ == 20000u);
  CHECK(CURRENT_SAMPLE_FREQUENCY_HZ == FOC_LOOP_FREQUENCY_HZ);
  CHECK(COMMAND_RAMP_FREQUENCY_HZ == FOC_LOOP_FREQUENCY_HZ);
  CHECK(VELOCITY_LOOP_DECIMATION == 4u);
  CHECK(POSITION_LOOP_DECIMATION == 20u);
  CHECK(fabsf(VELOCITY_LOOP_PERIOD_S - 0.0002f) < 1e-9f);
  CHECK(fabsf(POSITION_LOOP_PERIOD_S - 0.001f) < 1e-9f);

  MOTOR_DATA motor;
  InitMotor(&motor);
  motor.algo_input.Vbus = 24.0f;
  motor.algo_input.enabled = true;
  motor.feedback.phase_angle = 0.0f;
  motor.feedback.velocity = 0.0f;
  motor.feedback.position = 0.0f;
  motor.Controller.vel_limit = 100.0f;
  motor.Controller.current_limit = 100.0f;
  motor.Controller.torque_limit = 100.0f;

  const float integral_only[] = {0.0f, 1.0f, 0.0f};
  PID_Init(&motor.VelPID, PID_POSITION, integral_only, 100.0f, 100.0f);
  motor.state.Control_Mode = CONTROL_MODE_VELOCITY;
  motor.Controller.input_velocity = 1.0f;
  for (unsigned cycle = 1; cycle <= 20; ++cycle) {
    float before = motor.VelPID.Iout;
    Control_RunCurrentCycle(&motor);
    float increment = motor.VelPID.Iout - before;
    if ((cycle % 4u) == 0u) {
      CHECK(fabsf(increment - 0.0002f) < 1e-8f);
    } else {
      CHECK(increment == 0.0f);
    }
  }
  CHECK(fabsf(motor.VelPID.Iout - 0.001f) < 1e-8f);

  PID_Init(&motor.PosPID, PID_POSITION, integral_only, 100.0f, 100.0f);
  motor.state.Control_Mode = CONTROL_MODE_POSITION;
  motor.Controller.input_position = 1.0f;
  motor.Controller.input_velocity = 100.0f;
  for (unsigned cycle = 1; cycle <= 20; ++cycle) {
    float before = motor.PosPID.Iout;
    Control_RunCurrentCycle(&motor);
    float increment = motor.PosPID.Iout - before;
    if (cycle == 20u) {
      CHECK(fabsf(increment - 0.001f) < 1e-8f);
    } else {
      CHECK(increment == 0.0f);
    }
  }
  CHECK(fabsf(motor.PosPID.Iout - 0.001f) < 1e-8f);
}

int main(int argc, char **argv) {
  trace_enabled = argc == 2 && strcmp(argv[1], "--trace") == 0;
  CHECK(argc == 1 || trace_enabled);
  /* Keep the historical --trace input/state sequence bit-for-bit comparable. */
  if (!trace_enabled) {
    CheckConfiguredOuterLoopTiming();
  }
  MOTOR_DATA motor;
  SMO_Observer_t observer;
  InitMotor(&motor);
  SMO_Observer_Init(&observer);
  SMO_Observer_Init(NULL); SMO_Observer_Update(NULL, &motor);
  SMO_Observer_Update(&observer, NULL);
  observer.alpha = 0.15f; observer.beta = 0.02f;
  const CONTROL_MODE modes[] = {CONTROL_MODE_OPEN, CONTROL_MODE_TORQUE,
      CONTROL_MODE_VELOCITY, CONTROL_MODE_POSITION, CONTROL_MODE_VELOCITY_RAMP,
      CONTROL_MODE_POSITION_RAMP, CONTROL_MODE_MIT, CONTROL_MODE_VF, CONTROL_MODE_IF,
      (CONTROL_MODE)7}; /* 7 is intentionally invalid. Preserve error behavior. */
  unsigned cycles = 0, completed_trajectories = 0, paused_trajectories = 0;
  unsigned saturation_cycles = 0, limited_cycles = 0;
  for (unsigned pass = 0; pass < 2; ++pass) {
    motor.ladrc_enable = (float)pass;
    for (unsigned mode_index = 0; mode_index < sizeof(modes) / sizeof(modes[0]); ++mode_index) {
      CONTROL_MODE mode = modes[mode_index];
      motor.state.Control_Mode = mode;
      unsigned count = mode == CONTROL_MODE_POSITION_RAMP ? 520 : 193 + mode_index;
      for (unsigned i = 0; i < count; ++i) {
        hal_available = !(i == 91 && mode != CONTROL_MODE_OPEN && mode != CONTROL_MODE_VF);
#if defined(CONTROL_TEST_LEGACY_VTABLE) || defined(CONTROL_TEST_LEGACY_HAL)
        motor.components.hal = hal_available ? &hal : NULL;
#endif
        motor.Controller.input_velocity = (i < 100 ? 30.0f : -25.0f);
        motor.Controller.input_torque = (i < 100 ? 2.0f : -2.0f);
        motor.Controller.input_current = (i < 100 ? 10.0f : -10.0f);
        if (pass == 0 && mode_index == 0 && i == 0) {
          motor.Controller.input_velocity = 0.0f;
          motor.Controller.input_torque = 0.0f;
        }
        if (mode == CONTROL_MODE_IF && i >= 40 && i < 80) motor.Controller.input_torque = 0.0f;
        motor.Controller.current_limit = i < 90 ? 8.0f : 0.4f;
        motor.Controller.torque_limit = i < 90 ? 1.5f : 0.02f;
        motor.Controller.input_position = (i < 260 ? 0.002f : -0.002f);
        motor.Controller.input_updated = (i == 0 || i == 260);
        motor.feedback.position = 0.0001f * sinf((float)i * 0.13f);
        motor.feedback.velocity = 9.0f * sinf((float)i * 0.07f);
        if (mode == CONTROL_MODE_POSITION_RAMP) {
          motor.feedback.position = motor.Controller.pos_setpoint;
          motor.feedback.velocity = 0.0f;
          if (i >= 32 && i < 40) motor.feedback.position += 2.0f;
        }
        motor.feedback.phase_angle = -3.0f + (float)(i % 61) * 0.1f;
        motor.algo_input.Ia = 0.45f * sinf((float)i * 0.17f);
        motor.algo_input.Ib = 0.25f * cosf((float)i * 0.23f);
        motor.algo_input.Ic = -motor.algo_input.Ia - motor.algo_input.Ib;
        motor.algo_input.Vbus = i == 80 ? 0.5f : (i % 3 == 0 ? 6.0f : 24.0f);
        motor.Controller.mit_kp = i < 20 ? -1.0f : 4.0f;
        motor.Controller.mit_kd = 0.4f;
        motor.Controller.mit_pos_des = motor.feedback.position * M_2PI + (i < 40 ? 20.0f : 0.15f);
        motor.Controller.mit_vel_des = motor.feedback.velocity * M_2PI + 0.2f;
        if (i == 0 || i == 3 || i == 77 || i == 90) motor.params_updated = true;
        motor.algo_config.deadtime_i_thresh = i < 90 ? 0.0f : 0.15f;
        motor.algo_config.deadtime_Vdiode = i < 90 ? 0.0f : 0.6f;
        ControlTestSnapshot before, after;
        ControlTest_ReadSnapshot(&before);
        FOC_AlgorithmState_t previous_state = motor.algo_state;
        float previous_iq = motor.algo_input.Iq_ref;
        unsigned previous_pwm_calls = pwm_calls;
        unsigned previous_brakes = brake_calls, previous_errors = error_calls;
        Control_RunCurrentCycle(&motor);
        ControlTest_ReadSnapshot(&after);
        bool velocity_loop = mode >= CONTROL_MODE_VELOCITY && mode <= CONTROL_MODE_POSITION_RAMP;
        bool position_loop = mode == CONTROL_MODE_POSITION || mode == CONTROL_MODE_POSITION_RAMP;
        CHECK(after.velocity_loop_count ==
              (velocity_loop ? (before.velocity_loop_count + 1) % VELOCITY_LOOP_DECIMATION : 0));
        CHECK(after.position_loop_count ==
              (position_loop ? (before.position_loop_count + 1) % POSITION_LOOP_DECIMATION : 0));
        CHECK(pwm_calls == previous_pwm_calls + (hal_available ? 1 : 0));
        bool voltage_only = mode == CONTROL_MODE_OPEN || mode == CONTROL_MODE_VF;
        if (voltage_only) CHECK(memcmp(&previous_state, &motor.algo_state, sizeof(previous_state)) == 0);
        else CHECK(motor.algo_input.enabled);
        if (voltage_only || mode == CONTROL_MODE_IF) {
          CHECK(motor.algo_input.theta_elec >= 0.0f && motor.algo_input.theta_elec < M_2PI);
        } else if (mode != (CONTROL_MODE)7) CHECK(motor.algo_input.theta_elec == motor.feedback.phase_angle);
        if (mode == (CONTROL_MODE)7) {
          CHECK(error_calls == previous_errors + 1);
          CHECK(brake_calls == previous_brakes + (hal_available ? 1 : 0));
        } else CHECK(error_calls == previous_errors && brake_calls == previous_brakes);
        if (mode == CONTROL_MODE_MIT && i < 20) CHECK(motor.algo_input.Iq_ref == 0.0f);
        if (mode == CONTROL_MODE_MIT && i >= 20 && i < 40) CHECK(motor.algo_input.Iq_ref == previous_iq * MIT_MODE_DECAY_FACTOR);
        if (mode == CONTROL_MODE_POSITION_RAMP) {
          CHECK(!motor.Controller.input_updated);
          if (after.traj.trajectory_done) ++completed_trajectories;
          if (i >= 32 && i < 40 && before.traj.t == after.traj.t) ++paused_trajectories;
        }
        saturation_cycles += motor.algo_output.voltage_saturated;
        limited_cycles += motor.algo_output.current_limited;
        SMO_Observer_Update(&observer, &motor);
        TraceCycle(pass, i, &motor, &after, &observer);
        ++cycles;
      }
    }
  }
  CHECK(completed_trajectories > 0 && paused_trajectories == 16);
  CHECK(saturation_cycles > 0 && limited_cycles > 0);
  motor.algo_input.Vbus = 0.0f;
  Control_InjectVoltage(&motor, 0.3f, -0.7f, 1.2f);
  CHECK(motor.algo_output.Vd == 0.3f && motor.algo_output.Vq == -0.7f);
  CHECK(motor.algo_input.theta_elec == 1.2f);
  F(motor.algo_output.Ta); F(motor.algo_output.Tb); F(motor.algo_output.Tc);
  if (trace_enabled) putchar('\n');
  printf("PASS cycles=%u modes=9 invalid_mode=1 outer=PID,LADRC trajectory_done=%u trajectory_paused=%u saturated=%u limited=%u\n",
         cycles, completed_trajectories, paused_trajectories, saturation_cycles, limited_cycles);
  return 0;
}
