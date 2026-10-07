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

/**
 * Hardware-independent FOC current loop. The caller supplies sampled currents,
 * electrical angle, setpoints and configuration, then applies the returned PWM.
 * State is explicit: one FOC_AlgorithmState_t per controlled motor.
 */
#ifndef FOC_ALGORITHM_H
#define FOC_ALGORITHM_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Phase-current feedback and d/q current commands. */
typedef struct {
  float Ia;         /* Phase A current [A]. */
  float Ib;         /* Phase B current [A]. */
  float Ic;         /* Phase C current [A]. */
  float Vbus;       /* DC bus voltage [V]. */
  float theta_elec; /* Electrical angle [rad]. */
  float omega_elec; /* Electrical angular velocity [rad/s]. */
  float Id_ref;     /* D-axis current command [A]. */
  float Iq_ref;     /* Q-axis current command [A]. */
  bool enabled;
} FOC_AlgorithmInput_t;

/* PWM output plus intermediate values available to telemetry/observers. */
typedef struct {
  float Ta;     /* Phase A duty [0, 1]. */
  float Tb;     /* Phase B duty [0, 1]. */
  float Tc;     /* Phase C duty [0, 1]. */
  float Ialpha; /* Alpha-axis current [A]. */
  float Ibeta;  /* Beta-axis current [A]. */
  float Id;     /* D-axis current [A]. */
  float Iq;     /* Q-axis current [A]. */
  float Vd;     /* D-axis voltage [V]. */
  float Vq;     /* Q-axis voltage [V]. */
  float Valpha; /* Alpha-axis voltage [V]. */
  float Vbeta;  /* Beta-axis voltage [V]. */
  bool overmodulation;    /* SVPWM returned a nonzero status. */
  bool voltage_saturated;
  bool current_limited;
} FOC_AlgorithmOutput_t;

/* Gains and limits are supplied by the motor/control layer. */
typedef struct {
  float Rs;           /* Phase resistance [ohm]. */
  float Ls;           /* Phase inductance [H]. */
  float flux;         /* Flux linkage [Wb]. */
  uint8_t pole_pairs;
  float Kp_current_d;  /* D-axis proportional gain. */
  float Ki_current_d;  /* D-axis integral gain per second. */
  float Kp_current_q;  /* Q-axis proportional gain. */
  float Ki_current_q;  /* Q-axis integral gain per second. */
  float Ts_current;    /* Current-loop sample period [s]. */
  float voltage_limit; /* Configured voltage limit [V]. */
  float current_limit; /* Current-vector magnitude limit [A]. */
  bool enable_decoupling;
  float decoupling_gain;  /* Feedforward scale [0, 1]. */
  float Kb_current;        /* Anti-windup back-calculation gain [1/s]. */
  float current_filter_fc; /* Current filter cutoff [Hz]; <= 0 disables it. */
  /* Consumed by the control layer's PWM deadtime compensation. */
  float deadtime_i_thresh; /* Current zero-crossing interpolation threshold [A]. */
  float deadtime_Vdiode;   /* Body-diode voltage drop [V]. */
} FOC_AlgorithmConfig_t;

typedef struct {
  float integral_d; /* D-axis PI integral contribution [V]. */
  float integral_q; /* Q-axis PI integral contribution [V]. */
  float Id_filt;    /* Filtered D-axis current [A]. */
  float Iq_filt;    /* Filtered Q-axis current [A]. */
} FOC_AlgorithmState_t;

void FOC_Algorithm_InitState(FOC_AlgorithmState_t *state);

/* One sample: Clarke/Park -> filter -> PI/decoupling -> voltage limiting ->
 * inverse Park -> SVPWM. No hardware access or scheduling occurs here. */
void FOC_Algorithm_CurrentLoop(const FOC_AlgorithmInput_t *input,
                               const FOC_AlgorithmConfig_t *config,
                               FOC_AlgorithmState_t *state,
                               FOC_AlgorithmOutput_t *output);

/* Clear current integrators and filters on the caller's chosen transition. */
void FOC_Algorithm_ResetState(FOC_AlgorithmState_t *state);

/* Validate configured limits and electrical parameters before use. */
bool FOC_Algorithm_ValidateConfig(const FOC_AlgorithmConfig_t *config);
/* Derive current PI gains from an electrical bandwidth expressed in Hz. */
void FOC_Algorithm_CalculateCurrentGains(float Rs, float Ls, float bw_hertz,
                                        float *Kp, float *Ki);

#ifdef __cplusplus
}
#endif
#endif /* FOC_ALGORITHM_H */
