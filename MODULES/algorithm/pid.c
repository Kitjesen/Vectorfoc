/**
 * @file    pid.c
 * @brief   PID controller implementation.
 */

#include "pid.h"
#include <math.h>

/**
 * @brief Limit value to +/- max.
 */
static void LimitMax(float *input, float max) {
  if (*input > max) {
    *input = max;
  } else if (*input < -max) {
    *input = -max;
  }
}

void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out,
              float max_iout) {
  if (pid == NULL || PID == NULL) {
    return;
  }
  pid->mode = mode;
  pid->Kp = PID[0];
  pid->Ki = PID[1];
  pid->Kd = PID[2];
  pid->max_out = max_out;
  pid->max_iout = max_iout;
  pid->tau = 0.0f; // Default: no filtering

  PID_clear(pid);
}

void PID_clear(PidTypeDef *pid) {
  if (pid == NULL) {
    return;
  }

  pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->out = pid->out_sat = pid->Pout = pid->Iout = pid->Dout = 0.0f;
  pid->fdb = pid->set = pid->prevMeasure = 0.0f;
}

float PID_CalcDt(PidTypeDef *pid, float fdb, float set, float dt) {
  if (pid == NULL) {
    return 0.0f;
  }

  // Sanity check for dt
  if (dt <= 0.000001f) {
    return pid->out;
  }

  // Update State
  pid->set = set;
  pid->fdb = fdb;
  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->error[0] = set - fdb;

  float p_term, i_term, d_term;

  if (pid->mode == PID_POSITION) {
    // --- Proportional ---
    p_term = pid->Kp * pid->error[0];
    pid->Pout = p_term;

    // --- Integral ---
    // Calculate candidate integral
    i_term = pid->Iout + (pid->Ki * pid->error[0] * dt);

    // Anti-windup: Conditional Integration (Clamping)
    // If output is saturated AND error is pushing further into saturation, stop
    // integrating. However, classical clamping just limits Iout. Better: Limit
    // Iout strictly.
    LimitMax(&i_term, pid->max_iout);

    // --- Derivative ---
    // Derivative on Measurement: -Kd * (fdb - prev_fdb) / dt
    // This avoids "Derivative Kick" when setpoint changes.
    // Falls back to Error derivative if needed, but Measurement is standard for
    // motor control.
    float delta_measure = fdb - pid->prevMeasure;
    float derivative_raw = delta_measure / dt;

    // Low Pass Filter for D-term
    // D_filt = alpha * D_filt_prev + (1-alpha) * D_raw
    // alpha = dt / (dt + tau)
    // If tau = 0, alpha = 1, no filtering.
    // Standard discrete form: y[k] = alpha*x[k] + (1-alpha)*y[k-1] ? No.
    // LPF: y[k] = y[k-1] + alpha * (x[k] - y[k-1])
    float alpha = 1.0f;
    if (pid->tau > 0.0f) {
      alpha = dt / (pid->tau + dt);
    }

    // Use Dout as the state variable for filtered value
    // Dout += alpha * (derivative_raw - Dout)
    pid->Dbuf[0] += alpha * (derivative_raw - pid->Dbuf[0]);

    // Dout = -Kd * filtered_rate (Derivative on Measure -> sign inversion)
    // Wait, error = set - fdb.
    // d(error)/dt = d(set)/dt - d(fdb)/dt.
    // If setpoint is constant, d(error) = -d(fdb).
    // So +Kd * d(error) becomes -Kd * d(fdb). Correct.
    d_term = -pid->Kd * pid->Dbuf[0];
    pid->Dout = d_term;

    // --- Summation ---
    float out_unlimited = p_term + i_term + d_term;
    pid->out_sat = out_unlimited;

    // --- Output Saturation & Integral Clamping support ---
    // If we simply clamped Iout earlier, we are checking Iout bounds.
    // But if we want output anti-windup:
    // If (out_unlimited > max_out) and (i_term trying to increase), inhibit
    // i_term. For simplicity and standard robust behavior, we just use Iout
    // clamping + Output clamping. Logic updated above: i_term was clamped to
    // max_iout. Now check total output.

    if (out_unlimited > pid->max_out) {
      pid->out = pid->max_out;
      // Conditional integration: if error > 0 (trying to increase), don't
      // update Iout
      if (pid->error[0] > 0.0f) {
        i_term = pid->Iout; // Revert integration
      }
    } else if (out_unlimited < -pid->max_out) {
      pid->out = -pid->max_out;
      // Conditional integration: if error < 0 (trying to decrease), don't
      // update Iout
      if (pid->error[0] < 0.0f) {
        i_term = pid->Iout; // Revert integration
      }
    } else {
      pid->out = out_unlimited;
    }

    pid->Iout = i_term;

  } else if (pid->mode == PID_DELTA) {
    // Incremental PID (Historical/Legacy support)
    // out += Kp*(e - e_1) + Ki*e*dt + Kd*(e - 2e_1 + e_2)/dt

    // Proportional Inc
    float dp = pid->Kp * (pid->error[0] - pid->error[1]);

    // Integral Inc
    float di = pid->Ki * pid->error[0] * dt;

    // Derivative Inc
    // Using simple error difference for Delta mode (standard)
    float dd =
        pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) / dt;
    // Note: Delta mode typically doesn't filter perfectly without state.

    float delta_u = dp + di + dd;

    pid->out += delta_u;

    // Saturation
    if (pid->out > pid->max_out) {
      pid->out = pid->max_out;
    } else if (pid->out < -pid->max_out) {
      pid->out = -pid->max_out;
    }

    pid->Pout = dp;
    pid->Iout = di;
    pid->Dout = dd;
  }

  pid->prevMeasure = fdb;
  return pid->out;
}

float PID_Calc(PidTypeDef *pid, float ref, float set) {
  // Wrapper assuming unit dt or implicit dt scaling (Legacy Support)
  // If the legacy gains were scaled for a specific loop time, this preserves
  // behavior IF dt = 1.0f. WARNING: This assumes caller scaled Ki/Kd for the
  // loop rate already.
  return PID_CalcDt(pid, ref, set, 1.0f);
}
