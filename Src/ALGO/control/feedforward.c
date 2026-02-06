#include "feedforward.h"
#include "config.h"

void Feedforward_Init(Feedforward_Params_t *params) {
  if (!params)
    return;
  params->inertia = 0.0f; // Default values, should be loaded from config
  params->friction_coeff = 0.0f;
}

void Feedforward_Update(MOTOR_DATA *motor, const Feedforward_Params_t *params) {
  if (!motor || !params)
    return;

  /*
   * 2. Friction (Velocity Torque)
   * torque_ff += friction_coeff * velocity
   */
  // Friction Compensation (Coulomb + Viscous)
  // Using Setpoint for pure feedforward. Using Feedback would be
  // "Feed-linearization"
  float velocity_ref = motor->Controller.vel_setpoint;

  float dt = 1.0f / (float)ADV_CONTROL_HZ;
  static float last_velocity_ref = 0.0f;
  static bool has_last = false;

  float accel = 0.0f;
  if (has_last && dt > 0.0f) {
    accel = (velocity_ref - last_velocity_ref) / dt;
  }
  last_velocity_ref = velocity_ref;
  has_last = true;

  // Inertia: T = J * dw/dt
  float inertia = params->inertia * accel;

  // Viscous: T = B * w
  float viscous = params->friction_coeff * velocity_ref;

  // Total FF
  float total_ff = inertia + viscous;

  // Inject into Torque Setpoint
  // WARNING: Ensuring this is additive relies on Control Loop behavior
  // If Control Loop resets torque_setpoint, this must be handled inside the
  // loop or summing junction. For now, we assume input_torque is the summing
  // junction for User + FF
  motor->Controller.input_torque += total_ff;
}
