#include "motor_plant.h"
#include <math.h>

void MotorPlant_Init(MotorPlant_t *plant) {
  plant->R = 0.1f;
  plant->L = 0.001f;
  plant->Flux = 0.01f;
  plant->J = 0.0001f;
  plant->B = 0.0001f;
  plant->P = 7;

  plant->i_alpha = 0.0f;
  plant->i_beta = 0.0f;
  plant->omega = 0.0f;
  plant->theta = 0.0f;

  plant->dt = 0.00005f; // 20kHz
}

void MotorPlant_Step(MotorPlant_t *plant, float v_alpha, float v_beta,
                     float load_torque) {
  // 1. Electrical Model (RL Circuit + BEMF)
  // di/dt = (V - R*i - BEMF) / L

  // BEMF Calculation
  // bemf_alpha = -lambda * omega_elec * sin(theta_elec)
  // bemf_beta  =  lambda * omega_elec * cos(theta_elec)

  float omega_elec = plant->omega * plant->P;
  float theta_elec = plant->theta * plant->P;

  plant->bemf_alpha = -plant->Flux * omega_elec * sinf(theta_elec);
  plant->bemf_beta = plant->Flux * omega_elec * cosf(theta_elec);

  float di_alpha =
      (v_alpha - plant->R * plant->i_alpha - plant->bemf_alpha) / plant->L;
  float di_beta =
      (v_beta - plant->R * plant->i_beta - plant->bemf_beta) / plant->L;

  plant->i_alpha += di_alpha * plant->dt;
  plant->i_beta += di_beta * plant->dt;

  // 2. Mechanical Model
  // T_motor = 1.5 * P * lambda * Iq
  // To get Iq, we need Park transform of Ialpha, Ibeta
  // Iq = -Ialpha * sin(theta) + Ibeta * cos(theta)

  float iq =
      -plant->i_alpha * sinf(theta_elec) + plant->i_beta * cosf(theta_elec);
  float t_motor = 1.5f * plant->P * plant->Flux * iq;

  float t_net = t_motor - load_torque - (plant->B * plant->omega);

  float d_omega = t_net / plant->J;
  plant->omega += d_omega * plant->dt;
  plant->theta += plant->omega * plant->dt;

  // Wrap theta
  while (plant->theta > 6.2831853f)
    plant->theta -= 6.2831853f;
  while (plant->theta < 0.0f)
    plant->theta += 6.2831853f;
}

void MotorPlant_GetCurrents(MotorPlant_t *plant, float *ia, float *ib,
                            float *ic) {
  // Inverse Clarke
  // Ia = Ialpha
  // Ib = -0.5*Ialpha + sqrt(3)/2 * Ibeta
  // Ic = -0.5*Ialpha - sqrt(3)/2 * Ibeta

  *ia = plant->i_alpha;
  *ib = -0.5f * plant->i_alpha + 0.866025f * plant->i_beta;
  *ic = -0.5f * plant->i_alpha - 0.866025f * plant->i_beta;
}
