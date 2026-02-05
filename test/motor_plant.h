#ifndef MOTOR_PLANT_H
#define MOTOR_PLANT_H

#include <stdbool.h>

typedef struct {
  // Motor Parameters (Model)
  float R;    // Phase Res [Ohm]
  float L;    // Phase Inductance [H]
  float Flux; // Flux Linkage [Wb]
  float J;    // Inertia [kg m^2]
  float B;    // Friction [Nm s/rad]
  int P;      // Pole Pairs

  // State Variables
  float i_alpha; // Alpha current [A]
  float i_beta;  // Beta current [A]
  float omega;   // Mechanical Velocity [rad/s]
  float theta;   // Mechanical Angle [rad]

  // Current simulation step inputs (Back EMF)
  float bemf_alpha;
  float bemf_beta;

  float dt; // Simulation timestep [s]

} MotorPlant_t;

void MotorPlant_Init(MotorPlant_t *plant);
void MotorPlant_Step(MotorPlant_t *plant, float v_alpha, float v_beta,
                     float load_torque);
void MotorPlant_GetCurrents(MotorPlant_t *plant, float *ia, float *ib,
                            float *ic);

#endif
