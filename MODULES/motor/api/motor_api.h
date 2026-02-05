#ifndef MOTOR_API_H
#define MOTOR_API_H

#include "motor.h"
#include <stdbool.h>

/**
 * @brief Initialization of the high-level API and underlying modules.
 * Should be called once at startup.
 */
// Motor_API_Init removed - lazy initialization used

// =============================================================================
// Tuning / Configuration APIs
// Provide simple "Bridge" for users to adjust settings without digging into
// structs
// =============================================================================

/**
 * @brief Configure and Enable Sliding Mode Observer
 * @param alpha SMO Alpha Gain
 * @param beta SMO Beta Gain
 */
void Motor_API_ConfigSMO(float alpha, float beta);

/**
 * @brief Configure Feedforward terms
 * @param friction_coeff Friction coefficient for velocity feedforward
 * Note: Inertia is configured in motor_data.Controller.inertia directly or via
 * param table
 */
void Motor_API_ConfigFeedforward(float friction_coeff);

/**
 * @brief Configure Field Weakening
 * @param max_current Maximum extra Id current to inject [A]
 * @param start_velocity Velocity start threshold [rad/s] (?)
 */
void Motor_API_ConfigFieldWeakening(float max_current, float start_velocity);

/**
 * @brief Configure Anti-Cogging Compensation
 * @param enable Enable/Disable
 */
void Motor_API_ConfigCogging(bool enable);
void Motor_API_StartCoggingCalib(MOTOR_DATA *motor);
void Motor_API_StopCoggingCalib(MOTOR_DATA *motor);
bool Motor_API_IsCoggingCalibrating(void);
bool Motor_API_IsCoggingMapValid(void);

// =============================================================================
// Task Hooks (Called by Motor Task / Interrupts)
// =============================================================================

/**
 * @brief Update observer (SMO) state
 * @param motor Pointer to motor data structure
 * @note Called in high-frequency ISR context (20kHz)
 */
void Motor_API_Observer_Update(MOTOR_DATA *motor);

/**
 * @brief Update feedforward control terms
 * @param motor Pointer to motor data structure
 * @note Called in high-frequency ISR context (20kHz)
 */
void Motor_API_Feedforward_Update(MOTOR_DATA *motor);

/**
 * @brief Update field weakening control
 * @param motor Pointer to motor data structure
 * @note Called in high-frequency ISR context (20kHz)
 */
void Motor_API_FieldWeakening_Update(MOTOR_DATA *motor);

/**
 * @brief Update cogging torque compensation
 * @param motor Pointer to motor data structure
 * @note Called in high-frequency ISR context (20kHz)
 */
void Motor_API_Cogging_Update(MOTOR_DATA *motor);

#endif // MOTOR_API_H
