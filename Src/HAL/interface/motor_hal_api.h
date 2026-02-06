#ifndef MOTOR_HAL_API_H
#define MOTOR_HAL_API_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Motor Hardware Abstraction Layer API
 * 
 * This file defines the abstract interfaces for motor control hardware.
 * The core motor logic (motor_core) depends ONLY on these interfaces,
 * allowing it to be decoupled from specific hardware drivers (STM32 HAL, MT6816, etc.).
 */

/* ============================================================================
 * Data Structures
 * ============================================================================ */

typedef struct {
    float i_a;      // Phase A current [A]
    float i_b;      // Phase B current [A]
    float i_c;      // Phase C current [A]
    float v_bus;    // Bus voltage [V]
    float temp;     // Temperature [degC]
} Motor_HAL_SensorData_t;

typedef struct {
    float angle_rad;     // Mechanical angle [rad] (0 ~ 2PI)
    float velocity_rad;  // Mechanical velocity [rad/s]
    float elec_angle;    // Electrical angle [rad] (-PI ~ PI) (Optional, or calculated by core)
    int32_t raw_value;   // Raw encoder value (for debugging/calibration)
} Motor_HAL_EncoderData_t;

/* ============================================================================
 * Interfaces (Function Pointers)
 * ============================================================================ */

/**
 * @brief PWM Driver Interface
 */
typedef struct {
    /**
     * @brief Set PWM duty cycles for 3 phases
     * @param dtc_a Duty cycle phase A (0.0 - 1.0)
     * @param dtc_b Duty cycle phase B (0.0 - 1.0)
     * @param dtc_c Duty cycle phase C (0.0 - 1.0)
     */
    void (*set_duty)(float dtc_a, float dtc_b, float dtc_c);

    /**
     * @brief Enable PWM output
     */
    void (*enable)(void);

    /**
     * @brief Disable PWM output (High-Z)
     */
    void (*disable)(void);

    /**
     * @brief Brake (Low-side ON or similar)
     */
    void (*brake)(void);
    
} Motor_HAL_PwmInterface_t;

/**
 * @brief ADC Driver Interface
 */
typedef struct {
    /**
     * @brief Update sensor data (Currents, Vbus, Temp)
     * This may trigger ADC conversion or read buffered values.
     * @param data Pointer to data structure to fill
     */
    void (*update)(Motor_HAL_SensorData_t *data);

    /**
     * @brief Calibrate current offsets
     * Should be called when motor is idle.
     */
    void (*calibrate_offsets)(void);

} Motor_HAL_AdcInterface_t;

/**
 * @brief Encoder Driver Interface
 */
typedef struct {
    /**
     * @brief Update encoder state
     * Should be called periodically or before control loop.
     */
    void (*update)(void);

    /**
     * @brief Get latest encoder data
     * @param data Pointer to data structure to fill
     */
    void (*get_data)(Motor_HAL_EncoderData_t *data);

    /**
     * @brief Set electrical zero offset (if handled by driver)
     * @param offset Offset value
     */
    void (*set_offset)(float offset);

} Motor_HAL_EncoderInterface_t;

/**
 * @brief Main HAL Handle
 */
typedef struct {
    const Motor_HAL_PwmInterface_t     *pwm;
    const Motor_HAL_AdcInterface_t     *adc;
    const Motor_HAL_EncoderInterface_t *encoder;
} Motor_HAL_Handle_t;

#endif // MOTOR_HAL_API_H
