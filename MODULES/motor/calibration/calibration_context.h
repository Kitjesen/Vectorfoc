#ifndef CALIBRATION_CONTEXT_H
#define CALIBRATION_CONTEXT_H

#include "common.h"
#include "motor/config.h"

/**
 * @file calibration_context.h
 * @brief Calibration process context structure
 *
 * This file defines context structures for all calibration processes, replacing
 * legacy static variable implementations.
 * Benefits:
 * 1. Supports multiple motor instances
 * 2. State serializable
 * 3. Facilitates unit testing
 * 4. Clear lifecycle management
 */

//=============================================================================
// Current Offset Calibration Context
//=============================================================================

/**
 * @brief Current offset calibration state
 */
typedef struct {
  uint32_t loop_count; // Loop counter
  float offset_sum_a;  // Phase A offset accumulator
  float offset_sum_b;  // Phase B offset accumulator
  float offset_sum_c;  // Phase C offset accumulator
  bool is_initialized; // Initialization flag
} CurrentCalibContext;

//=============================================================================
// Resistance Calibration Context
//=============================================================================

/**
 * @brief Resistance calibration state
 */
typedef struct {
  uint32_t loop_count; // Loop counter
  float voltage;       // Applied test voltage
  float kI;            // PI controller gain (constant)
} ResistanceCalibContext;

//=============================================================================
// Inductance Calibration Context
//=============================================================================

/**
 * @brief Inductance calibration state
 */
typedef struct {
  uint32_t loop_count; // Loop counter
  float Ialphas[2];    // Steady state currents for two states
  float voltages[2];   // Two test voltages [+V, -V]
} InductanceCalibContext;

//=============================================================================
// Direction/Pole Pair Calibration Context
//=============================================================================

/**
 * @brief Direction and pole pair calibration state
 */
typedef struct {
  uint32_t loop_count; // Loop counter
  float phase_set;     // Current electrical angle setpoint
  float start_count;   // Start encoder count
  float voltage;       // Test voltage
} DirectionPoleCalibContext;

//=============================================================================
// Encoder Calibration Context
//=============================================================================

/**
 * @brief Encoder offset and lookup table calibration state
 */
typedef struct {
  uint32_t loop_count;     // Loop counter
  float phase_set;         // Current electrical angle
  int16_t sample_count;    // Sample counter
  float next_sample_time;  // Next sampling timestamp
  int error_array_storage[SAMPLES_PER_POLE_PAIR * MAX_POLE_PAIRS];
  int *error_array;        // Error array (preallocated storage)
  size_t error_array_size; // Array size
} EncoderCalibContext;

//=============================================================================
// Flux Calibration Context
//=============================================================================

/**
 * @brief Flux calibration state
 */
typedef struct {
  uint32_t loop_count;   // Loop counter
  float phase_set;       // Current electrical angle
  float flux_sum;        // Flux accumulator
  uint32_t flux_samples; // Valid sample count
  float target_velocity; // Target velocity [turn/s]
} FluxCalibContext;

//=============================================================================
// Complete Calibration Context (Aggregates all sub-states)
//=============================================================================

/**
 * @brief Complete calibration process context
 *
 * Contains state data for all calibration stages
 */
typedef struct {
  CurrentCalibContext current;        // Current offset calibration
  ResistanceCalibContext resistance;  // Resistance calibration
  InductanceCalibContext inductance;  // Inductance calibration
  DirectionPoleCalibContext dir_pole; // Direction/Pole Pair calibration
  EncoderCalibContext encoder;        // Encoder calibration
  FluxCalibContext flux;              // Flux calibration
} CalibrationContext;

//=============================================================================
// Calibration Result Enum
//=============================================================================

/**
 * @brief Calibration result status codes
 */
typedef enum {
  CALIB_SUCCESS = 0,           // Calibration successful
  CALIB_IN_PROGRESS,           // Calibration in progress
  CALIB_FAILED_TIMEOUT,        // Timeout failure
  CALIB_FAILED_NO_CURRENT,     // Current anomaly
  CALIB_FAILED_NO_MOVEMENT,    // Motor not moving
  CALIB_FAILED_INVALID_PARAMS, // Invalid parameters
  CALIB_FAILED_MEMORY,         // Memory allocation failure
} CalibResult;

//=============================================================================
// Context Management Functions
//=============================================================================

/**
 * @brief Initialize calibration context
 * @param ctx Calibration context pointer
 */
void CalibContext_Init(CalibrationContext *ctx);

/**
 * @brief Release calibration context resources
 * @param ctx Calibration context pointer
 */
void CalibContext_Release(CalibrationContext *ctx);

/**
 * @brief Reset calibration context (for re-calibration)
 * @param ctx Calibration context pointer
 */
void CalibContext_Reset(CalibrationContext *ctx);

#endif // CALIBRATION_CONTEXT_H
