/**
 * @file    pid.h
 * @brief   PID controller implementation (Position & Delta modes).
 * @details
 * - Context: Generic PID module used for motor control loops.
 * - Units:   Dependent on input/output (e.g., A, rad/s, rad).
 */

#ifndef ALGORITHM_PID_H
#define ALGORITHM_PID_H

#include "common.h"

/**
 * @brief PID control mode.
 */
typedef enum {
  PID_POSITION = 0, /**< Standard positional PID */
  PID_DELTA = 1     /**< Incremental (delta) PID */
} PidMode_e;

/**
 * @brief PID State and Parameters.
 */
typedef struct {
  uint8_t mode;   /**< [PidMode_e] Control mode */
  float Kp;       /**< Proportional gain */
  float Ki;       /**< Integral gain */
  float Kd;       /**< Derivative gain */
  float max_out;  /**< Output saturation limit */
  float max_iout; /**< Integral anti-windup limit */
  float out;      /**< Last calculated output (limited) */
  float out_sat;  /**< Calculated output (unlimited) for anti-windup */
  float Pout;     /**< Proportional term output */
  float Iout;     /**< Integral term output */
  float Dout;     /**< Derivative term output */
  float Dbuf[3];  /**< Buffer for derivative filtering/calculation (Legacy) */
  float error[3]; /**< Error history (0:curr, 1:last, 2:prev) */
  float prevMeasure; /**< Last feedback measurement (for measurement derivative)
                      */
  float tau;         /**< Derivative low-pass filter time constant */
  float fdb;         /**< Last feedback value */
  float set;         /**< Last setpoint value */
} PidTypeDef;

/**
 * @brief  Initialize PID structure.
 * @param  pid      PID instance.
 * @param  mode     Control mode (PID_POSITION or PID_DELTA).
 * @param  PID      Array of gains [Kp, Ki, Kd].
 * @param  max_out  Maximum output limit.
 * @param  max_iout Maximum integral limit.
 */
void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out,
              float max_iout);

/**
 * @brief  Calculate PID output with dt (Recommended).
 * @param  pid  PID instance.
 * @param  fdb  Feedback value.
 * @param  set  Setpoint value.
 * @param  dt   Sampling time [s].
 * @return Calculated PID output.
 */
float PID_CalcDt(PidTypeDef *pid, float fdb, float set, float dt);

/**
 * @brief  Calculate PID output (Legacy wrapper, assumes implicit dt or unit
 * dt).
 * @param  pid  PID instance.
 * @param  ref  Feedback value.
 * @param  set  Setpoint value.
 * @return Calculated PID output.
 */
float PID_Calc(PidTypeDef *pid, float ref, float set);

/**
 * @brief  Clear PID state (reset integrator and errors).
 * @param  pid PID instance.
 */
void PID_clear(PidTypeDef *pid);

#endif // ALGORITHM_PID_H
