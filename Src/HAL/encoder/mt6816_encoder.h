/**
 * @file    mt6816_encoder.h
 * @brief   MT6816 magnetic encoder driver (14-bit, SPI).
 * @details
 * - Context: Reads absolute position, runs PLL for velocity estimation.
 * - Units:   Position [rad], Velocity [rad/s], with internal counts &
 * revolutions.
 * - Resolution: 14-bit (16384 CPR).
 * @version 2.0
 */

#ifndef MT6816_ENCODER_H
#define MT6816_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Constants */
#define MT6816_CPR 16384u /**< Counts per revolution (14-bit) */
#define MT6816_CPR_F 16384.0f
#define MT6816_LUT_SIZE 128           /**< Linearity compensation LUT size */
#define MT6816_MAX_DELAY 10           /**< [ms] SPI timeout */
#define MT6816_PLL_BW_DEFAULT 2000.0f /**< [Hz] Default PLL bandwidth */

/**
 * @brief Motor rotation direction.
 */
typedef enum {
  MT6816_DIR_CW = 1,  /**< Clockwise */
  MT6816_DIR_CCW = -1 /**< Counter-clockwise */
} MT6816_Direction_t;

/**
 * @brief Encoder status codes.
 */
typedef enum {
  MT6816_OK = 0,         /**< Normal operation */
  MT6816_ERR_SPI = 1,    /**< SPI communication error */
  MT6816_ERR_PARITY = 2, /**< Parity check error */
  MT6816_ERR_MAG = 3     /**< Magnetic field anomaly */
} MT6816_Status_t;

/**
 * @brief MT6816 encoder handle.
 */
typedef struct MT6816_Handle_t {
  /* Hardware interface */
  SPI_HandleTypeDef *hspi; /**< SPI handle */
  GPIO_TypeDef *cs_port;   /**< CS GPIO port */
  uint16_t cs_pin;         /**< CS GPIO pin */

  /* Configuration */
  uint8_t pole_pairs;     /**< Motor pole pairs */
  MT6816_Direction_t dir; /**< Rotation direction */
  float offset_rev;       /**< [revolutions] Zero offset (0-1.0) */
  int32_t offset_counts;  /**< [counts] Zero offset */
  float pll_bandwidth;    /**< [Hz] PLL bandwidth */

  /* Raw data */
  uint16_t raw_angle;       /**< [counts] Raw angle (0-16383) */
  uint16_t angle;           /**< [counts] Processed angle */
  uint32_t rx_err_count;    /**< SPI error counter */
  uint32_t check_err_count; /**< Parity error counter */
  int64_t shadow_count; /**< [counts] Multi-turn accumulator (64-bit to prevent
                           overflow) */
  int32_t count_in_cpr; /**< [counts] Single-turn count (0-16383) */

  /* Calibration data */
  int16_t offset_lut[MT6816_LUT_SIZE]; /**< Linearity compensation LUT */
  bool calib_valid;                    /**< Calibration data valid flag */

  /* PLL state (internal) */
  float pos_estimate_counts_; /**< [counts] Position estimate */
  float vel_estimate_counts_; /**< [counts/s] Velocity estimate */
  float pos_cpr_counts_;      /**< [counts] Single-turn position estimate */
  float interpolation_;       /**< [0-1] Interpolation coefficient */

  /* Output (SI units) */
  float pos_estimate_;  /**< [revolutions] Position estimate */
  float vel_estimate_;  /**< [rev/s] Velocity estimate */
  float pos_cpr_;       /**< [revolutions] Single-turn position (0-1) */
  float phase_;         /**< [rad] Electrical angle (-π~π) */
  float mec_angle_rad;  /**< [rad] Mechanical angle (0~2π) */
  float elec_angle_rad; /**< [rad] Electrical angle (-π~π) */
  float velocity_rad_s; /**< [rad/s] Velocity */

  /* Status */
  MT6816_Status_t last_status; /**< Last operation status */
  bool is_calibrated;          /**< Legacy calibration flag */

} MT6816_Handle_t;

/** Legacy typedef for backward compatibility */
typedef struct MT6816_Handle_t ENCODER_DATA;

extern ENCODER_DATA encoder_data;

/**
 * @brief  Initialize encoder.
 * @param  encoder  Encoder handle.
 * @param  hspi     SPI handle.
 * @param  cs_port  CS GPIO port.
 * @param  cs_pin   CS GPIO pin.
 */
void MT6816_Init(MT6816_Handle_t *encoder, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * @brief  Update encoder data (call in control loop).
 * @param  encoder Encoder handle.
 * @param  dt      [s] Sampling period.
 * @return Status code.
 * @note   Should be called at fixed rate (e.g., 20kHz).
 */
MT6816_Status_t MT6816_Update(MT6816_Handle_t *encoder, float dt);

/**
 * @brief  Reset multi-turn counter.
 * @param  encoder Encoder handle.
 */
void MT6816_ResetCount(MT6816_Handle_t *encoder);

/**
 * @brief  Get encoder angle and velocity (core processing).
 * @param  encoder Encoder handle.
 * @note   Includes PLL, interpolation, and electrical angle calculation.
 */
void GetMotor_Angle(ENCODER_DATA *encoder, float dt);

/**
 * @brief  Normalize angle to [-π, π].
 * @param  angle [rad] Input angle.
 * @return [rad] Normalized angle.
 */
float normalize_angle(float angle);

#ifdef __cplusplus
}
#endif

#endif // MT6816_ENCODER_H