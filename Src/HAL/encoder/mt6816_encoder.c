#include "mt6816_encoder.h"
#include "board_config.h"
#include "common.h"
#include "config.h"
#include "error_manager.h"
#include "error_types.h"
#include <math.h>


/** MT6816 Register Addresses */
#define MT6816_REG_ANGLE_H 0x83 ///< Angle High Byte (Read)
#define MT6816_REG_ANGLE_L 0x84 ///< Angle Low Byte (Read)
#define MT6816_REG_STATUS 0x85  ///< Status Register

/** PLL Default Bandwidth */
#ifndef ENCODER_PLL_BANDWIDTH
#define ENCODER_PLL_BANDWIDTH 1000.0f
#endif

/** Helper Macros */
#define SQ(x) ((x) * (x))

/* ============================================================================
 * Global Variables
 * ============================================================================
 */

/** Default encoder instance (kept for compatibility, but low-level code won't
 * rely on it) */
ENCODER_DATA encoder_data = {
    .hspi = &HW_ENC_SPI,
    .cs_port = HW_ENC_CS_PORT,
    .cs_pin = HW_ENC_CS_PIN,
    .pole_pairs = 7,
    .offset_counts = 0,
    .dir = MT6816_DIR_CW,
    .raw_angle = 0,
    .shadow_count = 0,
    .count_in_cpr = 0,
    .pos_estimate_counts_ = 0.0f,
    .vel_estimate_counts_ = 0.0f,
    .pos_cpr_counts_ = 0.0f,
    .pos_estimate_ = 0.0f,
    .vel_estimate_ = 0.0f,
    .pos_cpr_ = 0.0f,
    .phase_ = 0.0f,
    .interpolation_ = 0.0f,
    .elec_angle_rad = 0.0f,
    .mec_angle_rad = 0.0f,
    .velocity_rad_s = 0.0f,
    .calib_valid = false,
};

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================
 */

/**
 * @brief Chip Select Control (inline helper)
 */
static inline void MT6816_CS_Low(MT6816_Handle_t *encoder) {
  HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_RESET);
}

static inline void MT6816_CS_High(MT6816_Handle_t *encoder) {
  HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Read raw angle from MT6816 via SPI
 * @param encoder Encoder handle
 * @return bool True if successful (SPI OK and Parity OK)
 */
static bool mt6816_read_raw(MT6816_Handle_t *encoder);

/* ============================================================================
 * Public Function Implementation
 * ============================================================================
 */

void MT6816_Init(MT6816_Handle_t *encoder, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin) {
  if (encoder == NULL)
    return;

  encoder->hspi = hspi;
  encoder->cs_port = cs_port;
  encoder->cs_pin = cs_pin;
  encoder->pole_pairs = 7; // Default pole pairs
  encoder->dir = MT6816_DIR_CW;
  encoder->offset_counts = 0;
  // Initialize Bandwidth from Macro or Default
#ifdef ENCODER_PLL_BANDWIDTH
  encoder->pll_bandwidth = ENCODER_PLL_BANDWIDTH;
#else
  encoder->pll_bandwidth = MT6816_PLL_BW_DEFAULT;
#endif

  encoder->raw_angle = 0;
  encoder->shadow_count = 0;
  encoder->count_in_cpr = 0;
  encoder->calib_valid = false;
  encoder->is_calibrated = false;

  // Initialize PLL State
  encoder->pos_estimate_counts_ = 0.0f;
  encoder->vel_estimate_counts_ = 0.0f;
  encoder->pos_cpr_counts_ = 0.0f;
  encoder->pos_estimate_ = 0.0f;
  encoder->vel_estimate_ = 0.0f;
  encoder->pos_cpr_ = 0.0f;
  encoder->phase_ = 0.0f;
  encoder->interpolation_ = 0.0f;

  // Initialize Outputs
  encoder->mec_angle_rad = 0.0f;
  encoder->elec_angle_rad = 0.0f;
  encoder->velocity_rad_s = 0.0f;
  encoder->last_status = MT6816_OK;
}

MT6816_Status_t MT6816_Update(MT6816_Handle_t *encoder, float dt) {
  if (encoder == NULL) {
    return MT6816_ERR_SPI;
  }

  // Call main processing function with proper dt
  GetMotor_Angle(encoder, dt);

  // Update velocity (rad/s)
  encoder->velocity_rad_s = encoder->vel_estimate_ * M_2PI;

  return encoder->last_status;
}

void MT6816_ResetCount(MT6816_Handle_t *encoder) {
  if (encoder == NULL)
    return;

  encoder->shadow_count = 0;
  encoder->pos_estimate_counts_ = 0.0f;
  encoder->pos_estimate_ = 0.0f;
}

float normalize_angle(float angle) {
  float a = fmodf(angle, M_2PI);
  return a >= 0 ? a : (a + M_2PI);
}

/**
 * @brief Get encoder angle and velocity (Core Processing)
 */
void GetMotor_Angle(MT6816_Handle_t *encoder, float dt) {
  // PLL Parameters (Calculated based on bandwidth and dt)
  // Optimization: Pre-calculate these if dt is constant, but dynamic is safer
  // for jittery loops.
  float pll_kp_ = 2.0f * encoder->pll_bandwidth;
  float pll_ki_ = 0.25f * SQ(pll_kp_);

  // Snap threshold: velocity change needed to integrate < 0.5 counts per
  // period? Threshold should be small enough to stop drift but large enough to
  // ignore noise.
  float snap_threshold = 0.5f * dt * pll_ki_;

  /* === 1. Read Raw Angle === */
  if (mt6816_read_raw(encoder)) {
    // Direction Compensation (Boundary Safe)
    if (encoder->dir == MT6816_DIR_CW) {
      encoder->raw_angle = encoder->angle;
    } else {
      encoder->raw_angle = (MT6816_CPR - 1 - encoder->angle);
    }
  } else {
    // SPI or Parity Error:
    // EARLY RETURN to avoid corrupting state with stale or zero data.
    // The system will effectively "coast" on previous estimates.
    return;
  }

  /* === 2. Non-linearity Calibration (LUT) === */
  int cnt;
  if (encoder->calib_valid) {
    // Linear interpolation using LUT
    int idx1 = encoder->raw_angle >> 7; // raw_angle / 128
    int idx2 = (idx1 + 1) % MT6816_LUT_SIZE;
    int off_1 = encoder->offset_lut[idx1];
    int off_2 = encoder->offset_lut[idx2];

    int remainder = encoder->raw_angle - (idx1 << 7);
    int off_interp = off_1 + ((off_2 - off_1) * remainder >> 7);

    cnt = encoder->raw_angle - off_interp;
  } else {
    cnt = encoder->raw_angle;
  }

  // Normalize to [0, MT6816_CPR)
  if (cnt >= MT6816_CPR) {
    cnt -= MT6816_CPR;
  } else if (cnt < 0) {
    cnt += MT6816_CPR;
  }

  /* === 3. Calculate Incremental Position (Wrap Handling) === */
  int old_cnt = encoder->count_in_cpr;
  encoder->count_in_cpr = cnt;

  int delta_enc = cnt - old_cnt;

  // Robust wrap detection
  if (delta_enc > (int)(MT6816_CPR / 2)) {
    delta_enc -= MT6816_CPR;
  } else if (delta_enc < -(int)(MT6816_CPR / 2)) {
    delta_enc += MT6816_CPR;
  }

  // Accumulate to 64-bit shadow counter
  encoder->shadow_count += (int64_t)delta_enc;

  /* === 4. Run PLL (Incremental / Modulo Domain Only) === */
  // We use PLL primarily for Velocity estimation and sub-count smoothing.
  // We DO NOT use absolute float position for long-term feedback to avoid
  // precision loss.

  // Predict current single-turn position
  encoder->pos_cpr_counts_ += dt * encoder->vel_estimate_counts_;

  // Phase Detector (Modulo CPR)
  float delta_pos_cpr_counts =
      (float)(encoder->count_in_cpr -
              (int32_t)floorf(encoder->pos_cpr_counts_));
  delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(MT6816_CPR / 2));

  // Update PLL State
  // Proportional term drives position
  encoder->pos_cpr_counts_ += dt * pll_kp_ * delta_pos_cpr_counts;

  // Wrap tracked position
  encoder->pos_cpr_counts_ = fmodf_pos(encoder->pos_cpr_counts_, MT6816_CPR_F);

  // Integral term drives velocity
  encoder->vel_estimate_counts_ += dt * pll_ki_ * delta_pos_cpr_counts;

  // Snap to zero velocity if very slow (anti-drift)
  bool snap_to_zero_vel = false;
  if (ABS(encoder->vel_estimate_counts_) < snap_threshold) {
    encoder->vel_estimate_counts_ = 0.0f;
    snap_to_zero_vel = true;
  }

  /* === 5. Interpolation for Low Speed Smoothness === */
  // Use velocity estimate to interpolate between counts
  if (snap_to_zero_vel) {
    encoder->interpolation_ = 0.5f; // Center in the bin
  } else if (delta_enc != 0) {
    // We moved, reset interpolation based on direction
    encoder->interpolation_ = (delta_enc > 0) ? 0.0f : 1.0f;
  } else {
    // We haven't moved a full count yet, estimate sub-count movement
    encoder->interpolation_ += dt * encoder->vel_estimate_counts_;
    // Clamp to [0, 1]
    if (encoder->interpolation_ > 1.0f)
      encoder->interpolation_ = 1.0f;
    if (encoder->interpolation_ < 0.0f)
      encoder->interpolation_ = 0.0f;
  }

  /* === 6. Output Calculation === */
  // Velocity comes from PLL
  encoder->vel_estimate_ = encoder->vel_estimate_counts_ / MT6816_CPR_F;

  // Position "Estimate" (Multi-turn)
  // Uses integer shadow_count for absolute truth, plus interpolation/PLL phase
  // for smoothness? User suggestion: "pos_estimate_ = shadow_count / CPR".
  // Simple, robust, no float integration drift. To get sub-count resolution, we
  // can add (count_in_cpr - corrected_enc)? No. Let's us shadow_count +
  // interpolation offset (centered at 0) interpolation [0,1], count is integer.
  // shadow_count is at the integer step.
  // pos_estimate_ = (shadow_count + interpolation - 0.5? No, interpolation is
  // 0..1 relative to count) Let's stick to the reliable integer base for now:
  encoder->pos_estimate_ = (double)encoder->shadow_count / MT6816_CPR_F;

  // Single turn position
  encoder->pos_cpr_ = encoder->pos_cpr_counts_ / MT6816_CPR_F;

  /* === 7. Electrical & Mechanical Angles === */
  // Compute corrected integer count
  int32_t corrected_enc = encoder->count_in_cpr - encoder->offset_counts;
  // Wrap
  while (corrected_enc < 0)
    corrected_enc += MT6816_CPR;
  while (corrected_enc >= MT6816_CPR)
    corrected_enc -= MT6816_CPR;

  // Add interpolation for smoothness
  float interpolated_enc =
      (float)corrected_enc +
      encoder->interpolation_; // - 0.5f if we want center alignment?

  float elec_rad_per_enc = encoder->pole_pairs * M_2PI * (1.0f / MT6816_CPR_F);

  // Electrical Angle
  encoder->phase_ = wrap_pm_pi(elec_rad_per_enc * interpolated_enc);
  encoder->elec_angle_rad = encoder->phase_;

  // Mechanical Angle [0, 2PI)
  encoder->mec_angle_rad =
      normalize_angle(interpolated_enc * (M_2PI / MT6816_CPR_F));
}

/* ============================================================================
 * Private Function Implementation
 * ============================================================================
 */

/**
 * @brief Read Raw Angle using HAL_SPI_TransmitReceive for robust timing
 */
static bool mt6816_read_raw(MT6816_Handle_t *encoder) {
  // Protocol Assumption Fix:
  // Transaction size = 3 bytes.
  // TX: [CmdH, CmdL, Dummy]
  // RX: [Dummy, DataH, DataL]
  // OR [DatePrev, DataH, DataL]?
  // We allocate 3 bytes to ensure we clock out the data responding to the
  // command.

  uint8_t tx_data[3] = {MT6816_REG_ANGLE_H, MT6816_REG_ANGLE_L, 0x00};
  uint8_t rx_data[3] = {0, 0, 0};

  MT6816_CS_Low(encoder);

  if (HAL_SPI_TransmitReceive(encoder->hspi, tx_data, rx_data, 3,
                              MT6816_MAX_DELAY) != HAL_OK) {
    MT6816_CS_High(encoder);
    if (encoder->rx_err_count < 0xFFFF)
      encoder->rx_err_count++;
    
    // 报告SPI错误（仅在第一次和每100次）
    if (encoder->rx_err_count == 1 || encoder->rx_err_count % 100 == 0) {
      ERROR_REPORT(ERROR_MOTOR_ENCODER_SPI, "MT6816 SPI error");
    }
    encoder->last_status = MT6816_ERR_SPI;
    return false;
  }

  MT6816_CS_High(encoder);

  // Success
  if (encoder->rx_err_count > 0)
    encoder->rx_err_count--;

  // Parse Raw Angle from RX[1] and RX[2]
  // Assuming the First byte Rx[0] is dummy/response to CmdH
  uint16_t raw_val = ((uint16_t)rx_data[1] << 8) | rx_data[2];

  // Parity Check (Even Parity on 16-bit word)
  // p & 1 == 0 means Even # of 1s (including parity bit)
  uint16_t p = raw_val;
  p ^= p >> 8;
  p ^= p >> 4;
  p ^= p >> 2;
  p ^= p >> 1;

  if (p & 1) {
    if (encoder->check_err_count < 0xFFFF)
      encoder->check_err_count++;
    encoder->last_status = MT6816_ERR_PARITY;
    return false;
  }

  // Extract valid 14-bit angle (Top 14 bits)
  encoder->angle = raw_val >> 2;

  if (encoder->check_err_count > 0)
    encoder->check_err_count--;

  encoder->last_status = MT6816_OK;
  return true;
}
