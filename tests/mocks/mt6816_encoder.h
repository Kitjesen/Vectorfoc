// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
/**
 * @file mt6816_encoder.h (TEST_ENV stub)
 */
#ifndef MT6816_ENCODER_H
#define MT6816_ENCODER_H
#include <stdbool.h>
#include <stdint.h>

#define MT6816_CPR    16384u
#define MT6816_CPR_F  16384.0f
#define MT6816_LUT_SIZE 128

typedef enum { MT6816_DIR_CW = 1, MT6816_DIR_CCW = -1 } MT6816_Direction_t;
typedef enum { MT6816_OK=0, MT6816_ERR_SPI=1, MT6816_ERR_PARITY=2, MT6816_ERR_MAG=3 } MT6816_Status_t;

typedef struct MT6816_Handle_t {
    void *hspi;
    void *cs_port;
    uint16_t cs_pin;
    uint8_t  pole_pairs;
    MT6816_Direction_t dir;
    float    offset_rev;
    int32_t  offset_counts;
    float    pll_bandwidth;
    uint16_t raw_angle;
    uint16_t angle;
    uint32_t rx_err_count;
    uint32_t check_err_count;
    int64_t  shadow_count;
    int32_t  count_in_cpr;
    int16_t  offset_lut[MT6816_LUT_SIZE];
    bool     calib_valid;
    float    pos_estimate_counts_;
    float    vel_estimate_counts_;
    float    pos_cpr_counts_;
    float    interpolation_;
    float    pos_estimate_;
    float    vel_estimate_;
    float    pos_cpr_;
    float    phase_;
    float    mec_angle_rad;
    float    elec_angle_rad;
    float    velocity_rad_s;
    MT6816_Status_t last_status;
    bool     is_calibrated;
} MT6816_Handle_t;

typedef struct MT6816_Handle_t ENCODER_DATA;
extern ENCODER_DATA encoder_data;

static inline void MT6816_Init(MT6816_Handle_t *e, void *h, void *p, uint16_t pin) {
    (void)e; (void)h; (void)p; (void)pin;
}
static inline MT6816_Status_t MT6816_Update(MT6816_Handle_t *e, float dt) {
    (void)e; (void)dt; return MT6816_OK;
}
static inline void MT6816_ResetCount(MT6816_Handle_t *e) { (void)e; }
static inline void GetMotor_Angle(ENCODER_DATA *e, float dt) { (void)e; (void)dt; }
static inline float normalize_angle(float a) { return a; }

#endif /* MT6816_ENCODER_H */
