#ifndef ABZ_ENCODER_H
#define ABZ_ENCODER_H

#include "motor_hal_api.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t cpr;
  uint8_t pole_pairs;
  bool direction;
  bool calib_valid;
  bool index_seen;
  bool last_z_state;
  int32_t raw_count;
  int32_t count_in_cpr;
  int32_t last_count;
  int32_t offset_counts;
  int64_t shadow_count;
  uint32_t last_update_us;
  float offset_rad;
  float mec_angle_rad;
  float elec_angle_rad;
  float velocity_rad_s;
  float vel_estimate_;
} Abz_Handle_t;

void Abz_Init(void);
void Abz_SetPolePairs(uint8_t pp);
void Abz_ResetCount(void);

extern Abz_Handle_t abz_data;
extern const Motor_HAL_EncoderInterface_t g_abz_encoder_interface;

#endif /* ABZ_ENCODER_H */
