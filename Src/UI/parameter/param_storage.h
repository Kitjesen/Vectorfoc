// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file param_storage.h
 * @brief paramFlash -
 *
 * :
 *   - Flash(Page1+Page2)
 *   - CRC32
 *   - protection
 *
 * :
 *   - : 0x464F4331 ("FOC1")
 *   - CRC32
 *   - ,
 */
#ifndef PARAM_STORAGE_H
#define PARAM_STORAGE_H
#include <stdint.h>
#include <stdbool.h>
/* Flash (STM32G4?param) */
/* Keep the transactional copies on different physical erase pages. */
#define FLASH_PARAM_PAGE1_ADDR    0x0801F000u /* Page 62 */
#define FLASH_PARAM_PAGE2_ADDR    0x0801F800u /* Page 63 */
#define FLASH_PARAM_PAGE_SIZE     2048u
#define PARAM_FLASH_IMAGE_SIZE    FLASH_PARAM_PAGE_SIZE

typedef char FlashParamPagesMustDiffer[(FLASH_PARAM_PAGE2_ADDR - FLASH_PARAM_PAGE1_ADDR == FLASH_PARAM_PAGE_SIZE) ? 1 : -1];
/* param */
#define FLASH_MAGIC_WORD          0x464F4331  // "FOC1"
#define FLASH_MAGIC_WORD_V2       0x464F4332  // "FOC2"
#define FLASH_PARAM_VERSION       0x00010000  // v1.0.0
typedef uint8_t FlashPageIndex;  // 0=Page1, 1=Page2
/* Flash */
#if defined(_MSC_VER)
#pragma pack(push, 1)
#define PARAM_STORAGE_PACKED
#else
#define PARAM_STORAGE_PACKED __attribute__((packed))
#endif
typedef struct {
    uint32_t magic;           // ?
    uint32_t param_version;   // param
    uint32_t crc32;           // CRC32
    uint32_t reserved;        //
    uint32_t generation;      // monotonically increasing write counter
    uint32_t committed;       // 0=committed, 0xFFFFFFFF=pending/erased
    /* motorparam */
    float motor_rs;           //  [?]
    float motor_ls;           //  [H]
    float motor_flux;         // flux [Wb]
    uint8_t motor_pole_pairs; // pole pairs
    /* PIDparam */
    float cur_kp;             // currentKp
    float cur_ki;             // currentKi
    float spd_kp;             // speed/velocityKp
    float spd_ki;             // speed/velocityKi
    float pos_kp;             // positionKp
    float cur_filt_gain;      // currentfilter (?)
    float spd_filt_gain;      // speed/velocityfilter (?)
    /* limitparam */
    float limit_torque;       // limit [Nm]
    float limit_current;      // currentlimit [A]
    float limit_speed;        // speed/velocitylimit [rad/s]
    /* position/speed/velocitymodeparam */
    float vel_max;            // PPmodespeed/velocity
    float acc_set;            // PPmodespeed/velocity
    float acc_rad;            // speed/velocitymodespeed/velocity
    float inertia;            //  [kg?m?]
    /* CANconfig */
    uint8_t can_id;           // CAN ID
    uint8_t can_baudrate;     // CAN (0=1M, 1=500K, 2=250K)
    uint8_t protocol_type;    //  (0=, 1=CANopen, 2=MIT)
    /* config */
    uint8_t zero_sta;         //  (0: 0~2?, 1: -?~?)
    float add_offset;         //  [rad]
    uint8_t damper;           //
    uint32_t can_timeout;     // CANtimeoutthreshold [ms]
    uint8_t run_mode;         // runningmode
    /* protectionconfig */
    float over_voltage_threshold;      // threshold
    float under_voltage_threshold;     // threshold
    float over_current_threshold;      // threshold
    float over_temp_threshold;         // threshold
    /* config */
    float smo_alpha;                   // SMO Alphagain
    float smo_beta;                    // SMO Betagain
    float ff_friction;                 // feedforward
    float fw_max_current;              // current [A]
    float fw_start_velocity;           // speed/velocity [rad/s]
    float cogging_comp_enabled;        // enable (0.0/1.0)
    float ladrc_enable;                // LADRC enable (0.0=PID, 1.0=LADRC)
    float ladrc_omega_o;               // LADRC observer bandwidth [rad/s]
    float ladrc_omega_c;               // LADRC cutoff/bandwidth [rad/s]
    float ladrc_b0;                    // LADRC gain b0
    float ladrc_max_output;            // LADRC output max [A]
    uint8_t encoder_calib_valid;       // encoder LUT/offset calibration valid
    uint8_t encoder_calib_reserved[3]; // alignment/reserved
    int16_t encoder_offset_lut[128];   // encoder linearity compensation LUT
} PARAM_STORAGE_PACKED FlashParamData;
#if defined(_MSC_VER)
#pragma pack(pop)
#endif
#undef PARAM_STORAGE_PACKED
/*  */
typedef enum {
    FLASH_STORAGE_OK = 0,
    FLASH_STORAGE_ERR_ERASE,
    FLASH_STORAGE_ERR_WRITE,
    FLASH_STORAGE_ERR_VERIFY,
    FLASH_STORAGE_ERR_CRC,
    FLASH_STORAGE_ERR_MAGIC,
    FLASH_STORAGE_ERR_VERSION,
    FLASH_STORAGE_ERR_LOCKED,
    FLASH_STORAGE_ERR_CORRUPT,
} FlashStorageResult;
/**
 * @brief initparam
 */
void ParamStorage_Init(void);
/**
 * @brief paramFlash
 * @param data param
 * @return FlashStorageResult
 */
FlashStorageResult ParamStorage_Save(FlashParamData *data);
/**
 * @brief Flashparam
 * @param data outputparam
 * @return FlashStorageResult
 */
FlashStorageResult ParamStorage_Load(FlashParamData *data);
/**
 * @brief Flashparam
 * @return FlashStorageResult
 */
FlashStorageResult ParamStorage_Erase(void);
/**
 * @brief checkFlashparam
 * @return true=, false=
 */
bool ParamStorage_HasValidData(void);
/**
 * @brief get
 * @param write_count output
 * @param last_crc outputCRC
 */
void ParamStorage_GetStats(uint32_t *write_count, uint32_t *last_crc);
FlashPageIndex ParamStorage_GetActivePage(void);
#endif /* PARAM_STORAGE_H */
