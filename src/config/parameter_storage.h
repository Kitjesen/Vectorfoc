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
 * @file parameter_storage.h
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
/* Flash (STM32G4，param) */
#define FLASH_PARAM_PAGE1_ADDR    0x0801F800  // 2 (2KB)
#define FLASH_PARAM_PAGE2_ADDR    0x0801FC00  // 1 (2KB, )
#define FLASH_PARAM_PAGE_SIZE     2048
/* param */
#define FLASH_MAGIC_WORD          0x464F4331  // "FOC1"
#define FLASH_PARAM_VERSION       0x00010000  // v1.0.0
/* Flash */
typedef struct {
    uint32_t magic;           // ，
    uint32_t version;         // param
    uint32_t crc32;           // CRC32
    uint32_t reserved;        //
    uint32_t generation;      // monotonically increasing write counter
    uint32_t committed;       // 0=write in progress, 1=write complete
    /* motorparam */
    float motor_rs;           //  [Ω]
    float motor_ls;           //  [H]
    float motor_flux;         // flux [Wb]
    uint8_t motor_pole_pairs; // pole pairs
    /* PIDparam */
    float cur_kp;             // currentKp
    float cur_ki;             // currentKi
    float spd_kp;             // speed/velocityKp
    float spd_ki;             // speed/velocityKi
    float pos_kp;             // positionKp
    float cur_filt_gain;      // currentfilter (，)
    float spd_filt_gain;      // speed/velocityfilter (，)
    /* limitparam */
    float limit_torque;       // limit [Nm]
    float limit_current;      // currentlimit [A]
    float limit_speed;        // speed/velocitylimit [rad/s]
    /* position/speed/velocitymodeparam */
    float vel_max;            // PPmodespeed/velocity
    float acc_set;            // PPmodespeed/velocity
    float acc_rad;            // speed/velocitymodespeed/velocity
    float inertia;            //  [kg·m²]
    /* CANconfig */
    uint8_t can_id;           // CAN ID
    uint8_t can_baudrate;     // CAN (0=1M, 1=500K, 2=250K)
    uint8_t protocol_type;    //  (0=, 1=CANopen, 2=MIT)
    /* config */
    uint8_t zero_sta;         //  (0: 0~2π, 1: -π~π)
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
    /*  (2KB) */
    uint8_t reserved_data[1568];
} __attribute__((packed)) FlashParamData;
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
/* These are the three operations used by the parameter access module.
 * FlashParamData above is a compatibility layout; reserved fields stay intact. */
FlashStorageResult ParamStorage_Save(const FlashParamData *data);
FlashStorageResult ParamStorage_Load(FlashParamData *data);
bool ParamStorage_HasValidData(void);
#endif /* PARAM_STORAGE_H */
