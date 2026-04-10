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
 * @file param_table.h
 * @brief param - configparam
 *
 * :
 *   - param()
 *   - param
 *   - param
 *
 * :
 *   const ParamEntry *entry = ParamTable_Find(PARAM_MOTOR_RS);
 *   if (entry && entry->type == PARAM_TYPE_FLOAT) { ... }
 */
#ifndef PARAM_TABLE_H
#define PARAM_TABLE_H
#include <stdbool.h>
#include <stdint.h>
/* ============================================================================
 * param ()
 * ============================================================================
 */
typedef enum {
  /* motorparam 0x2000-0x200F */
  PARAM_MOTOR_RS = 0x2000,         ///<  [Ω]
  PARAM_MOTOR_LS = 0x2001,         ///<  [H]
  PARAM_MOTOR_FLUX = 0x2002,       ///< flux [Wb]
  PARAM_MOTOR_POLE_PAIRS = 0x2003, ///< pole pairs
  /* PIDparam 0x2010-0x201F */
  PARAM_CUR_KP = 0x2010,        ///< currentKp
  PARAM_CUR_KI = 0x2011,        ///< currentKi
  PARAM_SPD_KP = 0x2012,        ///< speed/velocityKp
  PARAM_SPD_KI = 0x2013,        ///< speed/velocityKi
  PARAM_POS_KP = 0x2014,        ///< positionKp
  PARAM_LOC_KP = 0x2014,        ///< positionKp (，)
  PARAM_CUR_FILT_GAIN = 0x2015, ///< currentfilter
  PARAM_SPD_FILT_GAIN = 0x2016, ///< speed/velocityfilter
  /* limitparam 0x2020-0x202F */
  PARAM_LIMIT_TORQUE = 0x2020,  ///< limit [Nm]
  PARAM_LIMIT_CUR = 0x2021,     ///< currentlimit [A] ()
  PARAM_LIMIT_CURRENT = 0x2021, ///< currentlimit [A]
  PARAM_LIMIT_SPD = 0x2022,     ///< speed/velocitylimit [rad/s] ()
  PARAM_LIMIT_SPEED = 0x2022,   ///< speed/velocitylimit [rad/s]
  /* position/speed/velocitymodeparam 0x2030-0x203F */
  PARAM_VEL_MAX = 0x2030, ///< PPmodespeed/velocity
  PARAM_ACC_SET = 0x2031, ///< PPmodespeed/velocity
  PARAM_ACC_RAD = 0x2032, ///< speed/velocitymodespeed/velocity
  PARAM_INERTIA = 0x2033, ///<  [kg·m²]
  /* CANconfig 0x3000-0x300F */
  PARAM_CAN_ID = 0x3000,        ///< CAN ID
  PARAM_CAN_BAUDRATE = 0x3001,  ///< CAN
  PARAM_PROTOCOL_TYPE = 0x3002, ///<
  PARAM_CAN_TIMEOUT = 0x3003,   ///< CANtimeoutthreshold [ms]
  /* config 0x3010-0x301F */
  PARAM_ZERO_STA = 0x3010,   ///<
  PARAM_ADD_OFFSET = 0x3011, ///<  [rad]
  PARAM_DAMPER = 0x3012,     ///<
  PARAM_RUN_MODE = 0x3030,   ///< runningmode
  /* protectionconfig 0x3020-0x302F */
  PARAM_OV_THRESHOLD = 0x3020, ///< threshold [V]
  PARAM_UV_THRESHOLD = 0x3021, ///< threshold [V]
  PARAM_OC_THRESHOLD = 0x3022, ///< threshold [A]
  PARAM_OT_THRESHOLD = 0x3023, ///< threshold [℃]
  /* config 0x3040-0x305F */
  PARAM_SMO_ALPHA = 0x3040,    ///< SMO Alphagain
  PARAM_SMO_BETA = 0x3041,     ///< SMO Betagain
  PARAM_FF_FRICTION = 0x3042,  ///< feedforward
  PARAM_FW_MAX_CUR = 0x3043,   ///< current [A]
  PARAM_FW_START_VEL = 0x3044, ///< speed/velocity [rad/s]
  PARAM_COGGING_EN = 0x3045,   ///< enable (0.0/1.0)
  PARAM_COGGING_CALIB = 0x3046, ///< Anticogging calibration trigger (0/1)
  /* LADRC speed/velocityparam 0x3050-0x305F */
  PARAM_LADRC_ENABLE = 0x3050,  ///< LADRC enable (0.0=PID, 1.0=LADRC)
  PARAM_LADRC_OMEGA_O = 0x3051, ///< LADRC observer [rad/s]
  PARAM_LADRC_OMEGA_C = 0x3052, ///< LADRC  [rad/s]
  PARAM_LADRC_B0 = 0x3053,      ///< LADRC gain b0
  PARAM_LADRC_MAX_OUT = 0x3054, ///< LADRC output [A]
} ParamIndex;
/* ============================================================================
 * param
 * ============================================================================
 */
typedef enum {
  PARAM_TYPE_FLOAT,  ///<
  PARAM_TYPE_UINT8,  ///< 8
  PARAM_TYPE_UINT16, ///< 16
  PARAM_TYPE_UINT32, ///< 32
  PARAM_TYPE_INT32,  ///< 32
} ParamType;
/* param */
typedef enum {
  PARAM_ATTR_NONE = 0,
  PARAM_ATTR_READONLY = (1 << 0),   ///< param
  PARAM_ATTR_PERSISTENT = (1 << 1), ///< Flash
  PARAM_ATTR_RUNTIME = (1 << 2),    ///< running
} ParamAttr;
/* param () */
#define PARAM_ACCESS_R 0x01                               ///<
#define PARAM_ACCESS_W 0x02                               ///<
#define PARAM_ACCESS_RW (PARAM_ACCESS_R | PARAM_ACCESS_W) ///<
/* param */
typedef enum {
  PARAM_OK = 0,            ///<
  PARAM_ERR_INVALID_INDEX, ///< param
  PARAM_ERR_INVALID_TYPE,  ///<
  PARAM_ERR_READONLY,      ///< param
  PARAM_ERR_OUT_OF_RANGE,  ///<
  PARAM_ERR_NULL_PTR,      ///<
} ParamResult;
/* ============================================================================
 * param
 * ============================================================================
 */
typedef struct {
  uint16_t index;    ///< param
  ParamType type;    ///<
  uint8_t attr;      ///<  (PARAM_ATTR_*)
  uint8_t access;    ///<  (PARAM_ACCESS_*)
  const char *name;  ///< param
  void *ptr;         ///< (actualposition)
  float min;         ///<
  float max;         ///<
  float default_val; ///<
  bool need_save;    ///< Flash
} ParamEntry;
/* ============================================================================
 * param
 * ============================================================================
 */
/**
 * @brief initparam()
 */
void ParamTable_Init(void);
/**
 * @brief param
 * @param index param
 * @return param,NULL=
 */
const ParamEntry *ParamTable_Find(uint16_t index);
/**
 * @brief  getparam
 * @return param
 */
uint32_t ParamTable_GetCount(void);
/**
 * @brief getparam()
 * @return param
 */
const ParamEntry *ParamTable_GetTable(void);
#endif /* PARAM_TABLE_H */
