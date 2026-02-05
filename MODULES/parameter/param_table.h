/**
 * @file param_table.h
 * @brief 参数表定义 - 所有可配置参数的索引和元数据
 *
 * 职责:
 *   - 定义参数索引枚举(避免魔术数字)
 *   - 定义参数类型和访问属性
 *   - 提供参数查找和验证接口
 *
 * 使用示例:
 *   const ParamEntry *entry = ParamTable_Find(PARAM_MOTOR_RS);
 *   if (entry && entry->type == PARAM_TYPE_FLOAT) { ... }
 */

#ifndef PARAM_TABLE_H
#define PARAM_TABLE_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * 参数索引定义 (使用枚举避免魔术数字)
 * ============================================================================
 */

typedef enum {
  /* 电机参数 0x2000-0x200F */
  PARAM_MOTOR_RS = 0x2000,         ///< 定子电阻 [Ω]
  PARAM_MOTOR_LS = 0x2001,         ///< 定子电感 [H]
  PARAM_MOTOR_FLUX = 0x2002,       ///< 磁链 [Wb]
  PARAM_MOTOR_POLE_PAIRS = 0x2003, ///< 极对数

  /* PID参数 0x2010-0x201F */
  PARAM_CUR_KP = 0x2010,        ///< 电流环Kp
  PARAM_CUR_KI = 0x2011,        ///< 电流环Ki
  PARAM_SPD_KP = 0x2012,        ///< 速度环Kp
  PARAM_SPD_KI = 0x2013,        ///< 速度环Ki
  PARAM_POS_KP = 0x2014,        ///< 位置环Kp
  PARAM_LOC_KP = 0x2014,        ///< 位置环Kp (别名，兼容旧代码)
  PARAM_CUR_FILT_GAIN = 0x2015, ///< 电流滤波系数
  PARAM_SPD_FILT_GAIN = 0x2016, ///< 速度滤波系数

  /* 限制参数 0x2020-0x202F */
  PARAM_LIMIT_TORQUE = 0x2020,  ///< 转矩限制 [Nm]
  PARAM_LIMIT_CUR = 0x2021,     ///< 电流限制 [A] (别名)
  PARAM_LIMIT_CURRENT = 0x2021, ///< 电流限制 [A]
  PARAM_LIMIT_SPD = 0x2022,     ///< 速度限制 [rad/s] (别名)
  PARAM_LIMIT_SPEED = 0x2022,   ///< 速度限制 [rad/s]

  /* 位置/速度模式参数 0x2030-0x203F */
  PARAM_VEL_MAX = 0x2030, ///< PP模式最大速度
  PARAM_ACC_SET = 0x2031, ///< PP模式加速度
  PARAM_ACC_RAD = 0x2032, ///< 速度模式加速度
  PARAM_INERTIA = 0x2033, ///< 转动惯量 [kg·m²]

  /* CAN配置 0x3000-0x300F */
  PARAM_CAN_ID = 0x3000,        ///< CAN ID
  PARAM_CAN_BAUDRATE = 0x3001,  ///< CAN波特率
  PARAM_PROTOCOL_TYPE = 0x3002, ///< 协议类型
  PARAM_CAN_TIMEOUT = 0x3003,   ///< CAN超时阈值 [ms]

  /* 功能配置 0x3010-0x301F */
  PARAM_ZERO_STA = 0x3010,   ///< 零点标志位
  PARAM_ADD_OFFSET = 0x3011, ///< 零位偏置 [rad]
  PARAM_DAMPER = 0x3012,     ///< 阻尼开关
  PARAM_RUN_MODE = 0x3030,   ///< 运行模式

  /* 保护配置 0x3020-0x302F */
  PARAM_OV_THRESHOLD = 0x3020, ///< 过压阈值 [V]
  PARAM_UV_THRESHOLD = 0x3021, ///< 欠压阈值 [V]
  PARAM_OC_THRESHOLD = 0x3022, ///< 过流阈值 [A]
  PARAM_OT_THRESHOLD = 0x3023, ///< 过温阈值 [℃]

  /* 高级控制配置 0x3040-0x305F */
  PARAM_SMO_ALPHA = 0x3040,    ///< SMO Alpha增益
  PARAM_SMO_BETA = 0x3041,     ///< SMO Beta增益
  PARAM_FF_FRICTION = 0x3042,  ///< 前馈摩擦系数
  PARAM_FW_MAX_CUR = 0x3043,   ///< 弱磁最大电流 [A]
  PARAM_FW_START_VEL = 0x3044, ///< 弱磁起始速度 [rad/s]
  PARAM_COGGING_EN = 0x3045,   ///< 齿槽转矩补偿使能 (0.0/1.0)
  PARAM_COGGING_CALIB = 0x3046, ///< Anticogging calibration trigger (0/1)

} ParamIndex;

/* ============================================================================
 * 参数类型定义
 * ============================================================================
 */

typedef enum {
  PARAM_TYPE_FLOAT,  ///< 浮点数
  PARAM_TYPE_UINT8,  ///< 8位无符号整数
  PARAM_TYPE_UINT16, ///< 16位无符号整数
  PARAM_TYPE_UINT32, ///< 32位无符号整数
  PARAM_TYPE_INT32,  ///< 32位有符号整数
} ParamType;

/* 参数属性标志 */
typedef enum {
  PARAM_ATTR_NONE = 0,
  PARAM_ATTR_READONLY = (1 << 0),   ///< 只读参数
  PARAM_ATTR_PERSISTENT = (1 << 1), ///< 可持久化到Flash
  PARAM_ATTR_RUNTIME = (1 << 2),    ///< 仅运行时有效
} ParamAttr;

/* 参数访问权限 (兼容旧代码) */
#define PARAM_ACCESS_R 0x01                               ///< 可读
#define PARAM_ACCESS_W 0x02                               ///< 可写
#define PARAM_ACCESS_RW (PARAM_ACCESS_R | PARAM_ACCESS_W) ///< 可读写

/* 参数操作结果 */
typedef enum {
  PARAM_OK = 0,            ///< 操作成功
  PARAM_ERR_INVALID_INDEX, ///< 无效的参数索引
  PARAM_ERR_INVALID_TYPE,  ///< 类型不匹配
  PARAM_ERR_READONLY,      ///< 尝试写入只读参数
  PARAM_ERR_OUT_OF_RANGE,  ///< 值超出范围
  PARAM_ERR_NULL_PTR,      ///< 空指针
} ParamResult;

/* ============================================================================
 * 参数条目定义
 * ============================================================================
 */

typedef struct {
  uint16_t index;    ///< 参数索引
  ParamType type;    ///< 数据类型
  uint8_t attr;      ///< 属性标志 (PARAM_ATTR_*)
  uint8_t access;    ///< 访问权限 (PARAM_ACCESS_*)
  const char *name;  ///< 参数名称
  void *ptr;         ///< 数据指针(指向实际存储位置)
  float min;         ///< 最小值
  float max;         ///< 最大值
  float default_val; ///< 默认值
  bool need_save;    ///< 是否需要保存到Flash
} ParamEntry;

/* ============================================================================
 * 参数表接口
 * ============================================================================
 */

/**
 * @brief 初始化参数表(加载默认值)
 */
void ParamTable_Init(void);

/**
 * @brief 查找参数条目
 * @param index 参数索引
 * @return 参数条目指针,NULL=未找到
 */
const ParamEntry *ParamTable_Find(uint16_t index);

/**
 * @brief  获取参数表总数
 * @return 参数数量
 */
uint32_t ParamTable_GetCount(void);

/**
 * @brief 获取参数表指针(用于遍历)
 * @return 参数表首地址
 */
const ParamEntry *ParamTable_GetTable(void);

#endif /* PARAM_TABLE_H */
