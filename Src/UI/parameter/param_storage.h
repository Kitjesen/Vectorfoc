/**
 * @file param_storage.h
 * @brief 参数Flash存储 - 双页备份机制
 * 
 * 职责:
 *   - Flash双页备份(Page1主+Page2备份)
 *   - CRC32数据完整性校验
 *   - 掉电保护和数据恢复
 * 
 * 存储机制:
 *   - 魔术字验证: 0x464F4331 ("FOC1")
 *   - CRC32校验确保数据完整性
 *   - 双页冗余,主页损坏自动从备份页恢复
 */

#ifndef PARAM_STORAGE_H
#define PARAM_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

/* Flash地址定义 (STM32G4系列，最后两页用于参数存储) */
#define FLASH_PARAM_PAGE1_ADDR    0x0801F800  // 倒数第2页 (2KB)
#define FLASH_PARAM_PAGE2_ADDR    0x0801FC00  // 倒数第1页 (2KB, 备份)
#define FLASH_PARAM_PAGE_SIZE     2048

/* 参数存储魔术字 */
#define FLASH_MAGIC_WORD          0x464F4331  // "FOC1"
#define FLASH_PARAM_VERSION       0x00010000  // v1.0.0

/* Flash存储数据结构 */
typedef struct {
    uint32_t magic;           // 魔术字，用于识别有效数据
    uint32_t version;         // 参数版本号
    uint32_t crc32;           // CRC32校验值
    uint32_t reserved;        // 预留字段
    
    /* 电机参数 */
    float motor_rs;           // 定子电阻 [Ω]
    float motor_ls;           // 定子电感 [H]
    float motor_flux;         // 磁链 [Wb]
    uint8_t motor_pole_pairs; // 极对数
    
    /* PID参数 */
    float cur_kp;             // 电流环Kp
    float cur_ki;             // 电流环Ki
    float spd_kp;             // 速度环Kp
    float spd_ki;             // 速度环Ki
    float pos_kp;             // 位置环Kp
    float cur_filt_gain;      // 电流滤波系数 (已废弃，保留兼容)
    float spd_filt_gain;      // 速度滤波系数 (已废弃，保留兼容)
    
    /* 限制参数 */
    float limit_torque;       // 转矩限制 [Nm]
    float limit_current;      // 电流限制 [A]
    float limit_speed;        // 速度限制 [rad/s]
    
    /* 位置/速度模式参数 */
    float vel_max;            // PP模式最大速度
    float acc_set;            // PP模式加速度
    float acc_rad;            // 速度模式加速度
    float inertia;            // 转动惯量 [kg·m²]
    
    /* CAN配置 */
    uint8_t can_id;           // CAN ID
    uint8_t can_baudrate;     // CAN波特率标志位 (0=1M, 1=500K, 2=250K)
    uint8_t protocol_type;    // 协议类型 (0=私有, 1=CANopen, 2=MIT)
    
    /* 功能配置 */
    uint8_t zero_sta;         // 零点标志位 (0: 0~2π, 1: -π~π)
    float add_offset;         // 零位偏置 [rad]
    uint8_t damper;           // 阻尼开关
    uint32_t can_timeout;     // CAN超时阈值 [ms]
    uint8_t run_mode;         // 运行模式
    
    /* 保护配置 */
    float over_voltage_threshold;      // 过压阈值
    float under_voltage_threshold;     // 欠压阈值
    float over_current_threshold;      // 过流阈值
    float over_temp_threshold;         // 过温阈值
    
    /* 高级控制配置 */
    float smo_alpha;                   // SMO Alpha增益
    float smo_beta;                    // SMO Beta增益
    float ff_friction;                 // 前馈摩擦系数
    float fw_max_current;              // 弱磁最大电流 [A]
    float fw_start_velocity;           // 弱磁起始速度 [rad/s]
    float cogging_comp_enabled;        // 齿槽转矩补偿使能 (0.0/1.0)
    
    /* 预留空间供将来扩展 (总共不超过2KB) */
    uint8_t reserved_data[1576];
    
} __attribute__((packed)) FlashParamData;

/* 存储结果 */
typedef enum {
    FLASH_STORAGE_OK = 0,
    FLASH_STORAGE_ERR_ERASE,
    FLASH_STORAGE_ERR_WRITE,
    FLASH_STORAGE_ERR_VERIFY,
    FLASH_STORAGE_ERR_CRC,
    FLASH_STORAGE_ERR_MAGIC,
    FLASH_STORAGE_ERR_VERSION,
    FLASH_STORAGE_ERR_LOCKED,
} FlashStorageResult;

/**
 * @brief 初始化参数存储模块
 */
void ParamStorage_Init(void);

/**
 * @brief 保存参数到Flash
 * @param data 参数数据结构
 * @return FlashStorageResult 存储结果
 */
FlashStorageResult ParamStorage_Save(const FlashParamData *data);

/**
 * @brief 从Flash加载参数
 * @param data 输出参数数据结构
 * @return FlashStorageResult 加载结果
 */
FlashStorageResult ParamStorage_Load(FlashParamData *data);

/**
 * @brief 擦除Flash中的参数
 * @return FlashStorageResult 操作结果
 */
FlashStorageResult ParamStorage_Erase(void);

/**
 * @brief 检查Flash中是否有有效参数
 * @return true=有效, false=无效
 */
bool ParamStorage_HasValidData(void);

/**
 * @brief 获取存储统计信息
 * @param write_count 输出写入次数
 * @param last_crc 输出最后一次CRC值
 */
void ParamStorage_GetStats(uint32_t *write_count, uint32_t *last_crc);

#endif /* PARAM_STORAGE_H */
