/**
 * @file hal_encoder.h
 * @brief 编码器硬件抽象层接口
 * @note 提供统一的位置/速度获取接口
 */

#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 编码器接口结构体
 */
typedef struct {
  /**
   * @brief 初始化编码器
   */
  void (*init)(void);

  /**
   * @brief 更新编码器数据
   *
   * 在定时器中断中调用，更新位置和速度
   */
  void (*update)(void);

  /**
   * @brief 获取机械位置
   * @return 机械位置 [rad], 范围 [0, 2π]
   */
  float (*get_position)(void);

  /**
   * @brief 获取机械速度
   * @return 机械速度 [rad/s]
   */
  float (*get_velocity)(void);

  /**
   * @brief 获取电角度
   * @param pole_pairs 极对数
   * @return 电角度 [rad], 范围 [0, 2π]
   */
  float (*get_electrical_angle)(uint8_t pole_pairs);

  /**
   * @brief 获取电角速度
   * @param pole_pairs 极对数
   * @return 电角速度 [rad/s]
   */
  float (*get_electrical_velocity)(uint8_t pole_pairs);

  /**
   * @brief 设置零点偏移
   * @param offset 零点偏移 [rad]
   */
  void (*set_offset)(float offset);

  /**
   * @brief 获取零点偏移
   * @return 零点偏移 [rad]
   */
  float (*get_offset)(void);

} HAL_Encoder_Interface_t;

/**
 * @brief 注册编码器接口
 */
int MHAL_Encoder_Register(const HAL_Encoder_Interface_t *interface);
int MHAL_Encoder_Init(void);
int MHAL_Encoder_Update(void);
float MHAL_Encoder_GetPosition(void);
float MHAL_Encoder_GetVelocity(void);
float MHAL_Encoder_GetElectricalAngle(uint8_t pole_pairs);
float MHAL_Encoder_GetElectricalVelocity(uint8_t pole_pairs);
int MHAL_Encoder_SetOffset(float offset);
float MHAL_Encoder_GetOffset(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ENCODER_H */
