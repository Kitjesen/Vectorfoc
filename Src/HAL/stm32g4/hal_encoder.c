/**
 * @file hal_encoder.c
 * @brief 编码器硬件抽象层实现
 * @note 统一 HAL 架构：委托给 motor_data.components.hal->encoder 和 encoder_data
 *       motor_data 和 encoder_data 是静态初始化的，程序启动时即有效
 */

#include "hal_encoder.h"
#include "mt6816_encoder.h"
#include "motor.h"

/**
 * @brief 注册编码器接口 (已废弃)
 * @note 统一 HAL 后不再需要注册，保留函数签名以兼容遗留代码
 */
int MHAL_Encoder_Register(const HAL_Encoder_Interface_t *interface) {
  (void)interface; // 忽略参数，不再使用
  return 0;        // 总是返回成功
}

int MHAL_Encoder_Init(void) {
  // 编码器初始化由 MT6816_Init 完成，通常在 main.c 中调用
  return 0;
}

int MHAL_Encoder_Update(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->encoder == NULL ||
      motor_data.components.hal->encoder->update == NULL)
    return -1;

  motor_data.components.hal->encoder->update();
  return 0;
}

float MHAL_Encoder_GetPosition(void) {
  // 直接访问 encoder_data 获取机械角度
  return encoder_data.mec_angle_rad;
}

float MHAL_Encoder_GetVelocity(void) {
  // 直接访问 encoder_data 获取速度
  return encoder_data.velocity_rad_s;
}

float MHAL_Encoder_GetElectricalAngle(uint8_t pole_pairs) {
  (void)pole_pairs; // encoder_data 内部已处理极对数
  return encoder_data.elec_angle_rad;
}

float MHAL_Encoder_GetElectricalVelocity(uint8_t pole_pairs) {
  // 电角速度 = 机械角速度 * 极对数
  return encoder_data.velocity_rad_s * pole_pairs;
}

int MHAL_Encoder_SetOffset(float offset) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->encoder == NULL ||
      motor_data.components.hal->encoder->set_offset == NULL)
    return -1;

  motor_data.components.hal->encoder->set_offset(offset);
  return 0;
}

float MHAL_Encoder_GetOffset(void) {
  // 直接访问 encoder_data 获取偏移量
  return encoder_data.offset_rev;
}
