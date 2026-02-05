/**
 * @file hal_encoder.c
 * @brief 编码器硬件抽象层实现
 */

#include "hal_encoder.h"
#include <stddef.h>

/* 全局编码器接口指针 */
static const HAL_Encoder_Interface_t *g_encoder_interface = NULL;

/**
 * @brief 注册编码器接口
 */
int MHAL_Encoder_Register(const HAL_Encoder_Interface_t *interface) {
  if (interface == NULL)
    return -1;

  g_encoder_interface = interface;
  return 0;
}

int MHAL_Encoder_Init(void) {
  if (g_encoder_interface == NULL || g_encoder_interface->init == NULL)
    return -1;

  g_encoder_interface->init();
  return 0;
}

int MHAL_Encoder_Update(void) {
  if (g_encoder_interface == NULL || g_encoder_interface->update == NULL)
    return -1;

  g_encoder_interface->update();
  return 0;
}

float MHAL_Encoder_GetPosition(void) {
  if (g_encoder_interface == NULL || g_encoder_interface->get_position == NULL)
    return 0.0f;

  return g_encoder_interface->get_position();
}

float MHAL_Encoder_GetVelocity(void) {
  if (g_encoder_interface == NULL || g_encoder_interface->get_velocity == NULL)
    return 0.0f;

  return g_encoder_interface->get_velocity();
}

float MHAL_Encoder_GetElectricalAngle(uint8_t pole_pairs) {
  if (g_encoder_interface == NULL ||
      g_encoder_interface->get_electrical_angle == NULL)
    return 0.0f;

  return g_encoder_interface->get_electrical_angle(pole_pairs);
}

float MHAL_Encoder_GetElectricalVelocity(uint8_t pole_pairs) {
  if (g_encoder_interface == NULL ||
      g_encoder_interface->get_electrical_velocity == NULL)
    return 0.0f;

  return g_encoder_interface->get_electrical_velocity(pole_pairs);
}

int MHAL_Encoder_SetOffset(float offset) {
  if (g_encoder_interface == NULL || g_encoder_interface->set_offset == NULL)
    return -1;

  g_encoder_interface->set_offset(offset);
  return 0;
}

float MHAL_Encoder_GetOffset(void) {
  if (g_encoder_interface == NULL || g_encoder_interface->get_offset == NULL)
    return 0.0f;

  return g_encoder_interface->get_offset();
}
