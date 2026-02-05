/**
 * @file bsp_can.h
 * @brief CAN总线硬件抽象层接口声明
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "common.h"
#include "fdcan.h"
#include "main.h"
#include "protocol_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CAN设备最大数量（兼容旧代码） */
#define FDCAN_MX_REGISTER_CNT 6
#define DEVICE_CAN_CNT 1

/**
 * @brief FDCAN实例结构体
 */
typedef struct fdcaninstance {
  FDCAN_HandleTypeDef *fdcan_handle; // CAN句柄
  FDCAN_TxHeaderTypeDef txconf;      // 发送配置
  uint32_t tx_id;                    // 发送ID
  uint32_t tx_mailbox;               // 发送邮箱号
  uint8_t tx_buff[8];                // 发送缓存
  uint8_t rx_buff[8];                // 接收缓存
  uint32_t rx_id;                    // 接收ID
  uint8_t rx_len;                    // 接收长度

  void (*fdcan_module_callback)(struct fdcaninstance *); // 接收回调（兼容）
  void *id;                                              // 模块ID
} FDCANInstance;

/**
 * @brief CAN实例初始化配置（兼容接口）
 */
typedef struct {
  FDCAN_HandleTypeDef *fdcan_handle;
  uint32_t tx_id;
  uint32_t rx_id;
  void (*fdcan_module_callback)(FDCANInstance *);
  void *id;
} FDCAN_Init_Config_s;

/* ========== 公共接口 ========== */

/**
 * @brief 初始化CAN服务（硬件和过滤器）
 * @note 在RobotInit中手动调用
 */
void BSP_CAN_Init(void);

/**
 * @brief 发送CAN帧 (抽象接口)
 * @param frame CAN协议帧
 * @return true成功，false失败
 */
bool BSP_CAN_SendFrame(const CAN_Frame *frame);

/* ========== 兼容接口（旧代码） ========== */

/**
 * @brief 注册模块到CAN服务
 * @param config 配置参数
 * @return CAN实例指针
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config);

/**
 * @brief 设置数据长度
 * @param _instance CAN实例
 * @param length 数据长度（0-8字节）
 */
void FDCANSetDLC(FDCANInstance *_instance, uint8_t length);

/**
 * @brief 通过CAN发送消息
 * @param _instance CAN实例
 * @param timeout 超时时间（ms）
 * @return 1成功，0失败
 */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout);

#ifdef __cplusplus
}
#endif

#endif
