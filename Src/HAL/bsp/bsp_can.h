/**
 * @file bsp_can.h
 * @brief CAN
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
/* CAN（） */
#define FDCAN_MX_REGISTER_CNT 6
#define DEVICE_CAN_CNT 1
/**
 * @brief FDCAN
 */
typedef struct fdcaninstance {
  FDCAN_HandleTypeDef *fdcan_handle; // CAN
  FDCAN_TxHeaderTypeDef txconf;      // config
  uint32_t tx_id;                    // ID
  uint32_t tx_mailbox;               //
  uint8_t tx_buff[8];                //
  uint8_t rx_buff[8];                //
  uint32_t rx_id;                    // ID
  uint8_t rx_len;                    //
  void (*fdcan_module_callback)(struct fdcaninstance *); // （）
  void *id;                                              // ID
} FDCANInstance;
/**
 * @brief CANinitconfig（）
 */
typedef struct {
  FDCAN_HandleTypeDef *fdcan_handle;
  uint32_t tx_id;
  uint32_t rx_id;
  void (*fdcan_module_callback)(FDCANInstance *);
  void *id;
} FDCAN_Init_Config_s;
/* ==========  ========== */
/**
 * @brief initCAN（）
 * @note RobotInit
 */
void BSP_CAN_Init(void);
/**
 * @brief CAN ()
 * @param frame CAN
 * @return true，false
 */
bool BSP_CAN_SendFrame(const CAN_Frame *frame);
/* ========== （） ========== */
/**
 * @brief CAN
 * @param config configparam
 * @return CAN
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config);
/**
 * @brief set
 * @param _instance CAN
 * @param length （0-8）
 */
void FDCANSetDLC(FDCANInstance *_instance, uint8_t length);
/**
 * @brief CAN
 * @param _instance CAN
 * @param timeout timeout（ms）
 * @return 1，0
 */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout);
#ifdef __cplusplus
}
#endif
#endif
