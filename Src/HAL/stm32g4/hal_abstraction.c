#include "hal_abstraction.h"

#ifdef USE_HAL_DRIVER
#include "board_config.h"
#include "main.h"

/* ADC temperature reading */
#include "motor_adc.h"
#include "bsp_adc.h"
#endif

/**
 * @brief 获取系统时钟滴答数
 */
uint32_t HAL_GetSystemTick(void) {
#ifdef USE_HAL_DRIVER
  return HAL_GetTick();
#else
  /* 如果没有HAL库，使用自定义计数器 */
  static volatile uint32_t tick_counter = 0;
  return tick_counter;
#endif
}

/**
 * @brief 获取微秒级时间戳
 */
uint32_t HAL_GetMicroseconds(void) {
#ifdef USE_HAL_DRIVER
  /* 使用DWT（Data Watchpoint and Trace）计数器 */
  return DWT->CYCCNT / (SystemCoreClock / 1000000);
#else
  /* 退化到毫秒级 */
  return HAL_GetSystemTick() * 1000;
#endif
}

/**
 * @brief 发送CAN帧
 */
bool HAL_CAN_Transmit(const CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

#ifdef USE_HAL_DRIVER
  FDCAN_TxHeaderTypeDef tx_header;

  /* 配置CAN ID */
  tx_header.Identifier = frame->id;
  if (frame->is_extended) {
    tx_header.IdType = FDCAN_EXTENDED_ID;
  } else {
    tx_header.IdType = FDCAN_STANDARD_ID;
  }

  /* 配置RTR标志 */
  tx_header.TxFrameType = frame->is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;

  /* 数据长度 */
  // Map uint8_t DLC to FDCAN_DLC_BYTES_x
  switch (frame->dlc) {
  case 0:
    tx_header.DataLength = FDCAN_DLC_BYTES_0;
    break;
  case 1:
    tx_header.DataLength = FDCAN_DLC_BYTES_1;
    break;
  case 2:
    tx_header.DataLength = FDCAN_DLC_BYTES_2;
    break;
  case 3:
    tx_header.DataLength = FDCAN_DLC_BYTES_3;
    break;
  case 4:
    tx_header.DataLength = FDCAN_DLC_BYTES_4;
    break;
  case 5:
    tx_header.DataLength = FDCAN_DLC_BYTES_5;
    break;
  case 6:
    tx_header.DataLength = FDCAN_DLC_BYTES_6;
    break;
  case 7:
    tx_header.DataLength = FDCAN_DLC_BYTES_7;
    break;
  case 8:
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    break;
  default:
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    break;
  }

  /* 固定传输类型 */
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  /* 发送CAN消息 */
  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(
      &HW_CAN, &tx_header, (uint8_t *)frame->data);

  return (status == HAL_OK);
#else
  /* 仿真模式：总是返回成功 */
  (void)frame;
  return true;
#endif
}

/**
 * @brief 检查CAN发送邮箱是否可用
 */
bool HAL_CAN_IsTxMailboxAvailable(void) {
#ifdef USE_HAL_DRIVER
  return (HAL_FDCAN_GetTxFifoFreeLevel(&HW_CAN) > 0);
#else
  return true;
#endif
}

/**
 * @brief 获取温度传感器读数
 * @return 温度值（摄氏度）
 * 
 * @note 使用ADC2_IN12(PB2)采集NTC热敏电阻电压，通过查表转换为温度
 */
float HAL_GetTemperature(void) {
#ifdef USE_HAL_DRIVER
  /* 读取ADC2温度通道（通道0对应adc2_ch12）并做均值滤波 */
  uint16_t adc_raw = adc2_avg_filter(adc2_ch12);
  
  /* 通过NTC查表转换为实际温度 */
  float temp;
  GetTempNtc(adc_raw, &temp);
  
  return temp;
#else
  return 25.0f;
#endif
}

/**
 * @brief 延时函数
 */
void HAL_Delay(uint32_t ms) {
#ifdef USE_HAL_DRIVER
  HAL_Delay(ms);
#else
  /* 简单的延时循环（不精确） */
  for (uint32_t i = 0; i < ms * 1000; i++) {
    __NOP();
  }
#endif
}

/**
 * @brief 进入临界区
 */
uint32_t HAL_EnterCritical(void) {
#ifdef USE_HAL_DRIVER
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
#else
  return 0;
#endif
}

/**
 * @brief 退出临界区
 */
void HAL_ExitCritical(uint32_t prev_state) {
#ifdef USE_HAL_DRIVER
  __set_PRIMASK(prev_state);
#else
  (void)prev_state;
#endif
}

/**
 * @brief 喂独立看门狗
 * @note 写入 0xAAAA 刷新看门狗计数器
 */
void HAL_WatchdogFeed(void) {
#ifdef USE_HAL_DRIVER
  IWDG->KR = 0xAAAA;
#else
  /* 仿真模式：不做任何操作 */
#endif
}
