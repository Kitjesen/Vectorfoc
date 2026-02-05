/**
 * @file    crc16.h
 * @brief   CRC-16 checksum calculation (standard & Modbus).
 */

#ifndef ALGORITHM_CRC16_H
#define ALGORITHM_CRC16_H

#include "main.h"

#define CRC_START_16 0xFFFF
#define CRC_START_MODBUS 0xFFFF
#define CRC_POLY_16 0xA001

/**
 * @brief  Calculate CRC-16 checksum.
 * @param  input_str  Input data buffer.
 * @param  num_bytes  Number of bytes to process.
 * @return CRC-16 value.
 */
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);

/**
 * @brief  Calculate Modbus CRC-16 checksum.
 * @param  input_str  Input data buffer.
 * @param  num_bytes  Number of bytes to process.
 * @return Modbus CRC-16 value.
 */
uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes);

/**
 * @brief  Update CRC-16 checksum incrementally.
 * @param  crc  Previous CRC value.
 * @param  c    New data byte.
 * @return Updated CRC-16 value.
 */
uint16_t update_crc_16(uint16_t crc, uint8_t c);

/**
 * @brief  Initialize CRC-16 lookup table (internal).
 */
void init_crc16_tab(void);

#endif // ALGORITHM_CRC16_H
