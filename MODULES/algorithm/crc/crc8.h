/**
 * @file    crc8.h
 * @brief   CRC-8 checksum calculation.
 */

#ifndef ALGORITHM_CRC8_H
#define ALGORITHM_CRC8_H

#include "main.h"

#define CRC_START_8 0x00

/**
 * @brief  Calculate CRC-8 checksum.
 * @param  input_str  Input data buffer.
 * @param  num_bytes  Number of bytes to process.
 * @return CRC-8 value.
 */
uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);

/**
 * @brief  Update CRC-8 checksum incrementally.
 * @param  crc  Previous CRC value.
 * @param  val  New data byte.
 * @return Updated CRC-8 value.
 */
uint8_t update_crc_8(uint8_t crc, uint8_t val);

#endif // ALGORITHM_CRC8_H
