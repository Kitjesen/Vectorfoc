// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file bsp_flash.h
 * @brief Flash - STM32G4 Flash
 */
#ifndef BSP_FLASH_H
#define BSP_FLASH_H
#include "common.h"
#include <stdint.h>
#include <stdbool.h>
/* Flash configuration (STM32G431CB: 128KB, 64 pages, 2KB/page) */
#define BSP_FLASH_BASE              0x08000000u
#define BSP_FLASH_SIZE              (128u * 1024u)
#define BSP_FLASH_END               (BSP_FLASH_BASE + BSP_FLASH_SIZE)
#define BSP_FLASH_PAGE_SIZE         2048u
#define BSP_FLASH_PAGE_COUNT        (BSP_FLASH_SIZE / BSP_FLASH_PAGE_SIZE)
#define BSP_FLASH_DOUBLEWORD_SIZE   8u

/* Parameter storage pages: 0x0801F000-0x0801FFFF (4KB, pages 62-63). */
#define ADDR_FLASH_PAGE_62          0x0801F000u  ///< Page 62
#define ADDR_FLASH_PAGE_63          0x0801F800u  ///< Page 63
/**
 * @brief Flash
 */
void BSP_Flash_Unlock(void);
/**
 * @brief Flash
 */
void BSP_Flash_Lock(void);
/**
 * @brief Flash
 * @param page_addr
 * @return true=, false=
 */
bool BSP_Flash_ErasePage(uint32_t page_addr);
/**
 * @brief Flash(64)
 * @param address Flash
 * @param data 64
 * @return true=, false=
 */
bool BSP_Flash_WriteDoubleWord(uint32_t address, uint64_t data);
/**
 * @brief Flash
 * @param address Flash
 * @param data output
 * @param length ()
 */
void BSP_Flash_Read(uint32_t address, uint8_t *data, uint32_t length);
/**
 * @brief Flash
 * @param address Flash
 * @param data
 * @param length ()
 * @return true=, false=
 */
bool BSP_Flash_Verify(uint32_t address, const uint8_t *data, uint32_t length);
/**
 * @brief calcCRC32
 * @param data
 * @param length ()
 * @return CRC32
 */
uint32_t BSP_Flash_CalculateCRC32(const uint8_t *data, uint32_t length);
#endif /* BSP_FLASH_H */
