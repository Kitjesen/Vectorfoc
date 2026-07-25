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
 * @file bsp_flash.c
 * @brief Flash
 */
#include "bsp_flash.h"
#include "main.h"
#include <string.h>
/* ============================================================================
 * Flash
 * ============================================================================ */
static bool BSP_Flash_IsRangeValid(uint32_t address, uint32_t length)
{
    if (length == 0u || address < BSP_FLASH_BASE || address >= BSP_FLASH_END) {
        return false;
    }

    uint32_t offset = address - BSP_FLASH_BASE;
    return length <= (BSP_FLASH_SIZE - offset);
}

void BSP_Flash_Unlock(void)
{
    HAL_FLASH_Unlock();
}
void BSP_Flash_Lock(void)
{
    HAL_FLASH_Lock();
}
bool BSP_Flash_ErasePage(uint32_t page_addr)
{
    if (((page_addr % BSP_FLASH_PAGE_SIZE) != 0u) ||
        !BSP_Flash_IsRangeValid(page_addr, BSP_FLASH_PAGE_SIZE)) {
        return false;
    }

    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_error = 0xFFFFFFFFu;
    uint32_t page_num = (page_addr - BSP_FLASH_BASE) / BSP_FLASH_PAGE_SIZE;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = page_num;
    erase_init.NbPages = 1;
    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        return false;
    }
    return true;
}
bool BSP_Flash_WriteDoubleWord(uint32_t address, uint64_t data)
{
    if (((address % BSP_FLASH_DOUBLEWORD_SIZE) != 0u) ||
        !BSP_Flash_IsRangeValid(address, BSP_FLASH_DOUBLEWORD_SIZE)) {
        return false;
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) != HAL_OK) {
        return false;
    }
    return true;
}
void BSP_Flash_Read(uint32_t address, uint8_t *data, uint32_t length)
{
    if (data == NULL || !BSP_Flash_IsRangeValid(address, length)) {
        return;
    }
    memcpy(data, (const void*)(uintptr_t)address, length);
}
bool BSP_Flash_Verify(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (data == NULL || !BSP_Flash_IsRangeValid(address, length)) {
        return false;
    }
    return (memcmp((const void*)(uintptr_t)address, data, length) == 0);
}
/* ============================================================================
 * CRC32calc
 * ============================================================================ */
uint32_t BSP_Flash_CalculateCRC32(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0u) {
        return 0;
    }

    /* CRC32/IEEE 802.3: init=0xFFFFFFFF, refin/refout=true, xorout=0xFFFFFFFF. */
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc & 1u) != 0u) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}
