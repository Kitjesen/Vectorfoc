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
 * @file parameter_storage.c
 * @brief paramFlash
 */
#include "parameter_storage.h"
#include "board_flash.h"
#include <string.h>
static bool Flash_Write(uint32_t address, const uint8_t *data, uint32_t length)
{
    // STM32G4(64)
    uint64_t *src = (uint64_t*)data;
    uint32_t doubleword_count = (length + 7) / 8;  //
    for (uint32_t i = 0; i < doubleword_count; i++) {
        uint64_t value = (i * 8 < length) ? src[i] : 0xFFFFFFFFFFFFFFFF;
        if (!BSP_Flash_WriteDoubleWord(address + i * 8, value)) {
            return false;
        }
    }
    return true;
}

FlashStorageResult ParamStorage_Save(const FlashParamData *data)
{
    if (data == NULL) {
        return FLASH_STORAGE_ERR_LOCKED;
    }
    //
    FlashParamData flash_data = *data;
    flash_data.magic = FLASH_MAGIC_WORD;
    flash_data.version = FLASH_PARAM_VERSION;
    flash_data.crc32 = 0;  // CRCcalc
    // calcCRC (magic, version, crc32)
    uint8_t *crc_start = ((uint8_t*)&flash_data) + 16;  // 4uint32
    uint32_t crc_length = sizeof(FlashParamData) - 16;
    flash_data.crc32 = BSP_Flash_CalculateCRC32(crc_start, crc_length);
    BSP_Flash_Unlock();
    // Page1
    if (!BSP_Flash_ErasePage(FLASH_PARAM_PAGE1_ADDR)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_ERASE;
    }
    if (!Flash_Write(FLASH_PARAM_PAGE1_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData))) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_WRITE;
    }
    //
    if (!BSP_Flash_Verify(FLASH_PARAM_PAGE1_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData))) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_VERIFY;
    }
    // Page2
    if (!BSP_Flash_ErasePage(FLASH_PARAM_PAGE2_ADDR)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_ERASE;
    }
    if (!Flash_Write(FLASH_PARAM_PAGE2_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData))) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_WRITE;
    }
    BSP_Flash_Lock();
    return FLASH_STORAGE_OK;
}

FlashStorageResult ParamStorage_Load(FlashParamData *data)
{
    if (data == NULL) {
        return FLASH_STORAGE_ERR_LOCKED;
    }
    FlashParamData flash_data;
    // Page1
    BSP_Flash_Read(FLASH_PARAM_PAGE1_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData));
    // check
    if (flash_data.magic != FLASH_MAGIC_WORD) {
        // Page1，Page2
        BSP_Flash_Read(FLASH_PARAM_PAGE2_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData));
        if (flash_data.magic != FLASH_MAGIC_WORD) {
            return FLASH_STORAGE_ERR_MAGIC;
        }
    }
    // check ()
    if (flash_data.version != FLASH_PARAM_VERSION) {
        // ，
        // return FLASH_STORAGE_ERR_VERSION;
    }
    // CRC
    uint32_t stored_crc = flash_data.crc32;
    flash_data.crc32 = 0;
    uint8_t *crc_start = ((uint8_t*)&flash_data) + 16;
    uint32_t crc_length = sizeof(FlashParamData) - 16;
    uint32_t calculated_crc = BSP_Flash_CalculateCRC32(crc_start, crc_length);
    if (stored_crc != calculated_crc) {
        return FLASH_STORAGE_ERR_CRC;
    }
    // CRC
    flash_data.crc32 = stored_crc;
    //
    *data = flash_data;
    return FLASH_STORAGE_OK;
}

bool ParamStorage_HasValidData(void)
{
    FlashParamData temp;
    BSP_Flash_Read(FLASH_PARAM_PAGE1_ADDR, (uint8_t*)&temp, sizeof(uint32_t));
    if (temp.magic == FLASH_MAGIC_WORD) {
        return true;
    }
    // check
    BSP_Flash_Read(FLASH_PARAM_PAGE2_ADDR, (uint8_t*)&temp, sizeof(uint32_t));
    return (temp.magic == FLASH_MAGIC_WORD);
}
