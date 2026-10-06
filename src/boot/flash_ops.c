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
 * @file flash_ops.c
 * @brief Bootloader Flash 操作实现
 */
#include "flash_ops.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* ============================================================================
 * Flash 解锁/锁定
 * ============================================================================ */
void Flash_Unlock(void)
{
    HAL_FLASH_Unlock();
}

void Flash_Lock(void)
{
    HAL_FLASH_Lock();
}

/* ============================================================================
 * Flash 擦除
 * ============================================================================ */
BootStatus_t Flash_ErasePage(uint32_t page_num)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = page_num;
    erase_init.NbPages = 1;
    erase_init.Banks = FLASH_BANK_1;

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        return BOOT_ERR_ERASE_FAIL;
    }

    return BOOT_OK;
}

BootStatus_t Flash_EraseAppArea(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = APP_PAGE_START;
    erase_init.NbPages = APP_PAGE_COUNT;
    erase_init.Banks = FLASH_BANK_1;

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        return BOOT_ERR_ERASE_FAIL;
    }

    return BOOT_OK;
}

/* ============================================================================
 * Flash 写入
 * ============================================================================ */
BootStatus_t Flash_WriteData(uint32_t addr, const uint8_t *data, uint32_t len)
{
    /* 检查地址对齐 */
    if ((addr & 0x07) != 0) {
        return BOOT_ERR_INVALID_ADDR;
    }

    /* 检查地址范围 */
    if (!Flash_IsAddrInAppArea(addr, len)) {
        return BOOT_ERR_INVALID_ADDR;
    }

    /* 按 8 字节 (double word) 写入 */
    uint32_t offset = 0;
    while (offset < len) {
        uint64_t dword;
        
        /* 处理不足 8 字节的情况 */
        if (len - offset >= 8) {
            memcpy(&dword, data + offset, 8);
        } else {
            /* 填充 0xFF */
            dword = 0xFFFFFFFFFFFFFFFF;
            memcpy(&dword, data + offset, len - offset);
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              addr + offset, dword) != HAL_OK) {
            return BOOT_ERR_WRITE_FAIL;
        }

        offset += 8;
    }

    /* 验证写入 */
    if (memcmp((void *)addr, data, len) != 0) {
        return BOOT_ERR_VERIFY_FAIL;
    }

    return BOOT_OK;
}

/* ============================================================================
 * Flash 读取
 * ============================================================================ */
void Flash_ReadData(uint32_t addr, uint8_t *data, uint32_t len)
{
    memcpy(data, (void *)addr, len);
}

/* ============================================================================
 * CRC32 计算 (IEEE 802.3 多项式)
 * ============================================================================ */
uint32_t Flash_CalcCRC32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}

uint32_t Flash_CalcFlashCRC32(uint32_t addr, uint32_t len)
{
    return Flash_CalcCRC32((const uint8_t *)addr, len);
}

/* ============================================================================
 * 地址检查
 * ============================================================================ */
bool Flash_IsAddrInAppArea(uint32_t addr, uint32_t len)
{
    if (addr < APP_ADDR_START) {
        return false;
    }
    if (addr + len > APP_ADDR_END + 1) {
        return false;
    }
    return true;
}
