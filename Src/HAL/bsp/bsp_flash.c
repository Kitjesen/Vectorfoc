/**
 * @file bsp_flash.c
 * @brief Flash硬件抽象层实现
 */

#include "bsp_flash.h"
#include "main.h"
#include <string.h>

/* ============================================================================
 * Flash基本操作
 * ============================================================================ */

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
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    
    // 计算页号
    uint32_t page_num = (page_addr - BSP_FLASH_BASE) / BSP_FLASH_PAGE_SIZE;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = page_num;
    erase_init.NbPages = 1;
    
    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        return false;
    }
    
    return true;
}

bool BSP_Flash_WriteDoubleWord(uint32_t address, uint64_t data)
{
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) != HAL_OK) {
        return false;
    }
    return true;
}

void BSP_Flash_Read(uint32_t address, uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return;
    }
    
    memcpy(data, (void*)address, length);
}

bool BSP_Flash_Verify(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return false;
    }
    
    return (memcmp((void*)address, data, length) == 0);
}

/* ============================================================================
 * CRC32计算
 * ============================================================================ */

uint32_t BSP_Flash_CalculateCRC32(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return 0;
    }
    
#ifdef USE_HAL_CRC
    // 使用STM32硬件CRC
    extern CRC_HandleTypeDef hcrc;
    
    uint32_t word_count = length / 4;
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, word_count);
    
    return crc;
#else
    // 软件CRC32实现 (标准CRC32-IEEE 802.3)
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
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
#endif
}


