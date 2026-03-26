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
    // calc
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
 * CRC32calc
 * ============================================================================ */
uint32_t BSP_Flash_CalculateCRC32(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return 0;
    }
#ifdef USE_HAL_CRC
    // STM32CRC
    extern CRC_HandleTypeDef hcrc;
    uint32_t word_count = length / 4;
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, word_count);
    return crc;
#else
    // CRC32 (CRC32-IEEE 802.3)
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
