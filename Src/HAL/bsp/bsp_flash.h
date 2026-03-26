/**
 * @file bsp_flash.h
 * @brief Flash - STM32G4 Flash
 */
#ifndef BSP_FLASH_H
#define BSP_FLASH_H
#include "common.h"
#include <stdint.h>
#include <stdbool.h>
/* Flashconfigparam(STM32G4) */
#define BSP_FLASH_BASE          0x08000000
#define BSP_FLASH_PAGE_SIZE     2048        ///< 2KB
/* Flash */
#define ADDR_FLASH_PAGE_62      0x0801F000  ///< Page 62 (2)
#define ADDR_FLASH_PAGE_63      0x0801F800  ///< Page 63 (1)
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
