/**
 * @file bsp_flash.h
 * @brief Flash硬件抽象层 - STM32G4 Flash操作接口
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include "common.h"
#include <stdint.h>
#include <stdbool.h>

/* Flash配置参数(STM32G4) */
#define BSP_FLASH_BASE          0x08000000
#define BSP_FLASH_PAGE_SIZE     2048        ///< 每页2KB

/* Flash页地址定义 */
#define ADDR_FLASH_PAGE_62      0x0801F000  ///< Page 62 (倒数第2页)
#define ADDR_FLASH_PAGE_63      0x0801F800  ///< Page 63 (倒数第1页)

/**
 * @brief Flash解锁
 */
void BSP_Flash_Unlock(void);

/**
 * @brief Flash上锁
 */
void BSP_Flash_Lock(void);

/**
 * @brief 擦除Flash页
 * @param page_addr 页起始地址
 * @return true=成功, false=失败
 */
bool BSP_Flash_ErasePage(uint32_t page_addr);

/**
 * @brief 写入Flash双字(64位)
 * @param address Flash地址
 * @param data 64位数据
 * @return true=成功, false=失败
 */
bool BSP_Flash_WriteDoubleWord(uint32_t address, uint64_t data);

/**
 * @brief 从Flash读取数据
 * @param address Flash地址
 * @param data 输出缓冲区
 * @param length 读取长度(字节)
 */
void BSP_Flash_Read(uint32_t address, uint8_t *data, uint32_t length);

/**
 * @brief 验证Flash数据
 * @param address Flash地址
 * @param data 预期数据
 * @param length 数据长度(字节)
 * @return true=匹配, false=不匹配
 */
bool BSP_Flash_Verify(uint32_t address, const uint8_t *data, uint32_t length);

/**
 * @brief 计算CRC32
 * @param data 数据指针
 * @param length 数据长度(字节)
 * @return CRC32值
 */
uint32_t BSP_Flash_CalculateCRC32(const uint8_t *data, uint32_t length);

#endif /* BSP_FLASH_H */
