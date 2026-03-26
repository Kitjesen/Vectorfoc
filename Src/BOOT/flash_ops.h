/**
 * @file flash_ops.h
 * @brief Bootloader Flash 操作接口
 */
#ifndef FLASH_OPS_H
#define FLASH_OPS_H

#include "boot_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 解锁 Flash
 */
void Flash_Unlock(void);

/**
 * @brief 锁定 Flash
 */
void Flash_Lock(void);

/**
 * @brief 擦除 App 区域
 * @return BOOT_OK 成功, 其他失败
 */
BootStatus_t Flash_EraseAppArea(void);

/**
 * @brief 擦除指定页
 * @param page_num 页号
 * @return BOOT_OK 成功
 */
BootStatus_t Flash_ErasePage(uint32_t page_num);

/**
 * @brief 写入数据到 Flash
 * @param addr 目标地址 (必须 8 字节对齐)
 * @param data 数据指针
 * @param len 数据长度 (必须 8 的倍数)
 * @return BOOT_OK 成功
 */
BootStatus_t Flash_WriteData(uint32_t addr, const uint8_t *data, uint32_t len);

/**
 * @brief 读取 Flash 数据
 * @param addr 源地址
 * @param data 目标缓冲区
 * @param len 长度
 */
void Flash_ReadData(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief 计算 CRC32
 * @param data 数据指针
 * @param len 长度
 * @return CRC32 值
 */
uint32_t Flash_CalcCRC32(const uint8_t *data, uint32_t len);

/**
 * @brief 计算 Flash 区域的 CRC32
 * @param addr 起始地址
 * @param len 长度
 * @return CRC32 值
 */
uint32_t Flash_CalcFlashCRC32(uint32_t addr, uint32_t len);

/**
 * @brief 检查地址是否在 App 区域内
 * @param addr 地址
 * @param len 长度
 * @return true 有效
 */
bool Flash_IsAddrInAppArea(uint32_t addr, uint32_t len);

#endif /* FLASH_OPS_H */
