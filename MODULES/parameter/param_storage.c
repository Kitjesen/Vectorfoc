/**
 * @file param_storage.c
 * @brief 参数Flash存储实现
 */

#include "param_storage.h"
#include "bsp_flash.h"
#include <string.h>

/* CRC32查找表 (STM32硬件CRC使用) */
static uint32_t s_write_count = 0;
static uint32_t s_last_crc = 0;

/**
 * @brief 写入Flash (64位对齐)
 * @param address Flash地址
 * @param data 数据指针
 * @param length 数据长度(字节)
 * @return true=成功, false=失败
 */
static bool Flash_Write(uint32_t address, const uint8_t *data, uint32_t length)
{
    // STM32G4使用双字(64位)编程
    uint64_t *src = (uint64_t*)data;
    uint32_t doubleword_count = (length + 7) / 8;  // 向上取整
    
    for (uint32_t i = 0; i < doubleword_count; i++) {
        uint64_t value = (i * 8 < length) ? src[i] : 0xFFFFFFFFFFFFFFFF;
        
        if (!BSP_Flash_WriteDoubleWord(address + i * 8, value)) {
            return false;
        }
    }
    
    return true;
}

/**
 * @brief 初始化参数存储模块
 */
void ParamStorage_Init(void)
{
    s_write_count = 0;
    s_last_crc = 0;
    
    // 检查是否有有效数据
    if (ParamStorage_HasValidData()) {
        FlashParamData temp;
        if (ParamStorage_Load(&temp) == FLASH_STORAGE_OK) {
            s_last_crc = temp.crc32;
        }
    }
}

/**
 * @brief 保存参数到Flash (使用双页备份机制)
 */
FlashStorageResult ParamStorage_Save(const FlashParamData *data)
{
    if (data == NULL) {
        return FLASH_STORAGE_ERR_LOCKED;
    }
    
    // 准备数据
    FlashParamData flash_data = *data;
    flash_data.magic = FLASH_MAGIC_WORD;
    flash_data.version = FLASH_PARAM_VERSION;
    flash_data.crc32 = 0;  // CRC计算时要清零
    
    // 计算CRC (跳过magic, version, crc32字段)
    uint8_t *crc_start = ((uint8_t*)&flash_data) + 16;  // 跳过前4个uint32
    uint32_t crc_length = sizeof(FlashParamData) - 16;
    flash_data.crc32 = BSP_Flash_CalculateCRC32(crc_start, crc_length);
    
    s_last_crc = flash_data.crc32;
    
    BSP_Flash_Unlock();
    
    // 擦除并写入Page1
    if (!BSP_Flash_ErasePage(FLASH_PARAM_PAGE1_ADDR)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_ERASE;
    }
    
    if (!Flash_Write(FLASH_PARAM_PAGE1_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData))) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_WRITE;
    }
    
    // 验证写入
    if (!BSP_Flash_Verify(FLASH_PARAM_PAGE1_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData))) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_VERIFY;
    }
    
    // 写入Page2备份
    if (!BSP_Flash_ErasePage(FLASH_PARAM_PAGE2_ADDR)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_ERASE;
    }
    
    if (!Flash_Write(FLASH_PARAM_PAGE2_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData))) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_WRITE;
    }
    
    BSP_Flash_Lock();
    
    s_write_count++;
    
    return FLASH_STORAGE_OK;
}

/**
 * @brief 从Flash加载参数 (优先Page1，失败则用Page2备份)
 */
FlashStorageResult ParamStorage_Load(FlashParamData *data)
{
    if (data == NULL) {
        return FLASH_STORAGE_ERR_LOCKED;
    }
    
    FlashParamData flash_data;
    
    // 尝试从Page1加载
    BSP_Flash_Read(FLASH_PARAM_PAGE1_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData));
    
    // 检查魔术字
    if (flash_data.magic != FLASH_MAGIC_WORD) {
        // Page1无效，尝试Page2
        BSP_Flash_Read(FLASH_PARAM_PAGE2_ADDR, (uint8_t*)&flash_data, sizeof(FlashParamData));
        
        if (flash_data.magic != FLASH_MAGIC_WORD) {
            return FLASH_STORAGE_ERR_MAGIC;
        }
    }
    
    // 检查版本号 (可选)
    if (flash_data.version != FLASH_PARAM_VERSION) {
        // 版本不匹配，可以在这里做数据迁移
        // return FLASH_STORAGE_ERR_VERSION;
    }
    
    // 验证CRC
    uint32_t stored_crc = flash_data.crc32;
    flash_data.crc32 = 0;
    
    uint8_t *crc_start = ((uint8_t*)&flash_data) + 16;
    uint32_t crc_length = sizeof(FlashParamData) - 16;
    uint32_t calculated_crc = BSP_Flash_CalculateCRC32(crc_start, crc_length);
    
    if (stored_crc != calculated_crc) {
        return FLASH_STORAGE_ERR_CRC;
    }
    
    // 恢复CRC值
    flash_data.crc32 = stored_crc;
    
    // 复制数据
    *data = flash_data;
    
    return FLASH_STORAGE_OK;
}

/**
 * @brief 擦除Flash参数
 */
FlashStorageResult ParamStorage_Erase(void)
{
    BSP_Flash_Unlock();
    
    bool success1 = BSP_Flash_ErasePage(FLASH_PARAM_PAGE1_ADDR);
    bool success2 = BSP_Flash_ErasePage(FLASH_PARAM_PAGE2_ADDR);
    
    BSP_Flash_Lock();
    
    if (!success1 || !success2) {
        return FLASH_STORAGE_ERR_ERASE;
    }
    
    return FLASH_STORAGE_OK;
}

/**
 * @brief 检查是否有有效参数
 */
bool ParamStorage_HasValidData(void)
{
    FlashParamData temp;
    BSP_Flash_Read(FLASH_PARAM_PAGE1_ADDR, (uint8_t*)&temp, sizeof(uint32_t));
    
    if (temp.magic == FLASH_MAGIC_WORD) {
        return true;
    }
    
    // 检查备份页
    BSP_Flash_Read(FLASH_PARAM_PAGE2_ADDR, (uint8_t*)&temp, sizeof(uint32_t));
    return (temp.magic == FLASH_MAGIC_WORD);
}

/**
 * @brief 获取存储统计信息
 */
void ParamStorage_GetStats(uint32_t *write_count, uint32_t *last_crc)
{
    if (write_count != NULL) {
        *write_count = s_write_count;
    }
    if (last_crc != NULL) {
        *last_crc = s_last_crc;
    }
}
