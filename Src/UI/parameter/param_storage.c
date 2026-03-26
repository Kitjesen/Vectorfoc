/**
 * @file param_storage.c
 * @brief paramFlash
 */
#include "param_storage.h"
#include "bsp_flash.h"
#include <string.h>
static uint32_t s_write_count = 0;
static uint32_t s_last_crc = 0;
static uint32_t s_generation = 0;
static uint8_t  s_active_page = 0;
/**
 * @brief Flash (64)
 * @param address Flash
 * @param data
 * @param length ()
 * @return true=, false=
 */
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
/**
 * @brief initparam
 */
void ParamStorage_Init(void)
{
    s_write_count = 0;
    s_last_crc = 0;
    if (ParamStorage_HasValidData()) {
        FlashParamData temp;
        if (ParamStorage_Load(&temp) == FLASH_STORAGE_OK) {
            s_last_crc = temp.crc32;
        }
    }
    FlashParamData temp_v2;
    FlashStorageResult v2_res = ParamStorage_Load_v2(&temp_v2);
    if (v2_res != FLASH_STORAGE_OK) {
        s_generation = 0;
        s_active_page = 0;
    }
}
/**
 * @brief paramFlash ()
 */
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
    s_last_crc = flash_data.crc32;
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
    s_write_count++;
    return FLASH_STORAGE_OK;
}
/**
 * @brief Flashparam (Page1，Page2)
 */
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
/**
 * @brief Flashparam
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
 * @brief checkparam
 */
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
/**
 * @brief get
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
FlashPageIndex ParamStorage_GetActivePage(void)
{
    return s_active_page;
}
static bool page_is_valid_v2(uint32_t addr, FlashParamData *out)
{
    BSP_Flash_Read(addr, (uint8_t*)out, sizeof(FlashParamData));
    if (out->magic == FLASH_MAGIC_WORD_V2) {
        if (out->committed != 1) return false;
        uint32_t stored = out->crc32;
        out->crc32 = 0;
        uint8_t *crc_start = ((uint8_t*)out) + 24;
        uint32_t crc_len   = sizeof(FlashParamData) - 24;
        bool ok = (BSP_Flash_CalculateCRC32(crc_start, crc_len) == stored);
        out->crc32 = stored;
        return ok;
    }
    if (out->magic == FLASH_MAGIC_WORD) {
        uint32_t stored = out->crc32;
        out->crc32 = 0;
        uint8_t *crc_start_v1 = ((uint8_t*)out) + 16;
        uint32_t crc_len_v1   = sizeof(FlashParamData) - 16;
        bool ok = (BSP_Flash_CalculateCRC32(crc_start_v1, crc_len_v1) == stored);
        out->crc32 = stored;
        if (ok) {
            /* V1 header is 16 bytes; V2 header is 24 bytes (generation+committed added).
             * Raw Flash bytes [16..size-1] are V1 motor params mapped onto V2
             * generation/committed/motor_rs... — shift them 8 bytes forward so
             * motor_rs lands at offset 24 where V2 expects it. The last 8 bytes of
             * V1 reserved_data are lost (they are always 0xFF padding). */
            uint8_t *raw = (uint8_t*)out;
            memmove(raw + 24, raw + 16, sizeof(FlashParamData) - 24);
            out->magic      = FLASH_MAGIC_WORD_V2;
            out->generation = 0;
            out->committed  = 1;
        }
        return ok;
    }
    return false;
}
FlashStorageResult ParamStorage_Save_v2(const FlashParamData *data)
{
    if (data == NULL) {
        return FLASH_STORAGE_ERR_LOCKED;
    }

    uint8_t standby    = 1 - s_active_page;
    uint32_t write_addr = (standby == 0) ? FLASH_PARAM_PAGE1_ADDR : FLASH_PARAM_PAGE2_ADDR;

    FlashParamData wr = *data;
    wr.magic      = FLASH_MAGIC_WORD_V2;
    wr.version    = FLASH_PARAM_VERSION;
    wr.generation = s_generation + 1;
    wr.committed  = 0;
    wr.crc32      = 0;

    uint8_t *crc_start = ((uint8_t*)&wr) + 24;
    uint32_t crc_len   = sizeof(FlashParamData) - 24;
    wr.crc32 = BSP_Flash_CalculateCRC32(crc_start, crc_len);

    BSP_Flash_Unlock();

    if (!BSP_Flash_ErasePage(write_addr)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_ERASE;
    }

    uint32_t dw_count = (sizeof(FlashParamData) + 7) / 8;
    for (uint32_t i = 0; i < dw_count; i++) {
        uint64_t val = 0xFFFFFFFFFFFFFFFFULL;
        uint32_t chunk_offset = i * 8;
        if (chunk_offset < sizeof(FlashParamData)) {
            uint32_t remaining = sizeof(FlashParamData) - chunk_offset;
            uint32_t copy_len = (remaining < 8u) ? remaining : 8u;
            memcpy(&val, ((const uint8_t*)&wr) + chunk_offset, copy_len);
        }
        if (!BSP_Flash_WriteDoubleWord(write_addr + i * 8, val)) {
            BSP_Flash_Lock();
            return FLASH_STORAGE_ERR_WRITE;
        }
    }

    if (!BSP_Flash_Verify(write_addr, (const uint8_t*)&wr, sizeof(FlashParamData))) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_VERIFY;
    }

    uint32_t committed_offset = (uint32_t)((uint8_t*)&wr.committed - (uint8_t*)&wr);
    uint32_t dw_offset = (committed_offset / 8) * 8;
    uint32_t dw_addr   = write_addr + dw_offset;

    uint64_t commit_dw;
    BSP_Flash_Read(dw_addr, (uint8_t*)&commit_dw, 8);
    uint32_t field_byte_in_dw = committed_offset - dw_offset;
    uint8_t *dw_bytes = (uint8_t*)&commit_dw;
    dw_bytes[field_byte_in_dw + 0] = 1;
    dw_bytes[field_byte_in_dw + 1] = 0;
    dw_bytes[field_byte_in_dw + 2] = 0;
    dw_bytes[field_byte_in_dw + 3] = 0;

    if (!BSP_Flash_WriteDoubleWord(dw_addr, commit_dw)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_WRITE;
    }

    BSP_Flash_Lock();
    s_generation++;
    s_active_page = standby;
    s_write_count++;
    return FLASH_STORAGE_OK;
}
FlashStorageResult ParamStorage_Load_v2(FlashParamData *data)
{
    if (data == NULL) {
        return FLASH_STORAGE_ERR_LOCKED;
    }

    FlashParamData p1, p2;
    bool p1_valid = page_is_valid_v2(FLASH_PARAM_PAGE1_ADDR, &p1);
    bool p2_valid = page_is_valid_v2(FLASH_PARAM_PAGE2_ADDR, &p2);

    if (!p1_valid && !p2_valid) {
        return FLASH_STORAGE_ERR_CORRUPT;
    }

    FlashParamData *best;
    if      (p1_valid && !p2_valid) { best = &p1; s_active_page = 0; }
    else if (!p1_valid && p2_valid) { best = &p2; s_active_page = 1; }
    else {
        if (p1.generation >= p2.generation) { best = &p1; s_active_page = 0; }
        else                                { best = &p2; s_active_page = 1; }
    }

    s_generation = best->generation;
    *data = *best;
    return FLASH_STORAGE_OK;
}
