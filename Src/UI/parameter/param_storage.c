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
 * @file param_storage.c
 * @brief Transactional dual-page parameter storage.
 */
#include "config.h"
#include "param_storage.h"
#include "bsp_flash.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PARAM_HEADER_CRC_OFFSET       16u
#define PARAM_COMMIT_DW_OFFSET        16u
#define PARAM_COMMITTED_VALUE         0x00000000u
#define PARAM_PENDING_VALUE           0xFFFFFFFFu
#define PARAM_LEGACY_VERSION          0x00010001u
#define FLASH_DOUBLEWORD_SIZE         8u
#define FLASH_READ_CHUNK_SIZE         32u
#define FLASH_HEADER_SIZE             24u

typedef char ParamCommitDoublewordMustBeOffset16[
    (offsetof(FlashParamData, generation) == PARAM_COMMIT_DW_OFFSET &&
     offsetof(FlashParamData, committed) == PARAM_COMMIT_DW_OFFSET + 4u) ? 1 : -1];

typedef char ParamCompactImageMustFitFlashPage[(sizeof(FlashParamData) < PARAM_FLASH_IMAGE_SIZE) ? 1 : -1];
typedef char ParamLogicalImageMustMatchPage[(PARAM_FLASH_IMAGE_SIZE == FLASH_PARAM_PAGE_SIZE) ? 1 : -1];
typedef char ParamCompactImageMustCoverCommit[(sizeof(FlashParamData) >= FLASH_HEADER_SIZE) ? 1 : -1];

typedef struct {
    uint32_t magic;
    uint32_t param_version;
    uint32_t crc32;
    uint32_t reserved;
    uint32_t generation;
    uint32_t committed;
} PageHeader;

typedef struct {
    uint32_t address;
    uint32_t generation;
    uint32_t crc32;
    bool valid;
} PageInfo;

static uint32_t s_write_count = 0;
static uint32_t s_last_crc = 0;
static uint32_t s_generation = 0;
static uint8_t  s_active_page = 0;

static FlashStorageResult ParamStorage_Save_v2(FlashParamData *data);
static FlashStorageResult ParamStorage_Load_v2(FlashParamData *data);

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; bit++) {
            crc = (crc & 1u) ? ((crc >> 1u) ^ 0xEDB88320u) : (crc >> 1u);
        }
    }
    return crc;
}

static uint32_t crc32_finish(uint32_t crc)
{
    return ~crc;
}

static uint32_t crc32_ieee_start(void)
{
    return 0xFFFFFFFFu;
}

static uint32_t crc32_update_zeros(uint32_t crc, uint32_t length)
{
    static const uint8_t zeros[FLASH_READ_CHUNK_SIZE] = {0};
    while (length > 0u) {
        uint32_t chunk = (length > FLASH_READ_CHUNK_SIZE) ? FLASH_READ_CHUNK_SIZE : length;
        crc = crc32_update(crc, zeros, chunk);
        length -= chunk;
    }
    return crc;
}

static uint32_t param_crc32(const FlashParamData *data)
{
    uint32_t crc = crc32_ieee_start();
    const uint8_t *bytes = (const uint8_t*)data;

    crc = crc32_update(crc,
                       bytes + PARAM_HEADER_CRC_OFFSET,
                       (uint32_t)sizeof(FlashParamData) - PARAM_HEADER_CRC_OFFSET);
    crc = crc32_update_zeros(crc, PARAM_FLASH_IMAGE_SIZE - (uint32_t)sizeof(FlashParamData));
    return crc32_finish(crc);
}

static uint32_t page_addr_from_index(uint8_t page)
{
    return (page == 0u) ? FLASH_PARAM_PAGE1_ADDR : FLASH_PARAM_PAGE2_ADDR;
}

static bool generation_is_newer(uint32_t candidate, uint32_t incumbent)
{
    return (candidate != incumbent) && ((uint32_t)(candidate - incumbent) < 0x80000000u);
}

static void read_header(uint32_t addr, PageHeader *header)
{
    BSP_Flash_Read(addr, (uint8_t*)header, (uint32_t)sizeof(*header));
}

static uint32_t flash_crc32(uint32_t addr)
{
    uint8_t chunk[FLASH_READ_CHUNK_SIZE];
    uint32_t crc = crc32_ieee_start();
    uint32_t offset = PARAM_HEADER_CRC_OFFSET;

    while (offset < PARAM_FLASH_IMAGE_SIZE) {
        uint32_t remaining = PARAM_FLASH_IMAGE_SIZE - offset;
        uint32_t len = (remaining > FLASH_READ_CHUNK_SIZE) ? FLASH_READ_CHUNK_SIZE : remaining;
        BSP_Flash_Read(addr + offset, chunk, len);
        crc = crc32_update(crc, chunk, len);
        offset += len;
    }

    return crc32_finish(crc);
}

static bool flash_tail_is_zero(uint32_t addr)
{
    uint8_t chunk[FLASH_READ_CHUNK_SIZE];
    uint32_t offset = (uint32_t)sizeof(FlashParamData);

    while (offset < PARAM_FLASH_IMAGE_SIZE) {
        uint32_t remaining = PARAM_FLASH_IMAGE_SIZE - offset;
        uint32_t len = (remaining > FLASH_READ_CHUNK_SIZE) ? FLASH_READ_CHUNK_SIZE : remaining;
        BSP_Flash_Read(addr + offset, chunk, len);
        for (uint32_t i = 0; i < len; i++) {
            if (chunk[i] != 0u) {
                return false;
            }
        }
        offset += len;
    }

    return true;
}

static uint64_t expected_doubleword_from_image(const FlashParamData *image, uint32_t offset)
{
    uint8_t bytes[FLASH_DOUBLEWORD_SIZE] = {0};
    const uint8_t *src = (const uint8_t*)image;

    if (offset < (uint32_t)sizeof(FlashParamData)) {
        uint32_t remaining = (uint32_t)sizeof(FlashParamData) - offset;
        uint32_t copy_len = (remaining < FLASH_DOUBLEWORD_SIZE) ? remaining : FLASH_DOUBLEWORD_SIZE;
        memcpy(bytes, src + offset, copy_len);
    }

    uint64_t value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

static bool flash_write_image_except_commit_dw(uint32_t address, const FlashParamData *image)
{
    for (uint32_t offset = 0u; offset < PARAM_FLASH_IMAGE_SIZE; offset += FLASH_DOUBLEWORD_SIZE) {
        if (offset == PARAM_COMMIT_DW_OFFSET) {
            continue;
        }

        uint64_t value = expected_doubleword_from_image(image, offset);
        if (value == 0xFFFFFFFFFFFFFFFFULL) {
            continue;
        }
        if (!BSP_Flash_WriteDoubleWord(address + offset, value)) {
            return false;
        }
    }

    return true;
}

static bool verify_pending_image(uint32_t address, const FlashParamData *image)
{
    uint8_t expected[FLASH_DOUBLEWORD_SIZE];

    for (uint32_t offset = 0u; offset < PARAM_FLASH_IMAGE_SIZE; offset += FLASH_DOUBLEWORD_SIZE) {
        uint64_t value = (offset == PARAM_COMMIT_DW_OFFSET)
            ? 0xFFFFFFFFFFFFFFFFULL
            : expected_doubleword_from_image(image, offset);
        memcpy(expected, &value, sizeof(expected));
        if (!BSP_Flash_Verify(address + offset, expected, (uint32_t)sizeof(expected))) {
            return false;
        }
    }

    return true;
}

static void normalize_migrated_page(FlashParamData *out)
{
    out->can_baudrate = DEFAULT_CAN_BAUDRATE;
    out->ladrc_enable = (float)DEFAULT_LADRC_ENABLE;
    out->ladrc_omega_o = DEFAULT_LADRC_OMEGA_O;
    out->ladrc_omega_c = DEFAULT_LADRC_OMEGA_C;
    out->ladrc_b0 = DEFAULT_LADRC_B0;
    out->ladrc_max_output = DEFAULT_LADRC_MAX_OUT;
    out->encoder_calib_valid = 0u;
    memset(out->encoder_calib_reserved, 0, sizeof(out->encoder_calib_reserved));
    memset(out->encoder_offset_lut, 0, sizeof(out->encoder_offset_lut));
    out->magic = FLASH_MAGIC_WORD_V2;
    out->param_version = FLASH_PARAM_VERSION;
    out->generation = 0u;
    out->committed = PARAM_COMMITTED_VALUE;
    out->crc32 = 0u;
    out->crc32 = param_crc32(out);
}

static bool inspect_page(uint32_t addr, PageInfo *info)
{
    PageHeader header;
    read_header(addr, &header);

    info->address = addr;
    info->generation = 0u;
    info->crc32 = 0u;
    info->valid = false;

    if (header.magic == FLASH_MAGIC_WORD_V2) {
        if (header.param_version != FLASH_PARAM_VERSION ||
            header.committed != PARAM_COMMITTED_VALUE) {
            return false;
        }
        if (flash_crc32(addr) != header.crc32 || !flash_tail_is_zero(addr)) {
            return false;
        }
        info->generation = header.generation;
        info->crc32 = header.crc32;
        info->valid = true;
        return true;
    }

    if (header.magic == FLASH_MAGIC_WORD) {
        bool legacy_16byte_header = header.param_version == PARAM_LEGACY_VERSION;
        bool transitional_24byte_header =
            header.param_version == FLASH_PARAM_VERSION &&
            header.generation == 0u &&
            header.committed == PARAM_COMMITTED_VALUE;
        if (!legacy_16byte_header && !transitional_24byte_header) {
            return false;
        }
        if (flash_crc32(addr) != header.crc32) {
            return false;
        }
        info->generation = 0u;
        info->crc32 = header.crc32;
        info->valid = true;
        return true;
    }

    return false;
}

static bool load_page(uint32_t addr, FlashParamData *out)
{
    PageHeader header;
    read_header(addr, &header);

    if (header.magic == FLASH_MAGIC_WORD_V2) {
        if (!inspect_page(addr, &(PageInfo){0})) {
            return false;
        }
        BSP_Flash_Read(addr, (uint8_t*)out, (uint32_t)sizeof(*out));
        return true;
    }

    if (header.magic == FLASH_MAGIC_WORD) {
        bool legacy_16byte_header = header.param_version == PARAM_LEGACY_VERSION;
        bool transitional_24byte_header =
            header.param_version == FLASH_PARAM_VERSION &&
            header.generation == 0u &&
            header.committed == PARAM_COMMITTED_VALUE;
        if (!legacy_16byte_header && !transitional_24byte_header) {
            return false;
        }
        if (flash_crc32(addr) != header.crc32) {
            return false;
        }

        memset(out, 0, sizeof(*out));
        if (legacy_16byte_header) {
            uint32_t payload_len = (uint32_t)sizeof(*out) - FLASH_HEADER_SIZE;
            BSP_Flash_Read(addr + 16u, ((uint8_t*)out) + FLASH_HEADER_SIZE, payload_len);
        } else {
            BSP_Flash_Read(addr, (uint8_t*)out, (uint32_t)sizeof(*out));
        }
        normalize_migrated_page(out);
        return true;
    }

    return false;
}

static bool select_best_page(PageInfo *best, uint8_t *active_page)
{
    PageInfo p1;
    PageInfo p2;
    bool p1_valid = inspect_page(FLASH_PARAM_PAGE1_ADDR, &p1);
    bool p2_valid = inspect_page(FLASH_PARAM_PAGE2_ADDR, &p2);

    if (!p1_valid && !p2_valid) {
        return false;
    }

    if (p1_valid && !p2_valid) {
        *best = p1;
        *active_page = 0u;
    } else if (!p1_valid && p2_valid) {
        *best = p2;
        *active_page = 1u;
    } else if (generation_is_newer(p2.generation, p1.generation)) {
        *best = p2;
        *active_page = 1u;
    } else {
        *best = p1;
        *active_page = 0u;
    }

    return true;
}

void ParamStorage_Init(void)
{
    PageInfo best;
    uint8_t active_page;

    s_write_count = 0;
    s_last_crc = 0;

    if (select_best_page(&best, &active_page)) {
        s_generation = best.generation;
        s_active_page = active_page;
        s_last_crc = best.crc32;
    } else {
        s_generation = 0;
        s_active_page = 0;
    }
}

FlashStorageResult ParamStorage_Save(FlashParamData *data)
{
    return ParamStorage_Save_v2(data);
}

FlashStorageResult ParamStorage_Load(FlashParamData *data)
{
    return ParamStorage_Load_v2(data);
}

FlashStorageResult ParamStorage_Erase(void)
{
    BSP_Flash_Unlock();
    bool success1 = BSP_Flash_ErasePage(FLASH_PARAM_PAGE1_ADDR);
    bool success2 = BSP_Flash_ErasePage(FLASH_PARAM_PAGE2_ADDR);
    BSP_Flash_Lock();

    if (!success1 || !success2) {
        return FLASH_STORAGE_ERR_ERASE;
    }

    s_generation = 0;
    s_active_page = 0;
    s_last_crc = 0;
    return FLASH_STORAGE_OK;
}

bool ParamStorage_HasValidData(void)
{
    PageInfo info;
    return inspect_page(FLASH_PARAM_PAGE1_ADDR, &info) ||
           inspect_page(FLASH_PARAM_PAGE2_ADDR, &info);
}

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

static FlashStorageResult ParamStorage_Save_v2(FlashParamData *data)
{
    PageInfo current;
    uint8_t active_page;

    if (data == NULL) {
        return FLASH_STORAGE_ERR_LOCKED;
    }

    if (select_best_page(&current, &active_page)) {
        s_generation = current.generation;
        s_active_page = active_page;
    } else {
        s_generation = 0;
        s_active_page = 0;
    }

    uint8_t standby = (uint8_t)(1u - s_active_page);
    uint32_t write_addr = page_addr_from_index(standby);
    uint32_t next_generation = s_generation + 1u;

    data->magic = FLASH_MAGIC_WORD_V2;
    data->param_version = FLASH_PARAM_VERSION;
    data->generation = next_generation;
    data->committed = PARAM_COMMITTED_VALUE;
    data->crc32 = 0u;
    data->crc32 = param_crc32(data);

    BSP_Flash_Unlock();

    if (!BSP_Flash_ErasePage(write_addr)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_ERASE;
    }

    if (!flash_write_image_except_commit_dw(write_addr, data)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_WRITE;
    }

    if (!verify_pending_image(write_addr, data)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_VERIFY;
    }

    uint64_t commit_dw = ((uint64_t)PARAM_COMMITTED_VALUE << 32) | (uint64_t)next_generation;
    if (!BSP_Flash_WriteDoubleWord(write_addr + PARAM_COMMIT_DW_OFFSET, commit_dw)) {
        BSP_Flash_Lock();
        return FLASH_STORAGE_ERR_WRITE;
    }

    BSP_Flash_Lock();

    PageInfo verify;
    if (!inspect_page(write_addr, &verify) || verify.generation != next_generation) {
        return FLASH_STORAGE_ERR_VERIFY;
    }

    s_generation = next_generation;
    s_active_page = standby;
    s_last_crc = verify.crc32;
    s_write_count++;
    return FLASH_STORAGE_OK;
}

static FlashStorageResult ParamStorage_Load_v2(FlashParamData *data)
{
    PageInfo best;
    uint8_t active_page;

    if (data == NULL) {
        return FLASH_STORAGE_ERR_LOCKED;
    }

    if (!select_best_page(&best, &active_page)) {
        return FLASH_STORAGE_ERR_CORRUPT;
    }

    if (!load_page(best.address, data)) {
        return FLASH_STORAGE_ERR_CORRUPT;
    }

    s_generation = data->generation;
    s_active_page = active_page;
    s_last_crc = data->crc32;
    return FLASH_STORAGE_OK;
}