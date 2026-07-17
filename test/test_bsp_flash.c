// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "main.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

#define FLASH_TYPEERASE_PAGES 0x02u
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0x03u
#define FLASH_BANK_1 0x01u

typedef struct {
  uint32_t TypeErase;
  uint32_t Banks;
  uint32_t Page;
  uint32_t NbPages;
} FLASH_EraseInitTypeDef;

static unsigned erase_call_count;
static unsigned program_call_count;
static FLASH_EraseInitTypeDef last_erase;
static uint32_t last_program_type;
static uint32_t last_program_address;
static uint64_t last_program_data;

void HAL_FLASH_Unlock(void) {}
void HAL_FLASH_Lock(void) {}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase_init,
                                    uint32_t *page_error) {
  erase_call_count++;
  last_erase = *erase_init;
  *page_error = 0u;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address,
                                    uint64_t data) {
  program_call_count++;
  last_program_type = type;
  last_program_address = address;
  last_program_data = data;
  return HAL_OK;
}

#include "../Src/HAL/bsp/bsp_flash.c"

static void reset_hal_spy(void) {
  erase_call_count = 0u;
  program_call_count = 0u;
  memset(&last_erase, 0, sizeof(last_erase));
  last_program_type = 0u;
  last_program_address = 0u;
  last_program_data = 0u;
}

static void test_erase_requires_valid_page_boundary(void) {
  reset_hal_spy();
  assert(BSP_Flash_ErasePage(BSP_FLASH_BASE));
  assert(erase_call_count == 1u);
  assert(last_erase.TypeErase == FLASH_TYPEERASE_PAGES);
  assert(last_erase.Banks == FLASH_BANK_1);
  assert(last_erase.Page == 0u);
  assert(last_erase.NbPages == 1u);

  reset_hal_spy();
  assert(BSP_Flash_ErasePage(BSP_FLASH_END - BSP_FLASH_PAGE_SIZE));
  assert(erase_call_count == 1u);
  assert(last_erase.Page == BSP_FLASH_PAGE_COUNT - 1u);

  reset_hal_spy();
  assert(!BSP_Flash_ErasePage(BSP_FLASH_BASE + 1u));
  assert(!BSP_Flash_ErasePage(BSP_FLASH_BASE - BSP_FLASH_PAGE_SIZE));
  assert(!BSP_Flash_ErasePage(BSP_FLASH_END));
  assert(erase_call_count == 0u);
}

static void test_doubleword_write_requires_valid_aligned_range(void) {
  reset_hal_spy();
  assert(BSP_Flash_WriteDoubleWord(BSP_FLASH_END - BSP_FLASH_DOUBLEWORD_SIZE,
                                   0x0123456789ABCDEFULL));
  assert(program_call_count == 1u);
  assert(last_program_type == FLASH_TYPEPROGRAM_DOUBLEWORD);
  assert(last_program_address == BSP_FLASH_END - BSP_FLASH_DOUBLEWORD_SIZE);
  assert(last_program_data == 0x0123456789ABCDEFULL);

  reset_hal_spy();
  assert(!BSP_Flash_WriteDoubleWord(BSP_FLASH_BASE + 4u, 0u));
  assert(!BSP_Flash_WriteDoubleWord(BSP_FLASH_BASE - BSP_FLASH_DOUBLEWORD_SIZE,
                                    0u));
  assert(!BSP_Flash_WriteDoubleWord(BSP_FLASH_END - 4u, 0u));
  assert(!BSP_Flash_WriteDoubleWord(BSP_FLASH_END, 0u));
  assert(program_call_count == 0u);
}

static void test_verify_rejects_invalid_ranges_before_memory_access(void) {
  const uint8_t data[8] = {0};

  assert(!BSP_Flash_Verify(BSP_FLASH_END, data, sizeof(data)));
  assert(!BSP_Flash_Verify(BSP_FLASH_END - 4u, data, sizeof(data)));
  assert(!BSP_Flash_Verify(BSP_FLASH_BASE, NULL, sizeof(data)));
  assert(!BSP_Flash_Verify(BSP_FLASH_BASE, data, 0u));
}

static void test_crc32_uses_ieee_semantics_for_all_bytes(void) {
  static const uint8_t standard_vector[] = "123456789";
  static const uint8_t five_bytes[] = {1u, 2u, 3u, 4u, 5u};

  assert(BSP_Flash_CalculateCRC32(standard_vector, 9u) == 0xCBF43926u);
  assert(BSP_Flash_CalculateCRC32(five_bytes, sizeof(five_bytes)) ==
         0x470B99F4u);
  assert(BSP_Flash_CalculateCRC32(NULL, 9u) == 0u);
  assert(BSP_Flash_CalculateCRC32(standard_vector, 0u) == 0u);
}

int main(void) {
  test_erase_requires_valid_page_boundary();
  test_doubleword_write_requires_valid_aligned_range();
  test_verify_rejects_invalid_ranges_before_memory_access();
  test_crc32_uses_ieee_semantics_for_all_bytes();
  return 0;
}