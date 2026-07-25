#include "bsp_flash.h"
#include "config.h"
#include "param_storage.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PAGE_COUNT 2u
#define DW_PER_PAGE (FLASH_PARAM_PAGE_SIZE / 8u)
#define COMMIT_OFFSET 16u
#define LEGACY_PARAM_VERSION 0x00010001u

static uint8_t flash_pages[PAGE_COUNT][FLASH_PARAM_PAGE_SIZE];
static uint8_t programmed[PAGE_COUNT][DW_PER_PAGE];
static uint32_t read_lengths[PARAM_FLASH_IMAGE_SIZE + 1u];
static int unlocked;
static int fail_erase_page;
static int fail_write_offset;
static int check_ladrc_defaults_restored(const FlashParamData *image);

static int fail_verify;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
      return 1;                                                                \
    }                                                                          \
  } while (0)

static int page_index(uint32_t address) {
  if (address >= FLASH_PARAM_PAGE1_ADDR &&
      address < FLASH_PARAM_PAGE1_ADDR + FLASH_PARAM_PAGE_SIZE) {
    return 0;
  }
  if (address >= FLASH_PARAM_PAGE2_ADDR &&
      address < FLASH_PARAM_PAGE2_ADDR + FLASH_PARAM_PAGE_SIZE) {
    return 1;
  }
  return -1;
}

static uint32_t page_offset(uint32_t address) {
  int page = page_index(address);
  uint32_t base = (page == 0) ? FLASH_PARAM_PAGE1_ADDR : FLASH_PARAM_PAGE2_ADDR;
  return address - base;
}

static void fake_flash_reset(void) {
  memset(flash_pages, 0xFF, sizeof(flash_pages));
  memset(programmed, 0, sizeof(programmed));
  memset(read_lengths, 0, sizeof(read_lengths));
  unlocked = 0;
  fail_erase_page = -1;
  fail_write_offset = -1;
  fail_verify = 0;
}

void BSP_Flash_Unlock(void) { unlocked = 1; }
void BSP_Flash_Lock(void) { unlocked = 0; }

bool BSP_Flash_ErasePage(uint32_t page_addr) {
  int page = page_index(page_addr);
  if (!unlocked || page < 0 || page_offset(page_addr) != 0u ||
      page == fail_erase_page) {
    return false;
  }
  memset(flash_pages[page], 0xFF, FLASH_PARAM_PAGE_SIZE);
  memset(programmed[page], 0, DW_PER_PAGE);
  return true;
}

bool BSP_Flash_WriteDoubleWord(uint32_t address, uint64_t data) {
  int page = page_index(address);
  uint32_t offset;
  uint64_t existing;

  if (!unlocked || page < 0) {
    return false;
  }
  offset = page_offset(address);
  if ((offset % 8u) != 0u || offset + 8u > FLASH_PARAM_PAGE_SIZE) {
    return false;
  }
  if (fail_write_offset >= 0 && offset == (uint32_t)fail_write_offset) {
    return false;
  }
  if (programmed[page][offset / 8u]) {
    return false;
  }
  memcpy(&existing, &flash_pages[page][offset], sizeof(existing));
  if (((~existing) & data) != 0u) {
    return false;
  }
  memcpy(&flash_pages[page][offset], &data, sizeof(data));
  programmed[page][offset / 8u] = 1u;
  return true;
}

void BSP_Flash_Read(uint32_t address, uint8_t *data, uint32_t length) {
  int page = page_index(address);
  uint32_t offset = page_offset(address);
  if (data == NULL || length == 0u || page < 0 ||
      offset + length > FLASH_PARAM_PAGE_SIZE) {
    return;
  }
  if (length <= PARAM_FLASH_IMAGE_SIZE) {
    read_lengths[length]++;
  }
  memcpy(data, &flash_pages[page][offset], length);
}

bool BSP_Flash_Verify(uint32_t address, const uint8_t *data, uint32_t length) {
  int page = page_index(address);
  uint32_t offset = page_offset(address);
  if (fail_verify) {
    return false;
  }
  if (data == NULL || length == 0u || page < 0 ||
      offset + length > FLASH_PARAM_PAGE_SIZE) {
    return false;
  }
  return memcmp(&flash_pages[page][offset], data, length) == 0;
}

uint32_t BSP_Flash_CalculateCRC32(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  if (data == NULL || length == 0u) {
    return 0;
  }
  for (uint32_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8u; j++) {
      crc = (crc & 1u) ? ((crc >> 1u) ^ 0xEDB88320u) : (crc >> 1u);
    }
  }
  return ~crc;
}

static FlashParamData make_data(float rs, uint8_t can_id) {
  FlashParamData data;
  memset(&data, 0, sizeof(data));
  data.motor_rs = rs;
  data.motor_ls = rs + 1.0f;
  data.motor_flux = rs + 2.0f;
  data.motor_pole_pairs = 7;
  data.can_id = can_id;
  data.cur_kp = 1.0f;
  data.cur_ki = 2.0f;
  data.limit_current = 15.0f;
  return data;
}

static void direct_write_image(uint32_t page_addr,
                               const FlashParamData *image) {
  int page = page_index(page_addr);
  memcpy(flash_pages[page], image, sizeof(*image));
  memset(flash_pages[page] + sizeof(*image), 0,
         PARAM_FLASH_IMAGE_SIZE - sizeof(*image));
  memset(programmed[page], 1, DW_PER_PAGE);
}

static uint32_t test_crc32_update(uint32_t crc, const uint8_t *data,
                                  uint32_t length) {
  for (uint32_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8u; j++) {
      crc = (crc & 1u) ? ((crc >> 1u) ^ 0xEDB88320u) : (crc >> 1u);
    }
  }
  return crc;
}

static uint32_t test_crc32_finish(uint32_t crc) { return ~crc; }

static uint32_t image_crc(FlashParamData *image) {
  static const uint8_t zeros[32] = {0};
  uint32_t crc = 0xFFFFFFFFu;
  uint32_t tail = PARAM_FLASH_IMAGE_SIZE - (uint32_t)sizeof(*image);
  image->crc32 = 0;
  crc = test_crc32_update(crc, ((const uint8_t *)image) + 16u,
                          (uint32_t)sizeof(*image) - 16u);
  while (tail > 0u) {
    uint32_t chunk = (tail > sizeof(zeros)) ? (uint32_t)sizeof(zeros) : tail;
    crc = test_crc32_update(crc, zeros, chunk);
    tail -= chunk;
  }
  return test_crc32_finish(crc);
}

static uint32_t page_crc(uint32_t page) {
  return BSP_Flash_CalculateCRC32(flash_pages[page] + 16u,
                                  PARAM_FLASH_IMAGE_SIZE - 16u);
}

static int check_tail_zero(uint32_t page) {
  for (uint32_t i = (uint32_t)sizeof(FlashParamData);
       i < PARAM_FLASH_IMAGE_SIZE; i++) {
    CHECK(flash_pages[page][i] == 0u);
  }
  return 0;
}

static void poison_encoder_calibration_fields(FlashParamData *image) {
  image->encoder_calib_valid = 1u;
  memset(image->encoder_calib_reserved, 0xA5,
         sizeof(image->encoder_calib_reserved));
  for (uint32_t i = 0; i < 128u; i++) {
    image->encoder_offset_lut[i] = (int16_t)(1000 + (int32_t)i);
  }
}

static int
check_encoder_calibration_fields_cleared(const FlashParamData *image) {
  CHECK(image->encoder_calib_valid == 0u);
  for (uint32_t i = 0; i < sizeof(image->encoder_calib_reserved); i++) {
    CHECK(image->encoder_calib_reserved[i] == 0u);
  }
  for (uint32_t i = 0; i < 128u; i++) {
    CHECK(image->encoder_offset_lut[i] == 0);
  }
  return 0;
}

static int test_encoder_calibration_fields_fit_and_round_trip(void) {
  FlashParamData in = make_data(10.0f, 12);
  FlashParamData out;

  CHECK(sizeof(FlashParamData) < PARAM_FLASH_IMAGE_SIZE);
  CHECK(PARAM_FLASH_IMAGE_SIZE == FLASH_PARAM_PAGE_SIZE);
  fake_flash_reset();
  ParamStorage_Init();

  in.add_offset = 1.25f;
  in.encoder_calib_valid = 1u;
  for (uint32_t i = 0; i < 128u; i++) {
    in.encoder_offset_lut[i] = (int16_t)((int32_t)i - 64);
  }

  CHECK(ParamStorage_Save(&in) == FLASH_STORAGE_OK);
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.add_offset == in.add_offset);
  CHECK(out.encoder_calib_valid == 1u);
  for (uint32_t i = 0; i < 128u; i++) {
    CHECK(out.encoder_offset_lut[i] == in.encoder_offset_lut[i]);
  }
  return 0;
}
static int test_empty_flash_has_no_data(void) {
  FlashParamData out;
  fake_flash_reset();
  ParamStorage_Init();
  CHECK(!ParamStorage_HasValidData());
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_ERR_CORRUPT);
  return 0;
}

static int test_save_load_commits_offset16_once(void) {
  FlashParamData in = make_data(1.25f, 11);
  FlashParamData out;
  uint64_t commit_dw;
  uint32_t writes;
  uint32_t crc;

  fake_flash_reset();
  ParamStorage_Init();
  CHECK(ParamStorage_Save(&in) == FLASH_STORAGE_OK);
  CHECK(ParamStorage_GetActivePage() == 1u);
  CHECK(programmed[1][COMMIT_OFFSET / 8u] == 1u);
  memcpy(&commit_dw, &flash_pages[1][COMMIT_OFFSET], sizeof(commit_dw));
  CHECK(((uint32_t)commit_dw) == 1u);
  CHECK(((uint32_t)(commit_dw >> 32)) == 0u);
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.magic == FLASH_MAGIC_WORD_V2);
  CHECK(out.committed == 0u);
  CHECK(out.generation == 1u);
  CHECK(out.motor_rs == in.motor_rs);
  ParamStorage_GetStats(&writes, &crc);
  CHECK(writes == 1u);
  CHECK(crc == out.crc32);
  return 0;
}

static int test_failed_final_commit_falls_back_to_previous_page(void) {
  FlashParamData first = make_data(2.0f, 3);
  FlashParamData second = make_data(9.0f, 4);
  FlashParamData out;

  fake_flash_reset();
  ParamStorage_Init();
  CHECK(ParamStorage_Save(&first) == FLASH_STORAGE_OK);
  fail_write_offset = (int)COMMIT_OFFSET;
  CHECK(ParamStorage_Save(&second) == FLASH_STORAGE_ERR_WRITE);
  fail_write_offset = -1;
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.motor_rs == first.motor_rs);
  CHECK(out.can_id == first.can_id);
  return 0;
}

static int test_crc_corruption_falls_back_to_older_valid_page(void) {
  FlashParamData first = make_data(3.0f, 5);
  FlashParamData second = make_data(4.0f, 6);
  FlashParamData out;
  int active_page;

  fake_flash_reset();
  ParamStorage_Init();
  CHECK(ParamStorage_Save(&first) == FLASH_STORAGE_OK);
  CHECK(ParamStorage_Save(&second) == FLASH_STORAGE_OK);
  active_page = (int)ParamStorage_GetActivePage();
  flash_pages[active_page][64] ^= 0x5Au;
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.motor_rs == first.motor_rs);
  CHECK(out.can_id == first.can_id);
  return 0;
}

static int test_erase_failure_preserves_previous_page(void) {
  FlashParamData first = make_data(3.5f, 21);
  FlashParamData second = make_data(8.5f, 22);
  FlashParamData out;

  fake_flash_reset();
  ParamStorage_Init();
  CHECK(ParamStorage_Save(&first) == FLASH_STORAGE_OK);
  fail_erase_page = 0;
  CHECK(ParamStorage_Save(&second) == FLASH_STORAGE_ERR_ERASE);
  fail_erase_page = -1;
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.motor_rs == first.motor_rs);
  CHECK(out.can_id == first.can_id);
  return 0;
}

static int test_mid_image_write_failure_preserves_previous_page(void) {
  FlashParamData first = make_data(3.75f, 23);
  FlashParamData second = make_data(8.75f, 24);
  FlashParamData out;

  fake_flash_reset();
  ParamStorage_Init();
  CHECK(ParamStorage_Save(&first) == FLASH_STORAGE_OK);
  fail_write_offset = 24;
  CHECK(ParamStorage_Save(&second) == FLASH_STORAGE_ERR_WRITE);
  fail_write_offset = -1;
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.motor_rs == first.motor_rs);
  CHECK(out.can_id == first.can_id);
  return 0;
}

static int test_pending_verify_failure_preserves_previous_page(void) {
  FlashParamData first = make_data(4.25f, 25);
  FlashParamData second = make_data(9.25f, 26);
  FlashParamData out;

  fake_flash_reset();
  ParamStorage_Init();
  CHECK(ParamStorage_Save(&first) == FLASH_STORAGE_OK);
  fail_verify = 1;
  CHECK(ParamStorage_Save(&second) == FLASH_STORAGE_ERR_VERIFY);
  fail_verify = 0;
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.motor_rs == first.motor_rs);
  CHECK(out.can_id == first.can_id);
  return 0;
}

static int test_equal_generation_tie_prefers_page1(void) {
  FlashParamData p1 = make_data(4.5f, 27);
  FlashParamData p2 = make_data(9.5f, 28);
  FlashParamData out;

  fake_flash_reset();
  p1.magic = FLASH_MAGIC_WORD_V2;
  p1.param_version = FLASH_PARAM_VERSION;
  p1.generation = 42u;
  p1.committed = 0u;
  p1.crc32 = image_crc(&p1);
  p2.magic = FLASH_MAGIC_WORD_V2;
  p2.param_version = FLASH_PARAM_VERSION;
  p2.generation = 42u;
  p2.committed = 0u;
  p2.crc32 = image_crc(&p2);
  direct_write_image(FLASH_PARAM_PAGE1_ADDR, &p1);
  direct_write_image(FLASH_PARAM_PAGE2_ADDR, &p2);

  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.motor_rs == p1.motor_rs);
  CHECK(ParamStorage_GetActivePage() == 0u);
  return 0;
}

static int test_generation_wrap_selects_zero_after_uint32_max(void) {
  FlashParamData old_max = make_data(5.0f, 7);
  FlashParamData wrapped = make_data(6.0f, 8);
  FlashParamData out;

  fake_flash_reset();
  old_max.magic = FLASH_MAGIC_WORD_V2;
  old_max.param_version = FLASH_PARAM_VERSION;
  old_max.generation = 0xFFFFFFFFu;
  old_max.committed = 0u;
  old_max.crc32 = image_crc(&old_max);
  wrapped.magic = FLASH_MAGIC_WORD_V2;
  wrapped.param_version = FLASH_PARAM_VERSION;
  wrapped.generation = 0u;
  wrapped.committed = 0u;
  wrapped.crc32 = image_crc(&wrapped);
  direct_write_image(FLASH_PARAM_PAGE1_ADDR, &old_max);
  direct_write_image(FLASH_PARAM_PAGE2_ADDR, &wrapped);

  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.generation == 0u);
  CHECK(out.motor_rs == wrapped.motor_rs);
  CHECK(ParamStorage_GetActivePage() == 1u);
  return 0;
}

static int test_foc1_transitional_24byte_layout_not_shifted(void) {
  FlashParamData legacy = make_data(7.0f, 9);
  FlashParamData out;

  fake_flash_reset();
  legacy.magic = FLASH_MAGIC_WORD;
  legacy.can_baudrate = 1u;
  legacy.param_version = FLASH_PARAM_VERSION;
  legacy.generation = 0u;
  legacy.committed = 0u;
  poison_encoder_calibration_fields(&legacy);
  legacy.crc32 = image_crc(&legacy);
  direct_write_image(FLASH_PARAM_PAGE1_ADDR, &legacy);

  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.magic == FLASH_MAGIC_WORD_V2);
  CHECK(out.motor_rs == 7.0f);
  CHECK(out.can_id == 9u);
  CHECK(out.can_baudrate == DEFAULT_CAN_BAUDRATE);
  CHECK(check_encoder_calibration_fields_cleared(&out) == 0);
  CHECK(check_ladrc_defaults_restored(&out) == 0);
  return 0;
}

static int test_foc1_legacy_16byte_layout_migrates_cautiously(void) {
  FlashParamData source = make_data(8.0f, 10);
  source.motor_rs = 0.0f;
  source.motor_ls = 0.0f;
  poison_encoder_calibration_fields(&source);
  FlashParamData stored;
  FlashParamData out;
  uint8_t *raw = (uint8_t *)&stored;

  fake_flash_reset();
  memset(&stored, 0xFF, sizeof(stored));
  stored.magic = FLASH_MAGIC_WORD;
  source.can_baudrate = 1u;
  stored.param_version = LEGACY_PARAM_VERSION;
  stored.reserved = 0u;
  memcpy(raw + 16u, ((uint8_t *)&source) + 24u, sizeof(FlashParamData) - 24u);
  stored.crc32 = image_crc(&stored);
  direct_write_image(FLASH_PARAM_PAGE2_ADDR, &stored);

  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_OK);
  CHECK(out.magic == FLASH_MAGIC_WORD_V2);
  CHECK(out.generation == 0u);
  CHECK(out.committed == 0u);
  CHECK(out.motor_rs == 0.0f);
  CHECK(out.motor_ls == 0.0f);
  CHECK(out.can_id == source.can_id);
  CHECK(out.can_baudrate == DEFAULT_CAN_BAUDRATE);
  CHECK(check_encoder_calibration_fields_cleared(&out) == 0);
  CHECK(check_ladrc_defaults_restored(&out) == 0);
  return 0;
}

static int test_wrong_schema_version_is_rejected(void) {
  FlashParamData image = make_data(10.0f, 12);
  FlashParamData out;

  fake_flash_reset();
  image.magic = FLASH_MAGIC_WORD_V2;
  image.param_version = 0xDEADBEEFu;
  image.generation = 1u;
  image.committed = 0u;
  image.crc32 = image_crc(&image);
  direct_write_image(FLASH_PARAM_PAGE1_ADDR, &image);

  CHECK(!ParamStorage_HasValidData());
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_ERR_CORRUPT);
  return 0;
}

static int test_ambiguous_foc1_header_is_rejected(void) {
  FlashParamData image = make_data(11.0f, 13);
  FlashParamData out;

  fake_flash_reset();
  image.magic = FLASH_MAGIC_WORD;
  image.param_version = FLASH_PARAM_VERSION;
  image.generation = 9u;
  image.committed = 0u;
  image.crc32 = image_crc(&image);
  direct_write_image(FLASH_PARAM_PAGE1_ADDR, &image);

  CHECK(!ParamStorage_HasValidData());
  CHECK(ParamStorage_Load(&out) == FLASH_STORAGE_ERR_CORRUPT);
  return 0;
}

int main(void) {
  int rc = 0;
  rc |= test_encoder_calibration_fields_fit_and_round_trip();
  rc |= test_empty_flash_has_no_data();
  rc |= test_save_load_commits_offset16_once();
  rc |= test_failed_final_commit_falls_back_to_previous_page();
  rc |= test_crc_corruption_falls_back_to_older_valid_page();
  rc |= test_erase_failure_preserves_previous_page();
  rc |= test_mid_image_write_failure_preserves_previous_page();
  rc |= test_pending_verify_failure_preserves_previous_page();
  rc |= test_equal_generation_tie_prefers_page1();
  rc |= test_generation_wrap_selects_zero_after_uint32_max();
  rc |= test_foc1_transitional_24byte_layout_not_shifted();
  rc |= test_foc1_legacy_16byte_layout_migrates_cautiously();
  rc |= test_wrong_schema_version_is_rejected();
  rc |= test_ambiguous_foc1_header_is_rejected();
  return rc;
}

static int check_ladrc_defaults_restored(const FlashParamData *image) {
  CHECK(image->ladrc_enable == (float)DEFAULT_LADRC_ENABLE);
  CHECK(image->ladrc_omega_o == DEFAULT_LADRC_OMEGA_O);
  CHECK(image->ladrc_omega_c == DEFAULT_LADRC_OMEGA_C);
  CHECK(image->ladrc_b0 == DEFAULT_LADRC_B0);
  CHECK(image->ladrc_max_output == DEFAULT_LADRC_MAX_OUT);
  return 0;
}
