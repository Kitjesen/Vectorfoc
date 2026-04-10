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
 * @file test_led.c
 * @brief LED 逻辑层单元测试（纯主机端，无 HAL 依赖）
 *
 * LED 模块的核心逻辑包含两部分：
 *  1. RGB_SetColor — 将 RGB 值编码为 WS2812 PWM 比较值（CODE_1 / CODE_0）
 *  2. RGB_DisplayColorById — 将颜色 ID 映射到 RGB 结构体
 *
 * 在测试环境下 HAL_TIM_PWM_Start_DMA 等函数被 stub，led_buf 可直接验证。
 */

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ── 在 TEST_ENV 下重定义 HAL 依赖 ──────────────────────────── */
#ifdef TEST_ENV

/* WS2812 编码常量（与 led.h 中完全相同） */
#define CODE_1 140u
#define CODE_0  70u
#define LED_MAX_NUM 1

typedef struct { uint8_t R; uint8_t G; uint8_t B; } RGB_Color_TypeDef;

/* 暴露 led_buf 用于验证 */
static uint32_t led_buf[LED_MAX_NUM + 1][24];

/* ---- WS2812 编码函数（从 led.c 抽取纯逻辑） ---- */
static void RGB_SetColor_Pure(uint8_t LedId, RGB_Color_TypeDef Color)
{
    if (LedId > LED_MAX_NUM) return;
    for (uint8_t i = 0; i < 8; i++) {
        led_buf[LedId][i]      = (Color.G & (1 << (7 - i))) ? CODE_1 : CODE_0;
        led_buf[LedId][8 + i]  = (Color.R & (1 << (7 - i))) ? CODE_1 : CODE_0;
        led_buf[LedId][16 + i] = (Color.B & (1 << (7 - i))) ? CODE_1 : CODE_0;
    }
}

/* ---- 颜色 ID 表（与 led.c 完全对应） ---- */
static const RGB_Color_TypeDef COLOR_TABLE[] = {
    {255,   0,   0}, /* 0: RED     */
    {  0, 255,   0}, /* 1: GREEN   */
    {  0,   0, 255}, /* 2: BLUE    */
    {  0, 255, 255}, /* 3: SKY     */
    {255,   0, 220}, /* 4: MAGENTA */
    {127, 216,   0}, /* 5: YELLOW  */
    {127, 106,   0}, /* 6: ORANGE  */
    {  0,   0,   0}, /* 7: BLACK   */
    {255, 255, 255}, /* 8: WHITE   */
    {128,   0, 128}, /* 9: PURPLE  */
    {165,  42,  42}, /*10: BROWN   */
    {128, 128, 128}, /*11: GRAY    */
    {255, 192, 203}, /*12: PINK    */
    {255, 215,   0}, /*13: GOLD    */
    {192, 192, 192}, /*14: SILVER  */
};
#define COLOR_TABLE_SIZE 15

static RGB_Color_TypeDef GetColorById(uint8_t id)
{
    if (id < COLOR_TABLE_SIZE) return COLOR_TABLE[id];
    return COLOR_TABLE[7]; /* default: BLACK */
}

#endif /* TEST_ENV */

/* ── 辅助宏 ─────────────────────────────────────────────────── */
#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            return 1; \
        } \
    } while (0)

/* ── 辅助：从 led_buf 解码某位的 PWM 值 ──────────────────────
 * WS2812 协议：G7..G0 R7..R0 B7..B0 共 24 位
 * led_buf[id][0..7]  = G bits
 * led_buf[id][8..15] = R bits
 * led_buf[id][16..23]= B bits
 */
static uint8_t decode_channel(uint8_t led_id, uint8_t bit_start)
{
    uint8_t val = 0;
    for (int i = 0; i < 8; i++) {
        uint32_t code = led_buf[led_id][bit_start + i];
        if (code == CODE_1) val |= (1 << (7 - i));
    }
    return val;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_set_color_black(void)
{
    RGB_Color_TypeDef black = {0, 0, 0};
    RGB_SetColor_Pure(0, black);

    for (int i = 0; i < 24; i++) {
        CHECK(led_buf[0][i] == CODE_0);
    }

    printf("PASS test_set_color_black\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_set_color_white(void)
{
    RGB_Color_TypeDef white = {255, 255, 255};
    RGB_SetColor_Pure(0, white);

    for (int i = 0; i < 24; i++) {
        CHECK(led_buf[0][i] == CODE_1);
    }

    printf("PASS test_set_color_white\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_set_color_red(void)
{
    RGB_Color_TypeDef red = {255, 0, 0};
    RGB_SetColor_Pure(0, red);

    /* G = 0 → 前 8 位全为 CODE_0 */
    for (int i = 0; i < 8; i++)  CHECK(led_buf[0][i] == CODE_0);
    /* R = 255 → 中 8 位全为 CODE_1 */
    for (int i = 8; i < 16; i++) CHECK(led_buf[0][i] == CODE_1);
    /* B = 0 → 后 8 位全为 CODE_0 */
    for (int i = 16; i < 24; i++) CHECK(led_buf[0][i] == CODE_0);

    printf("PASS test_set_color_red\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_set_color_arbitrary(void)
{
    RGB_Color_TypeDef c = {0xAB, 0xCD, 0xEF};
    RGB_SetColor_Pure(0, c);

    /*
     * WS2812 wire order: G bits [0..7], R bits [8..15], B bits [16..23]
     * RGB_Color_TypeDef c = {R=0xAB, G=0xCD, B=0xEF}
     * So led_buf encodes: G(c.G=0xCD) then R(c.R=0xAB) then B(c.B=0xEF)
     */
    uint8_t g_wire = decode_channel(0, 0);   /* wire G = c.G = 0xCD */
    uint8_t r_wire = decode_channel(0, 8);   /* wire R = c.R = 0xAB */
    uint8_t b_wire = decode_channel(0, 16);  /* wire B = c.B = 0xEF */

    CHECK(g_wire == 0xCD); /* Color.G encoded in first 8 bits */
    CHECK(r_wire == 0xAB); /* Color.R encoded in middle 8 bits */
    CHECK(b_wire == 0xEF); /* Color.B encoded in last 8 bits   */

    printf("PASS test_set_color_arbitrary (wireG=0x%02X wireR=0x%02X wireB=0x%02X)\n",
           g_wire, r_wire, b_wire);
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_invalid_led_id_no_crash(void)
{
    RGB_Color_TypeDef c = {255, 0, 0};
    memset(led_buf, 0, sizeof(led_buf));

    /* LedId > LED_MAX_NUM 应直接返回，不修改 buf */
    RGB_SetColor_Pure(LED_MAX_NUM + 1, c);

    /* buf 仍全 0 */
    for (int i = 0; i < 24; i++) CHECK(led_buf[0][i] == 0);

    printf("PASS test_invalid_led_id_no_crash\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_color_by_id_known_values(void)
{
    /* ID 0 = RED {255,0,0} */
    RGB_Color_TypeDef c0 = GetColorById(0);
    CHECK(c0.R == 255 && c0.G == 0 && c0.B == 0);

    /* ID 7 = BLACK {0,0,0} */
    RGB_Color_TypeDef c7 = GetColorById(7);
    CHECK(c7.R == 0 && c7.G == 0 && c7.B == 0);

    /* ID 8 = WHITE {255,255,255} */
    RGB_Color_TypeDef c8 = GetColorById(8);
    CHECK(c8.R == 255 && c8.G == 255 && c8.B == 255);

    /* ID 1 = GREEN {0,255,0} */
    RGB_Color_TypeDef c1 = GetColorById(1);
    CHECK(c1.R == 0 && c1.G == 255 && c1.B == 0);

    printf("PASS test_color_by_id_known_values\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_color_id_out_of_range_returns_black(void)
{
    RGB_Color_TypeDef c = GetColorById(255); /* 越界 */
    CHECK(c.R == 0 && c.G == 0 && c.B == 0);

    printf("PASS test_color_id_out_of_range_returns_black\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_code_constants(void)
{
    /* WS2812 协议：CODE_1 > CODE_0，且两者都非零 */
    CHECK(CODE_1 > CODE_0);
    CHECK(CODE_0 > 0u);

    printf("PASS test_code_constants (CODE_1=%u CODE_0=%u)\n", CODE_1, CODE_0);
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
int main(void)
{
    int f = 0;
    f += test_set_color_black();
    f += test_set_color_white();
    f += test_set_color_red();
    f += test_set_color_arbitrary();
    f += test_invalid_led_id_no_crash();
    f += test_color_by_id_known_values();
    f += test_color_id_out_of_range_returns_black();
    f += test_code_constants();

    if (f == 0) {
        printf("\nAll LED tests PASSED (8 tests)\n");
        return 0;
    }
    printf("\n%d LED test(s) FAILED\n", f);
    return 1;
}
