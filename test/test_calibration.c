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
 * @file test_calibration.c
 * @brief 标定上下文（CalibrationContext）单元测试
 *
 * 覆盖：
 *  - CalibContext_Init：字段初始化、error_array 指针、is_initialized 标志
 *  - CalibContext_Release：error_array 指针复原
 *  - CalibContext_Reset：等价于 Release + Init
 *  - CalibContext_GetProgress：各阶段 / cs_state 进度映射
 *  - CalibResult 枚举存在性验证
 */

#include "calibration_context.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* ── 辅助宏 ─────────────────────────────────────────────────── */
#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            return 1; \
        } \
    } while (0)

/* ══════════════════════════════════════════════════════════════ */
static int test_init_sets_fields(void)
{
    CalibrationContext ctx;
    memset(&ctx, 0xAB, sizeof(ctx)); /* 脏数据 */

    CalibContext_Init(&ctx);

    /* 电流标定：is_initialized 应为 true */
    CHECK(ctx.current.is_initialized == true);
    CHECK(ctx.current.loop_count == 0);

    /* 电阻标定：kI 应为 2.0 */
    CHECK(ctx.resistance.kI == 2.0f);

    /* 编码器：error_array 指向内置 storage */
    CHECK(ctx.encoder.error_array == ctx.encoder.error_array_storage);
    CHECK(ctx.encoder.error_array_size == SAMPLES_PER_POLE_PAIR * MAX_POLE_PAIRS);
    CHECK(ctx.encoder.sample_count == 0);

    printf("PASS test_init_sets_fields\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_init_null_safe(void)
{
    CalibContext_Init(NULL);    /* 不崩溃 */
    CalibContext_Release(NULL);
    CalibContext_Reset(NULL);

    printf("PASS test_init_null_safe\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_release_restores_pointer(void)
{
    CalibrationContext ctx;
    CalibContext_Init(&ctx);

    /* 模拟指针被清空（如 RSLSCalib_Start 的 memset） */
    ctx.encoder.error_array = NULL;
    ctx.encoder.error_array_size = 0;

    CalibContext_Release(&ctx);

    CHECK(ctx.encoder.error_array == ctx.encoder.error_array_storage);
    CHECK(ctx.encoder.error_array_size == SAMPLES_PER_POLE_PAIR * MAX_POLE_PAIRS);

    printf("PASS test_release_restores_pointer\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_reset_is_init(void)
{
    CalibrationContext ctx;
    CalibContext_Init(&ctx);

    /* 写入一些数据 */
    ctx.current.loop_count = 500;
    ctx.resistance.kI = 99.0f;

    CalibContext_Reset(&ctx);

    /* 应恢复初始值 */
    CHECK(ctx.current.loop_count == 0);
    CHECK(ctx.resistance.kI == 2.0f);
    CHECK(ctx.encoder.error_array == ctx.encoder.error_array_storage);

    printf("PASS test_reset_is_init\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_get_progress_idle(void)
{
    CalibrationContext ctx;
    CalibContext_Init(&ctx);

    uint8_t p = CalibContext_GetProgress(0 /*SUB_STATE_IDLE*/, 0, &ctx);
    CHECK(p == 0);

    printf("PASS test_get_progress_idle\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_get_progress_current(void)
{
    CalibrationContext ctx;
    CalibContext_Init(&ctx);

    /* 未开始 */
    ctx.current.loop_count = 0;
    uint8_t p0 = CalibContext_GetProgress(1 /*CURRENT_CALIBRATING*/, 0, &ctx);
    CHECK(p0 == 0);

    /* 完成 */
    ctx.current.loop_count = CURRENT_CALIB_CYCLES;
    uint8_t p1 = CalibContext_GetProgress(1, 0, &ctx);
    CHECK(p1 == 10);

    /* 中间值应在 [0, 10] */
    ctx.current.loop_count = CURRENT_CALIB_CYCLES / 2;
    uint8_t pm = CalibContext_GetProgress(1, 0, &ctx);
    CHECK(pm >= 0 && pm <= 10);

    printf("PASS test_get_progress_current (half=%u%%)\n", pm);
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_get_progress_flux(void)
{
    CalibrationContext ctx;
    CalibContext_Init(&ctx);

    /* 磁链未开始 → 70% */
    ctx.flux.loop_count = 0;
    uint8_t p0 = CalibContext_GetProgress(3 /*FLUX_CALIBRATING*/, 0, &ctx);
    CHECK(p0 == 70);

    /* 磁链完成 → 100% */
    ctx.flux.loop_count = FLUX_CALIB_CYCLES;
    uint8_t p1 = CalibContext_GetProgress(3, 0, &ctx);
    CHECK(p1 == 100);

    printf("PASS test_get_progress_flux\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_get_progress_null(void)
{
    uint8_t p = CalibContext_GetProgress(1, 0, NULL);
    CHECK(p == 0);

    printf("PASS test_get_progress_null\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_calib_result_values(void)
{
    /* 枚举值存在性验证（编译期也会报错，运行期二次确认） */
    CalibResult r = CALIB_SUCCESS;
    CHECK(r == CALIB_SUCCESS);
    r = CALIB_IN_PROGRESS;
    CHECK(r != CALIB_SUCCESS);
    r = CALIB_FAILED_NO_MOVEMENT;
    CHECK(r != CALIB_SUCCESS && r != CALIB_IN_PROGRESS);

    printf("PASS test_calib_result_values\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_error_array_within_storage(void)
{
    CalibrationContext ctx;
    CalibContext_Init(&ctx);

    /* error_array 应指向 storage 区域内 */
    CHECK(ctx.encoder.error_array >= ctx.encoder.error_array_storage);
    CHECK(ctx.encoder.error_array <=
          ctx.encoder.error_array_storage + SAMPLES_PER_POLE_PAIR * MAX_POLE_PAIRS);

    printf("PASS test_error_array_within_storage\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   STUB — RSLSCalib_GetProgress は calibration_context.c から呼ばれる
   ════════════════════════════════════════════════════════════════ */
#include "rsls_calib.h"
uint8_t RSLSCalib_GetProgress(CalibrationContext *ctx) { (void)ctx; return 50; }

/* ══════════════════════════════════════════════════════════════ */
int main(void)
{
    int f = 0;
    f += test_init_sets_fields();
    f += test_init_null_safe();
    f += test_release_restores_pointer();
    f += test_reset_is_init();
    f += test_get_progress_idle();
    f += test_get_progress_current();
    f += test_get_progress_flux();
    f += test_get_progress_null();
    f += test_calib_result_values();
    f += test_error_array_within_storage();

    if (f == 0) {
        printf("\nAll calibration context tests PASSED (10 tests)\n");
        return 0;
    }
    printf("\n%d calibration context test(s) FAILED\n", f);
    return 1;
}
