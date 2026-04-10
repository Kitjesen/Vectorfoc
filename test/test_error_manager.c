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
 * @file test_error_manager.c
 * @brief ErrorManager 单元测试
 *
 * 覆盖：
 *  - 初始化与重置
 *  - 上报错误 → 历史记录写入、统计计数
 *  - 严重等级分类（INFO / WARNING / MAJOR / CRITICAL）
 *  - active fault 查询
 *  - 清除操作（ClearAll / ClearDomain / ClearActiveFaults）
 *  - 回调注册与触发
 *  - 宏 ERROR_REPORT / ERROR_REPORT_CODE
 *  - 历史环形缓冲区溢出时不崩溃
 *  - GetDomainName / GetSeverityName / FormatError
 */

#include "error_manager.h"
#include "error_types.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* Provide non-inline HAL_GetTick for error_manager.c's extern declaration */
uint32_t HAL_GetTick(void) {
    return (uint32_t)(clock() * 1000u / CLOCKS_PER_SEC);
}

/* ── 测试辅助宏 ────────────────────────────────────────────── */
#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            return 1; \
        } \
    } while (0)

/* ── 回调支持 ───────────────────────────────────────────────── */
static int s_cb_count = 0;
static uint32_t s_cb_last_code = 0;

static void test_callback(const ErrorRecord *rec)
{
    s_cb_count++;
    if (rec) s_cb_last_code = rec->error_code;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_init_clean_state(void)
{
    ErrorManager_Init();
    const ErrorStatistics *st = ErrorManager_GetStatistics();
    CHECK(st != NULL);
    CHECK(st->total_count == 0);
    CHECK(st->info_count  == 0);
    CHECK(st->warning_count == 0);
    CHECK(st->major_count == 0);
    CHECK(st->critical_count == 0);
    CHECK(ErrorManager_GetHistoryCount() == 0);
    CHECK(!ErrorManager_HasActiveFault());

    printf("PASS test_init_clean_state\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_report_increments_stats(void)
{
    ErrorManager_Init();

    ErrorManager_Report(ERROR_SYSTEM_TIMEOUT, "timeout");          /* WARNING */
    ErrorManager_Report(ERROR_HW_CAN_TX_FULL, "can full");         /* WARNING */
    ErrorManager_Report(ERROR_MOTOR_ENCODER_LOSS, "enc loss");     /* MAJOR */

    const ErrorStatistics *st = ErrorManager_GetStatistics();
    CHECK(st->total_count   == 3);
    CHECK(st->warning_count == 2);
    CHECK(st->major_count   == 1);

    printf("PASS test_report_increments_stats\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_history_records_written(void)
{
    ErrorManager_Init();

    ErrorManager_Report(ERROR_MOTOR_STALL, "stall");

    uint16_t cnt = ErrorManager_GetHistoryCount();
    CHECK(cnt >= 1);

    const ErrorRecord *r = ErrorManager_GetHistory(0);
    CHECK(r != NULL);
    CHECK(r->error_code == ERROR_MOTOR_STALL);

    printf("PASS test_history_records_written\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_active_fault_set_cleared(void)
{
    ErrorManager_Init();

    /* INFO 级不设置 active fault */
    ErrorManager_Report(
        ERROR_CODE(ERROR_SEVERITY_INFO, ERROR_DOMAIN_SYSTEM, 0, 0x01),
        "info msg");
    CHECK(!ErrorManager_HasActiveFault());

    /* MAJOR 级应设置 active fault */
    ErrorManager_Report(ERROR_MOTOR_ENCODER_LOSS, "enc loss");
    CHECK(ErrorManager_HasActiveFault());

    /* 清除后应消失 */
    ErrorManager_ClearActiveFaults();
    CHECK(!ErrorManager_HasActiveFault());

    printf("PASS test_active_fault_set_cleared\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_clear_all(void)
{
    ErrorManager_Init();
    /* 使用 MAJOR 级别（非 CRITICAL，避免触发 while(1) reset loop） */
    ErrorManager_Report(ERROR_HW_CAN_INIT, "can init");   /* MAJOR */
    ErrorManager_Report(ERROR_MOTOR_STALL, "stall");      /* MAJOR */
    CHECK(ErrorManager_GetHistoryCount() >= 1);

    ErrorManager_ClearAll();

    const ErrorStatistics *st = ErrorManager_GetStatistics();
    CHECK(st->total_count == 0);
    /* 历史可能不清零（实现可选），但 fault 必须清 */
    CHECK(!ErrorManager_HasActiveFault());

    printf("PASS test_clear_all\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_clear_domain(void)
{
    ErrorManager_Init();

    /* 上报 MOTOR 和 HARDWARE 各一个 MAJOR fault */
    ErrorManager_Report(ERROR_MOTOR_STALL, "stall");
    ErrorManager_Report(ERROR_HW_PWM_INIT, "pwm");
    CHECK(ErrorManager_HasActiveFault());

    /* 只清 MOTOR domain */
    ErrorManager_ClearDomain(ERROR_DOMAIN_MOTOR);

    /* HARDWARE fault 应仍存在 */
    CHECK(ErrorManager_HasFaultInDomain(ERROR_DOMAIN_HARDWARE));

    printf("PASS test_clear_domain\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_callback_invoked(void)
{
    ErrorManager_Init();
    s_cb_count = 0;
    s_cb_last_code = 0;

    bool ok = ErrorManager_RegisterCallback(test_callback);
    CHECK(ok);

    ErrorManager_Report(ERROR_HW_CAN_INIT, "can init");
    CHECK(s_cb_count == 1);
    CHECK(s_cb_last_code == ERROR_HW_CAN_INIT);

    /* 第二次上报再触发一次 */
    ErrorManager_Report(ERROR_MOTOR_STALL, "stall");
    CHECK(s_cb_count == 2);

    /* 注销后不再触发（使用 MAJOR 级别） */
    ErrorManager_UnregisterCallback(test_callback);
    ErrorManager_Report(ERROR_HW_CAN_INIT, "can2");
    CHECK(s_cb_count == 2); /* 未增加 */

    printf("PASS test_callback_invoked\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_ERROR_REPORT_macro(void)
{
    ErrorManager_Init();
    uint16_t before = ErrorManager_GetHistoryCount();

    ERROR_REPORT(ERROR_CALIB_ENCODER_FAILED, "enc calib fail");

    uint16_t after = ErrorManager_GetHistoryCount();
    CHECK(after > before);

    printf("PASS test_ERROR_REPORT_macro\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_history_overflow_no_crash(void)
{
    ErrorManager_Init();

    /* 写超过 ERROR_HISTORY_SIZE 条，不应崩溃 */
    for (int i = 0; i < ERROR_HISTORY_SIZE + 10; i++) {
        ErrorManager_Report(ERROR_SYSTEM_TIMEOUT, "flood");
    }

    /* 最新一条仍可读 */
    const ErrorRecord *r = ErrorManager_GetHistory(0);
    CHECK(r != NULL);

    printf("PASS test_history_overflow_no_crash\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_name_helpers(void)
{
    const char *dn = ErrorManager_GetDomainName(ERROR_DOMAIN_MOTOR);
    CHECK(dn != NULL && strlen(dn) > 0);

    const char *sn = ErrorManager_GetSeverityName(ERROR_SEVERITY_MAJOR);
    CHECK(sn != NULL && strlen(sn) > 0);

    char buf[64];
    int n = ErrorManager_FormatError(ERROR_MOTOR_STALL, buf, sizeof(buf));
    CHECK(n > 0);
    CHECK(strlen(buf) > 0);

    printf("PASS test_name_helpers (domain='%s' severity='%s' fmt='%s')\n",
           dn, sn, buf);
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
static int test_has_fault_in_domain(void)
{
    ErrorManager_Init();
    CHECK(!ErrorManager_HasFaultInDomain(ERROR_DOMAIN_SAFETY));

    ErrorManager_Report(ERROR_SAFETY_OVERCURRENT, "oc");
    CHECK(ErrorManager_HasFaultInDomain(ERROR_DOMAIN_SAFETY));
    CHECK(!ErrorManager_HasFaultInDomain(ERROR_DOMAIN_MOTOR));

    printf("PASS test_has_fault_in_domain\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
int main(void)
{
    int f = 0;
    f += test_init_clean_state();
    f += test_report_increments_stats();
    f += test_history_records_written();
    f += test_active_fault_set_cleared();
    f += test_clear_all();
    f += test_clear_domain();
    f += test_callback_invoked();
    f += test_ERROR_REPORT_macro();
    f += test_history_overflow_no_crash();
    f += test_name_helpers();
    f += test_has_fault_in_domain();

    if (f == 0) {
        printf("\nAll ErrorManager tests PASSED (11 tests)\n");
        return 0;
    }
    printf("\n%d ErrorManager test(s) FAILED\n", f);
    return 1;
}
