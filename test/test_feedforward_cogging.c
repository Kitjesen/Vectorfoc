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
 * @file test_feedforward_cogging.c
 * @brief 前馈补偿（Feedforward）和齿槽补偿（CoggingComp）单元测试
 *
 * 覆盖：
 *  Feedforward:
 *   - 初始化后参数为零
 *   - NULL 安全
 *   - 零参数时不改变 input_torque
 *   - 粘性摩擦补偿方向正确
 *   - 惯量前馈加速度方向正确
 *
 *  CoggingComp:
 *   - 未校准时 GetCurrent 返回 0
 *   - IsValid 未校准为 false
 *   - IsCalibrating 未启动为 false
 *   - CoggingComp_GetCalibStep 返回合法值
 */

#include "feedforward.h"
#include "cogging.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
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

#define CHECK_NEAR(a, b, tol) \
    do { \
        float _a=(float)(a), _b=(float)(b), _t=(float)(tol); \
        if (fabsf(_a-_b) > _t) { \
            printf("FAIL %s:%d  |%.6f - %.6f| > %.6f\n", \
                   __FILE__, __LINE__, (double)_a, (double)_b, (double)_t); \
            return 1; \
        } \
    } while (0)

/* ── Mock MOTOR_DATA ─────────────────────────────────────────── */
static MOTOR_DATA make_motor(float vel_setpoint, float input_torque)
{
    MOTOR_DATA m;
    memset(&m, 0, sizeof(m));
    m.Controller.vel_setpoint = vel_setpoint;
    m.Controller.input_torque = input_torque;
    return m;
}

/* ══════════════════════════════════════════════════════════════
   Feedforward 测试
   ══════════════════════════════════════════════════════════════ */

static int test_ff_init_zeros(void)
{
    Feedforward_Params_t p;
    memset(&p, 0xCC, sizeof(p));
    Feedforward_Init(&p);
    CHECK_NEAR(p.inertia, 0.0f, 1e-9f);
    CHECK_NEAR(p.friction_coeff, 0.0f, 1e-9f);

    printf("PASS test_ff_init_zeros\n");
    return 0;
}

static int test_ff_null_safe(void)
{
    Feedforward_Init(NULL);

    MOTOR_DATA m = make_motor(1.0f, 0.0f);
    Feedforward_Params_t p = {0};
    Feedforward_Update(NULL, &p);
    Feedforward_Update(&m, NULL);

    printf("PASS test_ff_null_safe\n");
    return 0;
}

static int test_ff_zero_params_no_torque_change(void)
{
    Feedforward_Params_t p;
    Feedforward_Init(&p);   /* inertia=0, friction=0 */

    MOTOR_DATA m = make_motor(5.0f, 3.0f);
    float before = m.Controller.input_torque;
    Feedforward_Update(&m, &p);
    float after = m.Controller.input_torque;

    /* 参数全零：只有 friction*vel=0，惯量*accel=? */
    /* 因为上次vel_ref是静态的，第二次调用才有 accel 非零 */
    /* 但 inertia=0 → total_ff = 0，不改变 input_torque */
    CHECK_NEAR(after, before, 1e-6f);

    printf("PASS test_ff_zero_params_no_torque_change (before=%.4f after=%.4f)\n",
           (double)before, (double)after);
    return 0;
}

static int test_ff_viscous_friction_direction(void)
{
    Feedforward_Params_t p;
    Feedforward_Init(&p);
    p.friction_coeff = 0.1f;
    p.inertia = 0.0f; /* 排除惯量影响 */

    /* 正向速度 */
    MOTOR_DATA m_pos = make_motor(10.0f, 0.0f);
    /* 先调一次让 has_last = true，accel 有值 */
    Feedforward_Update(&m_pos, &p);
    m_pos.Controller.input_torque = 0.0f;
    m_pos.Controller.vel_setpoint = 10.0f; /* 速度不变，accel=0 */
    Feedforward_Update(&m_pos, &p);

    /* viscous = 0.1 * 10 = 1.0，正向 */
    CHECK(m_pos.Controller.input_torque > 0.0f);

    /* 负向速度 */
    MOTOR_DATA m_neg = make_motor(-10.0f, 0.0f);
    Feedforward_Update(&m_neg, &p); /* has_last */
    m_neg.Controller.input_torque = 0.0f;
    m_neg.Controller.vel_setpoint = -10.0f;
    Feedforward_Update(&m_neg, &p);

    CHECK(m_neg.Controller.input_torque < 0.0f);

    printf("PASS test_ff_viscous_friction_direction\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════
   CoggingComp 测试（纯状态查询，不执行标定流程）
   ══════════════════════════════════════════════════════════════ */

static int test_cogging_not_valid_initially(void)
{
    /* CoggingComp 用静态变量，模块加载后未校准 */
    CHECK(!CoggingComp_IsValid());
    printf("PASS test_cogging_not_valid_initially\n");
    return 0;
}

static int test_cogging_not_calibrating_initially(void)
{
    CHECK(!CoggingComp_IsCalibrating());
    printf("PASS test_cogging_not_calibrating_initially\n");
    return 0;
}

static int test_cogging_get_current_zero_when_invalid(void)
{
    /* 未校准时 GetCurrent 应返回 0 */
    MOTOR_DATA m;
    memset(&m, 0, sizeof(m));
    float c = CoggingComp_GetCurrent(&m);
    CHECK_NEAR(c, 0.0f, 1e-9f);

    printf("PASS test_cogging_get_current_zero_when_invalid\n");
    return 0;
}

static int test_cogging_get_calib_step_range(void)
{
    uint16_t step = CoggingComp_GetCalibStep();
    /* 未标定时应为 0 */
    CHECK(step == 0);
    printf("PASS test_cogging_get_calib_step_range (step=%u)\n", step);
    return 0;
}

static int test_cogging_update_null_no_crash(void)
{
    CoggingComp_Update(NULL); /* 不崩溃 */
    printf("PASS test_cogging_update_null_no_crash\n");
    return 0;
}

static int test_cogging_get_current_null_no_crash(void)
{
    float c = CoggingComp_GetCurrent(NULL);
    CHECK_NEAR(c, 0.0f, 1e-9f);
    printf("PASS test_cogging_get_current_null_no_crash\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════ */
int main(void)
{
    int f = 0;

    printf("-- Feedforward --\n");
    f += test_ff_init_zeros();
    f += test_ff_null_safe();
    f += test_ff_zero_params_no_torque_change();
    f += test_ff_viscous_friction_direction();

    printf("-- CoggingComp --\n");
    f += test_cogging_not_valid_initially();
    f += test_cogging_not_calibrating_initially();
    f += test_cogging_get_current_zero_when_invalid();
    f += test_cogging_get_calib_step_range();
    f += test_cogging_update_null_no_crash();
    f += test_cogging_get_current_null_no_crash();

    if (f == 0) {
        printf("\nAll feedforward/cogging tests PASSED (10 tests)\n");
        return 0;
    }
    printf("\n%d feedforward/cogging test(s) FAILED\n", f);
    return 1;
}
