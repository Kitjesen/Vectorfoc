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

#if defined(__has_include)
#  if __has_include("runtime_compensation.h")
#    include "runtime_compensation.h"
#  else
#    include "torque_feedforward.h"
#  endif
#else
#  include "torque_feedforward.h"
#endif
#include "cogging_compensation.h"
#include "motor_runtime.h"
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

/* Adapt only the API when comparing a saved source tree. */
static void UpdateFeedforward(MOTOR_DATA *motor) {
#ifdef CONTROL_TEST_LEGACY_FEEDFORWARD
    Feedforward_Params_t params = {0};
    if (motor) {
        params.inertia = motor->Controller.inertia;
        params.friction_coeff = motor->advanced.ff_friction;
    }
    Feedforward_Update(motor, &params);
#else
    Feedforward_Update(motor);
#endif
}

static int TraceFeedforward(void) {
    MOTOR_DATA motor = {0};
    for (unsigned i = 0; i < 1024; ++i) {
        motor.Controller.vel_setpoint = ((int)(i % 37) - 18) * 0.125f;
        motor.Controller.input_torque = ((int)(i % 5) - 2) * 0.25f;
        motor.Controller.inertia = (i % 7) * 0.001f;
        motor.advanced.ff_friction = (i % 11) * 0.01f;
        if (i % 13 == 0) UpdateFeedforward(NULL);
        UpdateFeedforward(&motor);
        uint32_t bits;
        memcpy(&bits, &motor.Controller.input_torque, sizeof(bits));
        printf("%08x\n", (unsigned)bits);
    }
    puts("PASS feedforward cycles=1024 parameter_changes=null_calls=covered");
    return 0;
}

/* ── Mock MOTOR_DATA ─────────────────────────────────────────── */
static MOTOR_DATA make_motor(float vel_setpoint, float input_torque)
{
    MOTOR_DATA m;
    memset(&m, 0, sizeof(m));
    m.Controller.vel_setpoint = vel_setpoint;
    m.Controller.input_torque = input_torque;
    m.Controller.torque_const = 1.0f;
    m.state.Control_Mode = CONTROL_MODE_VELOCITY;
    return m;
}

/* ══════════════════════════════════════════════════════════════
   Feedforward 测试
   ══════════════════════════════════════════════════════════════ */

static int test_ff_null_safe(void)
{

    UpdateFeedforward(NULL);

    printf("PASS test_ff_null_safe\n");
    return 0;
}

static int test_ff_zero_params_no_torque_change(void)
{

    MOTOR_DATA m = make_motor(5.0f, 3.0f);
    Feedforward_Reset();
    float before = m.Controller.input_torque;
    UpdateFeedforward(&m);
    float after = Feedforward_GetCurrent(&m);

    /* 参数全零：只有 friction*vel=0，惯量*accel=? */
    /* 因为上次vel_ref是静态的，第二次调用才有 accel 非零 */
    /* 但 inertia=0 → total_ff = 0，不改变 input_torque */
    CHECK_NEAR(after, 0.0f, 1e-6f);
    CHECK_NEAR(m.Controller.input_torque, before, 1e-6f);

    printf("PASS test_ff_zero_params_no_torque_change (before=%.4f after=%.4f)\n",
           (double)before, (double)after);
    return 0;
}

static int test_ff_viscous_friction_direction(void)
{

    /* 正向速度 */
    MOTOR_DATA m_pos = make_motor(10.0f, 0.0f);
    m_pos.advanced.ff_friction = 0.1f;
    Feedforward_Reset();
    /* 先调一次让 has_last = true，accel 有值 */
    UpdateFeedforward(&m_pos);
    m_pos.Controller.input_torque = 0.0f;
    m_pos.Controller.vel_setpoint = 10.0f; /* 速度不变，accel=0 */
    UpdateFeedforward(&m_pos);

    /* viscous = 0.1 * 10 = 1.0，正向 */
    CHECK(Feedforward_GetCurrent(&m_pos) > 0.0f);

    /* 负向速度 */
    MOTOR_DATA m_neg = make_motor(-10.0f, 0.0f);
    m_neg.advanced.ff_friction = 0.1f;
    Feedforward_Reset();
    UpdateFeedforward(&m_neg); /* has_last */
    m_neg.Controller.input_torque = 0.0f;
    m_neg.Controller.vel_setpoint = -10.0f;
    UpdateFeedforward(&m_neg);

    CHECK(Feedforward_GetCurrent(&m_neg) < 0.0f);

    printf("PASS test_ff_viscous_friction_direction\n");
    return 0;
}

static int test_ff_ramp_mode_does_not_repeat_trajectory_inertia(void)
{
    MOTOR_DATA m = make_motor(0.0f, 0.0f);
    m.Controller.inertia = 0.01f;
    m.state.Control_Mode = CONTROL_MODE_VELOCITY_RAMP;
    Feedforward_Reset();
    UpdateFeedforward(&m);
    m.Controller.vel_setpoint = 10.0f;
    UpdateFeedforward(&m);
    CHECK_NEAR(Feedforward_GetCurrent(&m), 0.0f, 1e-6f);

    printf("PASS test_ff_ramp_mode_does_not_repeat_trajectory_inertia\n");
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
int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--trace") == 0) return TraceFeedforward();
    int f = 0;

    printf("-- Feedforward --\n");
    f += test_ff_null_safe();
    f += test_ff_zero_params_no_torque_change();
    f += test_ff_viscous_friction_direction();
    f += test_ff_ramp_mode_does_not_repeat_trajectory_inertia();

    printf("-- CoggingComp --\n");
    f += test_cogging_not_valid_initially();
    f += test_cogging_not_calibrating_initially();
    f += test_cogging_get_current_zero_when_invalid();
    f += test_cogging_get_calib_step_range();
    f += test_cogging_update_null_no_crash();
    f += test_cogging_get_current_null_no_crash();

    if (f == 0) {
        printf("\nAll feedforward/cogging tests PASSED (9 tests)\n");
        return 0;
    }
    printf("\n%d feedforward/cogging test(s) FAILED\n", f);
    return 1;
}
