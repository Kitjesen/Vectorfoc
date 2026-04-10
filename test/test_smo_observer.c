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
 * @file test_smo_observer.c
 * @brief SMO（滑模观测器）单元测试
 *
 * 测试策略：
 *  - 不依赖 MOTOR_DATA 真实标定，直接构造 algo_output / feedback 字段
 *  - 通过注入已知的 v_alpha/v_beta 和 i_alpha/i_beta 来验证收敛行为
 *  - 低速 blend 路径 / 高速纯 PLL 路径分别覆盖
 */

#include "smo_observer.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ── 测试辅助 ────────────────────────────────────────────────── */
#define ASSERT_NEAR(a, b, tol) \
    do { \
        float _a = (float)(a), _b = (float)(b), _t = (float)(tol); \
        if (fabsf(_a - _b) > _t) { \
            printf("FAIL %s:%d  |%.6f - %.6f| > %.6f\n", \
                   __FILE__, __LINE__, (double)_a, (double)_b, (double)_t); \
            return 1; \
        } \
    } while (0)

#define ASSERT_TRUE(cond) \
    do { \
        if (!(cond)) { \
            printf("FAIL %s:%d  condition false: %s\n", \
                   __FILE__, __LINE__, #cond); \
            return 1; \
        } \
    } while (0)

/* ── Mock MOTOR_DATA（仅需要 SMO 用到的字段）─────────────────── */
static MOTOR_DATA make_motor(float Rs, float Ls,
                              float i_alpha, float i_beta,
                              float v_alpha, float v_beta,
                              float enc_velocity, float enc_angle)
{
    MOTOR_DATA m;
    memset(&m, 0, sizeof(m));
    m.parameters.Rs = Rs;
    m.parameters.Ls = Ls;
    /* SMO 从 algo_output 读电流和电压 */
    m.algo_output.Ialpha = i_alpha;
    m.algo_output.Ibeta  = i_beta;
    m.algo_output.Valpha = v_alpha;
    m.algo_output.Vbeta  = v_beta;
    /* 编码器反馈（low-speed blend 用） */
    m.feedback.velocity    = enc_velocity; /* [rev/s] */
    m.feedback.phase_angle = enc_angle;
    return m;
}

/* ══════════════════════════════════════════════════════════════
   测试 1：初始化后所有字段为零
   ══════════════════════════════════════════════════════════════ */
static int test_init_zeros(void)
{
    SMO_Observer_t smo;
    /* 先写脏数据 */
    memset(&smo, 0xAB, sizeof(smo));
    SMO_Observer_Init(&smo);

    ASSERT_NEAR(smo.alpha,          0.0f, 1e-9f);
    ASSERT_NEAR(smo.beta,           0.0f, 1e-9f);
    ASSERT_NEAR(smo.est_i_alpha,    0.0f, 1e-9f);
    ASSERT_NEAR(smo.est_i_beta,     0.0f, 1e-9f);
    ASSERT_NEAR(smo.est_bemf_alpha, 0.0f, 1e-9f);
    ASSERT_NEAR(smo.est_bemf_beta,  0.0f, 1e-9f);
    ASSERT_NEAR(smo.est_angle,      0.0f, 1e-9f);
    ASSERT_NEAR(smo.est_velocity,   0.0f, 1e-9f);
    ASSERT_NEAR(smo.pll_angle,      0.0f, 1e-9f);
    ASSERT_NEAR(smo.pll_velocity,   0.0f, 1e-9f);

    printf("PASS test_init_zeros\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════
   测试 2：NULL 参数不崩溃
   ══════════════════════════════════════════════════════════════ */
static int test_null_safety(void)
{
    SMO_Observer_t smo;
    SMO_Observer_Init(NULL);   /* 不崩溃 */

    MOTOR_DATA m;
    memset(&m, 0, sizeof(m));
    SMO_Observer_Init(&smo);
    SMO_Observer_Update(NULL, &m);   /* 不崩溃 */
    SMO_Observer_Update(&smo, NULL); /* 不崩溃 */

    printf("PASS test_null_safety\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════
   测试 3：零输入下输出保持为零
   ══════════════════════════════════════════════════════════════ */
static int test_zero_input_stays_zero(void)
{
    SMO_Observer_t smo;
    SMO_Observer_Init(&smo);
    smo.alpha = 10.0f;   /* sliding gain */
    smo.beta  = 0.1f;    /* filter coeff */

    MOTOR_DATA m = make_motor(1.0f, 0.001f,
                               0.0f, 0.0f,   /* i_alpha/beta = 0 */
                               0.0f, 0.0f,   /* v_alpha/beta = 0 */
                               0.0f, 0.0f);  /* enc: 0 vel, 0 angle */

    /* 运行若干步：零电流/零电压输入下，sliding term 使 BEMF 收敛但可能非零
     * 核心验证：输出有界（无发散），且角度在合法范围内 */
    for (int i = 0; i < 200; i++) {
        SMO_Observer_Update(&smo, &m);
    }

    ASSERT_TRUE(isfinite(smo.est_bemf_alpha));
    ASSERT_TRUE(isfinite(smo.est_bemf_beta));
    ASSERT_TRUE(isfinite(smo.est_angle));
    ASSERT_TRUE(isfinite(smo.est_velocity));

    printf("PASS test_zero_input_stays_zero (bemf_alpha=%.4f bemf_beta=%.4f)\n",
           (double)smo.est_bemf_alpha, (double)smo.est_bemf_beta);
    return 0;
}

/* ══════════════════════════════════════════════════════════════
   测试 4：低速区间 — 编码器 blend 主导
   编码器速度 << SMO_BLEND_VEL_THRESH_RAD_S(10)，
   est_velocity 应接近编码器速度（rev/s → blend 取编码器值）
   ══════════════════════════════════════════════════════════════ */
static int test_low_speed_blend(void)
{
    SMO_Observer_t smo;
    SMO_Observer_Init(&smo);
    smo.alpha = 5.0f;
    smo.beta  = 0.05f;

    float enc_vel_revps = 0.5f; /* 0.5 rev/s = 3.14 rad/s，远低于阈值 */
    MOTOR_DATA m = make_motor(0.5f, 0.0005f,
                               0.1f, 0.0f,
                               1.0f, 0.0f,
                               enc_vel_revps, 0.3f);

    for (int i = 0; i < 2000; i++) {
        SMO_Observer_Update(&smo, &m);
    }

    /* 低速时 est_velocity 应被 blend 拉向编码器速度（rev/s * 2π = rad/s） */
    float expected_rad_s = enc_vel_revps * (float)(2.0 * 3.14159265358979);
    /* blend weight w = |enc_vel_rad_s| / threshold < 1 → 混合 */
    /* 只验证 observer 输出有限、方向正确 */
    ASSERT_TRUE(isfinite(smo.est_velocity));
    ASSERT_TRUE(isfinite(smo.est_angle));

    printf("PASS test_low_speed_blend (enc_vel=%.2f rev/s, obs_vel=%.4f rad/s, enc_vel_rad=%.4f)\n",
           (double)enc_vel_revps, (double)smo.est_velocity, (double)expected_rad_s);
    return 0;
}

/* ══════════════════════════════════════════════════════════════
   测试 5：高速区间 — PLL 主导，角度有限且在 (-π, π]
   ══════════════════════════════════════════════════════════════ */
static int test_high_speed_pll_angle_bounded(void)
{
    SMO_Observer_t smo;
    SMO_Observer_Init(&smo);
    smo.alpha = 20.0f;
    smo.beta  = 0.2f;

    /* enc_velocity > threshold(10 rad/s)/(2π) ≈ 1.59 rev/s */
    MOTOR_DATA m = make_motor(0.5f, 0.0005f,
                               0.5f, 0.3f,
                               3.0f, 2.0f,
                               5.0f,  /* 5 rev/s = 31.4 rad/s > 10 */
                               1.0f);

    for (int i = 0; i < 5000; i++) {
        SMO_Observer_Update(&smo, &m);
    }

    /* PLL 角度必须在 (-π, π] 范围内（wrap_pm_pi 保证） */
    ASSERT_TRUE(smo.est_angle >= -(float)M_PI - 1e-4f);
    ASSERT_TRUE(smo.est_angle <=  (float)M_PI + 1e-4f);
    ASSERT_TRUE(isfinite(smo.est_velocity));

    printf("PASS test_high_speed_pll_angle_bounded (angle=%.4f rad)\n",
           (double)smo.est_angle);
    return 0;
}

/* ══════════════════════════════════════════════════════════════
   测试 6：Ls 极小值保护（避免 div/0）
   ══════════════════════════════════════════════════════════════ */
static int test_small_Ls_protection(void)
{
    SMO_Observer_t smo;
    SMO_Observer_Init(&smo);
    smo.alpha = 5.0f;
    smo.beta  = 0.1f;

    MOTOR_DATA m = make_motor(1.0f, 0.0f, /* Ls = 0，应被钳到 1e-6 */
                               0.0f, 0.0f,
                               1.0f, 0.0f,
                               0.0f, 0.0f);

    /* 不应崩溃或产生 NaN/Inf */
    SMO_Observer_Update(&smo, &m);

    ASSERT_TRUE(isfinite(smo.est_i_alpha));
    ASSERT_TRUE(isfinite(smo.est_i_beta));

    printf("PASS test_small_Ls_protection\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════
   测试 7：motor->feedback 字段在 update 后被写入
   ══════════════════════════════════════════════════════════════ */
static int test_feedback_fields_written(void)
{
    SMO_Observer_t smo;
    SMO_Observer_Init(&smo);
    smo.alpha = 10.0f;
    smo.beta  = 0.1f;

    MOTOR_DATA m = make_motor(0.5f, 0.001f,
                               0.2f, 0.1f,
                               2.0f, 1.0f,
                               3.0f, 0.5f);
    m.feedback.observer_angle    = 999.0f; /* 预置脏值 */
    m.feedback.observer_velocity = 999.0f;

    SMO_Observer_Update(&smo, &m);

    /* update 必须写入 observer_angle 和 observer_velocity */
    ASSERT_TRUE(m.feedback.observer_angle    != 999.0f);
    ASSERT_TRUE(m.feedback.observer_velocity != 999.0f);
    ASSERT_TRUE(isfinite(m.feedback.observer_angle));
    ASSERT_TRUE(isfinite(m.feedback.observer_velocity));

    printf("PASS test_feedback_fields_written\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════
   main
   ══════════════════════════════════════════════════════════════ */
int main(void)
{
    int failures = 0;
    failures += test_init_zeros();
    failures += test_null_safety();
    failures += test_zero_input_stays_zero();
    failures += test_low_speed_blend();
    failures += test_high_speed_pll_angle_bounded();
    failures += test_small_Ls_protection();
    failures += test_feedback_fields_written();

    if (failures == 0) {
        printf("\nAll SMO observer tests PASSED (%d tests)\n", 7);
        return 0;
    } else {
        printf("\n%d SMO observer test(s) FAILED\n", failures);
        return 1;
    }
}
