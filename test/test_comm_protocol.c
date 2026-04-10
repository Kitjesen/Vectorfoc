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
 * @file test_comm_protocol.c
 * @brief Vector CAN 协议帧编解码单元测试
 *
 * 测试策略：
 *  - 直接调用 ProtocolVector_BuildFeedback / BuildFault /
 *    BuildParamResponse / BuildCalibStatus / BuildCalibValidate
 *  - ProtocolVector_Parse：MIT 控制帧、使能/停止、参数读写、标定、无效帧
 *  - 所有测试均在宿主机运行，通过 stub 隔离 Motor_RequestCalibration 等依赖
 *
 * 依赖 stub（见文件末尾）：
 *  - g_can_id, motor_data, Motor_RequestCalibration, Motor_AbortCalibration,
 *    Motor_PreCalibCheck, Motor_ClearFaults, Param_WriteFloat, Param_ScheduleSave,
 *    CmdService_SetReportEnable, Protocol_SendFrame, CalibContext_GetProgress,
 *    Safety_GetLastFaultTime, HAL_NVIC_SystemReset
 */

#include "protocol_types.h"
#include "motor.h"         /* CONTROL_MODE_MIT, MOTOR_DATA */
#include "vector_protocol.h"
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

/* ── 构造 CAN 帧辅助 ─────────────────────────────────────────── */
static CAN_Frame make_frame(uint32_t id, bool extended,
                             uint8_t dlc, const uint8_t *data)
{
    CAN_Frame f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.is_extended = extended;
    f.dlc = dlc;
    if (data && dlc > 0) memcpy(f.data, data, dlc);
    return f;
}

/* ── U16 → float 辅助（与 vector_protocol.c 中相同的映射）───── */
static float u16_to_float(uint16_t x, float min_v, float max_v)
{
    return ((float)x / 65535.0f) * (max_v - min_v) + min_v;
}

static float buf_to_float(const uint8_t *b, float min_v, float max_v)
{
    uint16_t raw = (uint16_t)((b[0] << 8) | b[1]);
    return u16_to_float(raw, min_v, max_v);
}

/* ════════════════════════════════════════════════════════════════
   BuildFeedback
   ════════════════════════════════════════════════════════════════ */
static int test_build_feedback_basic(void)
{
    MotorStatus s;
    memset(&s, 0, sizeof(s));
    s.can_id      = 0x05;
    s.position    = 1.0f;
    s.velocity    = 2.0f;
    s.torque      = 3.0f;
    s.temperature = 25.0f;
    s.calib_stage = 0; /* not calibrating → mode=2 */

    CAN_Frame f;
    CHECK(ProtocolVector_BuildFeedback(&s, &f) == true);
    CHECK(f.is_extended);
    CHECK(f.dlc == 8);

    /* CMD = 0x02 at bits [28:24] */
    uint8_t cmd = (uint8_t)((f.id >> 24) & 0x1F);
    CHECK(cmd == 0x02);

    /* 位置解码 */
    float pos_dec = buf_to_float(&f.data[0], -12.57f, 12.57f);
    CHECK_NEAR(pos_dec, 1.0f, 0.05f);

    /* 速度解码 */
    float vel_dec = buf_to_float(&f.data[2], -15.0f, 15.0f);
    CHECK_NEAR(vel_dec, 2.0f, 0.05f);

    /* NULL 安全 */
    CHECK(ProtocolVector_BuildFeedback(NULL, &f) == false);
    CHECK(ProtocolVector_BuildFeedback(&s, NULL) == false);

    printf("PASS test_build_feedback_basic\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   BuildParamResponse
   ════════════════════════════════════════════════════════════════ */
static int test_build_param_response(void)
{
    CAN_Frame f;
    CHECK(ProtocolVector_BuildParamResponse(0x2000, 1.23f, &f) == true);
    CHECK(f.dlc == 8);

    /* 参数索引 (LE) */
    uint16_t idx = (uint16_t)(f.data[0] | (f.data[1] << 8));
    CHECK(idx == 0x2000);

    /* 参数值 (4 bytes float) */
    float val;
    memcpy(&val, &f.data[4], 4);
    CHECK_NEAR(val, 1.23f, 1e-5f);

    CHECK(ProtocolVector_BuildParamResponse(0, 0.0f, NULL) == false);

    printf("PASS test_build_param_response\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   BuildFault
   ════════════════════════════════════════════════════════════════ */
static int test_build_fault(void)
{
    CAN_Frame f;
    CHECK(ProtocolVector_BuildFault(0xDEADBEEF, 0, &f) == true);
    CHECK(f.dlc == 8);

    /* CMD = 0x15 */
    CHECK(((f.id >> 24) & 0x1F) == 0x15u);

    CHECK(ProtocolVector_BuildFault(0, 0, NULL) == false);

    printf("PASS test_build_fault\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   BuildCalibStatus
   ════════════════════════════════════════════════════════════════ */
static int test_build_calib_status(void)
{
    MotorStatus s;
    memset(&s, 0, sizeof(s));
    s.can_id         = 1;
    s.calib_stage    = 2;
    s.calib_sub_stage= 5;
    s.calib_progress = 42;
    s.calib_result   = 0;

    CAN_Frame f;
    CHECK(ProtocolVector_BuildCalibStatus(&s, &f) == true);
    CHECK(f.dlc == 8);
    CHECK(((f.id >> 24) & 0xFF) == 0x09u);
    CHECK(f.data[0] == 2);   /* calib_stage */
    CHECK(f.data[1] == 5);   /* calib_sub_stage */
    CHECK(f.data[2] == 42);  /* progress */

    CHECK(ProtocolVector_BuildCalibStatus(NULL, &f) == false);
    CHECK(ProtocolVector_BuildCalibStatus(&s, NULL) == false);

    printf("PASS test_build_calib_status\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   BuildCalibValidate
   ════════════════════════════════════════════════════════════════ */
static int test_build_calib_validate(void)
{
    CAN_Frame f;
    CHECK(ProtocolVector_BuildCalibValidate(0x0F, 0x00, 24.0f, 30.0f, &f) == true);
    CHECK(f.dlc == 8);
    CHECK(((f.id >> 24) & 0xFF) == 0x0Eu);
    CHECK(f.data[0] == 0x0F);  /* pass_mask */
    CHECK(f.data[1] == 0x00);  /* fail_mask */

    /* Vbus×100 = 2400 */
    uint16_t vbus_raw = (uint16_t)((f.data[2] << 8) | f.data[3]);
    CHECK_NEAR((float)vbus_raw, 2400.0f, 1.0f);

    CHECK(ProtocolVector_BuildCalibValidate(0, 0, 0, 0, NULL) == false);

    printf("PASS test_build_calib_validate\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   Parse — 非扩展帧应拒绝
   ════════════════════════════════════════════════════════════════ */
static int test_parse_rejects_standard_frame(void)
{
    uint8_t d[8] = {0};
    CAN_Frame f = make_frame(0x100, false, 8, d);
    MotorCommand cmd;
    ParseResult r = ProtocolVector_Parse(&f, &cmd);
    CHECK(r == PARSE_ERR_INVALID_FRAME);

    printf("PASS test_parse_rejects_standard_frame\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   Parse — NULL 安全
   ════════════════════════════════════════════════════════════════ */
static int test_parse_null_safe(void)
{
    CAN_Frame f;
    memset(&f, 0, sizeof(f));
    f.is_extended = true;
    MotorCommand cmd;

    ParseResult r1 = ProtocolVector_Parse(NULL, &cmd);
    CHECK(r1 == PARSE_ERR_INVALID_FRAME);

    ParseResult r2 = ProtocolVector_Parse(&f, NULL);
    CHECK(r2 == PARSE_ERR_INVALID_FRAME);

    printf("PASS test_parse_null_safe\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   Parse — MOTOR_CTRL (MIT impedance, CMD=1)
   ID: [CMD5b=1][Torque16b][TargetID8b]
   data: [pos_hi][pos_lo][vel_hi][vel_lo][kp_hi][kp_lo][kd_hi][kd_lo]
   ════════════════════════════════════════════════════════════════ */
static int test_parse_motor_ctrl(void)
{
    /* torque_raw = 32767 → ~0 Nm (midpoint of ±120 range) */
    uint16_t tor = 32767;
    uint32_t id  = (1u << 24) | ((uint32_t)tor << 8) | 0x01u;

    /* pos = 0 rad (raw=0), vel = 0, kp=100, kd=1 */
    /* pos_raw 32767 → ~0 rad (midpoint of ±12.57 range) */
    uint16_t pos_raw = 32767;
    uint16_t vel_raw = 32767;
    uint16_t kp_raw  = (uint16_t)(100.0f / 500.0f * 65535.0f);
    uint16_t kd_raw  = (uint16_t)(1.0f  / 100.0f * 65535.0f);

    uint8_t data[8] = {
        (uint8_t)(pos_raw >> 8), (uint8_t)(pos_raw),
        (uint8_t)(vel_raw >> 8), (uint8_t)(vel_raw),
        (uint8_t)(kp_raw >> 8),  (uint8_t)(kp_raw),
        (uint8_t)(kd_raw >> 8),  (uint8_t)(kd_raw),
    };
    CAN_Frame f = make_frame(id, true, 8, data);
    MotorCommand cmd;
    ParseResult r = ProtocolVector_Parse(&f, &cmd);
    CHECK(r == PARSE_OK);
    CHECK(cmd.control_mode == CONTROL_MODE_MIT);
    CHECK(cmd.enable_motor == true);
    CHECK_NEAR(cmd.kp, 100.0f, 1.0f);
    CHECK_NEAR(cmd.kd,   1.0f, 0.1f);

    printf("PASS test_parse_motor_ctrl\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   Parse — ENABLE / STOP (CMD 3 / 4)
   ════════════════════════════════════════════════════════════════ */
static int test_parse_enable_stop(void)
{
    /* Enable: CMD=3 */
    uint32_t id_en = (3u << 24) | 0x01u;
    CAN_Frame f_en = make_frame(id_en, true, 0, NULL);
    MotorCommand cmd;
    CHECK(ProtocolVector_Parse(&f_en, &cmd) == PARSE_OK);
    CHECK(cmd.enable_motor == true);

    /* Stop: CMD=4 */
    uint32_t id_st = (4u << 24) | 0x01u;
    CAN_Frame f_st = make_frame(id_st, true, 0, NULL);
    CHECK(ProtocolVector_Parse(&f_st, &cmd) == PARSE_OK);
    CHECK(cmd.enable_motor == false);

    printf("PASS test_parse_enable_stop\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   Parse — PARAM_READ (CMD=17)
   ════════════════════════════════════════════════════════════════ */
static int test_parse_param_read(void)
{
    uint32_t id = (17u << 24) | 0x01u;
    uint8_t data[4] = {0x00, 0x20, 0x00, 0x00}; /* index = 0x2000 LE */
    CAN_Frame f = make_frame(id, true, 4, data);
    MotorCommand cmd;
    CHECK(ProtocolVector_Parse(&f, &cmd) == PARSE_OK);
    CHECK(cmd.is_param_read == true);
    CHECK(cmd.param_index == 0x2000);

    printf("PASS test_parse_param_read\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   Parse — PARAM_WRITE (CMD=18)
   ════════════════════════════════════════════════════════════════ */
static int test_parse_param_write(void)
{
    uint32_t id = (18u << 24) | 0x01u;
    float val = 3.14f;
    uint8_t data[8] = {0x00, 0x20, 0x00, 0x00, 0,0,0,0};
    memcpy(&data[4], &val, 4);
    CAN_Frame f = make_frame(id, true, 8, data);
    MotorCommand cmd;
    CHECK(ProtocolVector_Parse(&f, &cmd) == PARSE_OK);
    CHECK(cmd.is_param_write == true);
    CHECK(cmd.param_index == 0x2000);

    printf("PASS test_parse_param_write\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   Parse — 未知 CMD 返回 UNSUPPORTED
   ════════════════════════════════════════════════════════════════ */
static int test_parse_unknown_cmd(void)
{
    uint32_t id = (0x1Fu << 24) | 0x01u; /* CMD=31，未定义 */
    CAN_Frame f = make_frame(id, true, 0, NULL);
    MotorCommand cmd;
    ParseResult r = ProtocolVector_Parse(&f, &cmd);
    CHECK(r == PARSE_ERR_UNSUPPORTED);

    printf("PASS test_parse_unknown_cmd\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   Parse — DLC 不足时返回 INVALID_FRAME
   ════════════════════════════════════════════════════════════════ */
static int test_parse_motor_ctrl_short_dlc(void)
{
    uint32_t id = (1u << 24) | 0x01u;
    uint8_t data[4] = {0};
    CAN_Frame f = make_frame(id, true, 4, data); /* dlc=4 < 8 */
    MotorCommand cmd;
    ParseResult r = ProtocolVector_Parse(&f, &cmd);
    CHECK(r == PARSE_ERR_INVALID_FRAME);

    printf("PASS test_parse_motor_ctrl_short_dlc\n");
    return 0;
}

/* ════════════════════════════════════════════════════════════════
   main
   ════════════════════════════════════════════════════════════════ */
int main(void)
{
    ProtocolVector_Init();

    int f = 0;
    f += test_build_feedback_basic();
    f += test_build_param_response();
    f += test_build_fault();
    f += test_build_calib_status();
    f += test_build_calib_validate();
    f += test_parse_rejects_standard_frame();
    f += test_parse_null_safe();
    f += test_parse_motor_ctrl();
    f += test_parse_enable_stop();
    f += test_parse_param_read();
    f += test_parse_param_write();
    f += test_parse_unknown_cmd();
    f += test_parse_motor_ctrl_short_dlc();

    if (f == 0) {
        printf("\nAll comm protocol tests PASSED (13 tests)\n");
        return 0;
    }
    printf("\n%d comm protocol test(s) FAILED\n", f);
    return 1;
}

/* ════════════════════════════════════════════════════════════════
   STUBS（链接 vector_protocol.c 所需的外部符号）
   ════════════════════════════════════════════════════════════════ */
#include "motor.h"
#include "calibration_context.h"

uint8_t g_can_id = 1;
uint8_t g_protocol_type = 0;
MOTOR_DATA motor_data;

void Motor_RequestCalibration(MOTOR_DATA *m, uint8_t type) { (void)m; (void)type; }
void Motor_AbortCalibration(MOTOR_DATA *m) { (void)m; }
void Motor_ClearFaults(MOTOR_DATA *m) { (void)m; }
uint8_t Motor_PreCalibCheck(MOTOR_DATA *m, uint8_t *fail) {
    (void)m; if (fail) *fail = 0; return 0xFF;
}
void Param_WriteFloat(uint16_t idx, float v) { (void)idx; (void)v; }
void Param_WriteUint8(uint16_t idx, uint8_t v) { (void)idx; (void)v; }
void Param_ScheduleSave(void) {}
void CmdService_SetReportEnable(bool en) { (void)en; }
void Protocol_SendFrame(const CAN_Frame *f) { (void)f; }
uint8_t CalibContext_GetProgress(uint8_t a, uint8_t b,
                                  const CalibrationContext *c) {
    (void)a; (void)b; (void)c; return 0;
}
uint32_t Safety_GetLastFaultTime(void) { return 0; }
/* HAL_NVIC_SystemReset is a macro in test stub — undefine before redefining as function */
#undef HAL_NVIC_SystemReset
void HAL_NVIC_SystemReset(void) {}

