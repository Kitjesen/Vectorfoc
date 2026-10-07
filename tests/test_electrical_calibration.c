/* Run the production R/L coordinator and every electrical calibration cycle.
 * Voltage injection, braking, gain refresh and encoder identification are
 * observed at their boundaries; no power stage is driven by this host test. */
#include "resistance_inductance_calibration.h"
#include "encoder_calibration.h"
#ifdef CONTROL_TEST_LEGACY_LAYOUT
#include "config.h"
#include "control/control.h"
#else
#include "motor_configuration.h"
#include "foc/control_dispatcher.h"
#endif
#include "pwm_interface.h"
#include "mt6816_encoder.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static bool trace_enabled;
static unsigned injections, brakes, gain_updates, events, encoder_steps;
static unsigned last_injection, last_brake, last_gain;
static float voltage_d, voltage_q, voltage_angle, gain_rs, gain_ls;
static unsigned sensor_prepare_calls;

const PositionSensorDescriptor_t *PositionSensor_GetDescriptor(void) {
  static const PositionSensorDescriptor_t descriptor = {
      .name = "electrical-calibration-test",
      .capabilities = POSITION_SENSOR_CAP_RAW_DIRECTION_POLE,
  };
  return &descriptor;
}
PositionSensorStatus_t PositionSensor_RawCalibrationPrepareClockwise(void) {
  ++sensor_prepare_calls;
  return POSITION_SENSOR_STATUS_OK;
}

void Control_InjectVoltage(MOTOR_DATA *motor, float vd, float vq, float angle) {
  CHECK(motor);
  ++injections; last_injection = ++events;
  voltage_d = vd; voltage_q = vq; voltage_angle = angle;
}
int MHAL_PWM_Brake(void) { ++brakes; last_brake = ++events; return 0; }
void Control_UpdateCurrentGains(MOTOR_DATA *motor) {
  CHECK(motor);
  ++gain_updates; last_gain = ++events;
  gain_rs = motor->parameters.Rs; gain_ls = motor->parameters.Ls;
}
CalibResult DirectionPoleCalib_Update(MOTOR_DATA *motor, DirectionPoleCalibContext *ctx) {
  (void)motor; (void)ctx; ++encoder_steps; return CALIB_IN_PROGRESS;
}
CalibResult EncoderCalib_Update(MOTOR_DATA *motor, EncoderCalibContext *ctx) {
  (void)motor; (void)ctx; ++encoder_steps; return CALIB_IN_PROGRESS;
}

static void TraceFloat(const char *name, float value) {
  CHECK(isfinite(value));
  if (trace_enabled) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    printf(" %s=%08" PRIx32, name, bits);
  }
}
#define F(field) TraceFloat(#field, (field))

static unsigned RunScenario(unsigned scenario) {
  MOTOR_DATA motor = {0};
  CalibrationContext ctx = {0};
  int error_workspace[SAMPLES_PER_POLE_PAIR * MAX_POLE_PAIRS];
  int16_t lut_workspace[POSITION_SENSOR_CALIBRATION_LUT_SIZE];
  ctx.encoder.error_array = error_workspace;
  ctx.encoder.error_array_size = sizeof(error_workspace) / sizeof(error_workspace[0]);
  ctx.encoder.offset_lut = lut_workspace;
  ctx.encoder.offset_lut_size = sizeof(lut_workspace) / sizeof(lut_workspace[0]);
  MT6816_Handle_t encoder = {0};
  motor.components.encoder = &encoder;
  injections = brakes = gain_updates = events = encoder_steps = 0;
  sensor_prepare_calls = 0;
  last_injection = last_brake = last_gain = 0;
  voltage_d = voltage_q = voltage_angle = gain_rs = gain_ls = 0.0f;
  float dt = scenario == 2 ? 0.0001f : CURRENT_SAMPLE_PERIOD_S;

  CHECK(RSLSCalib_Start(NULL, &ctx) == CALIB_FAILED_INVALID_PARAMS);
  CHECK(RSLSCalib_Start(&motor, NULL) == CALIB_FAILED_INVALID_PARAMS);
  CHECK(RSLSCalib_Update(NULL, &ctx, dt) == CALIB_FAILED_INVALID_PARAMS);
  CHECK(RSLSCalib_Update(&motor, NULL, dt) == CALIB_FAILED_INVALID_PARAMS);
  CHECK(RSLSCalib_GetProgress(NULL) == 0);

  if (scenario == 2) {
    /* Exercise the step-local defaults with a zero-initialized context. */
    motor.state.Cs_State = CS_MOTOR_R_START;
  } else {
    CHECK(RSLSCalib_Start(&motor, &ctx) == CALIB_IN_PROGRESS);
    CHECK(motor.state.Cs_State == CS_MOTOR_R_START);
    CHECK(ctx.encoder.error_array == error_workspace);
    CHECK(ctx.encoder.error_array_size == sizeof(error_workspace) / sizeof(error_workspace[0]));
    CHECK(ctx.encoder.offset_lut == lut_workspace);
    CHECK(ctx.encoder.offset_lut_size == sizeof(lut_workspace) / sizeof(lut_workspace[0]));
    CHECK(sensor_prepare_calls == 1);
  }

  unsigned cycles = 0;
  while (motor.state.Cs_State != CS_DIR_PP_START) {
    CHECK(cycles < RS_CALIB_CYCLES + 2u * LS_CALIB_CYCLES + 8u);
    CS_STATE before = motor.state.Cs_State;
    if (before <= CS_MOTOR_R_END) {
      motor.algo_input.Ia = 3.0f + (float)(cycles % 7u) * 0.03125f;
    } else if (scenario == 1) {
      motor.algo_input.Ia = 0.0f; /* Zero dI takes the specified L fallback. */
    } else {
      motor.algo_input.Ia = (ctx.inductance.loop_count & 1u) ? 1.25f : -0.75f;
    }
    CalibResult result = RSLSCalib_Update(&motor, &ctx, dt);
    CHECK(result == CALIB_IN_PROGRESS); /* R/L completion continues to encoder. */
    CHECK(encoder_steps == 0);
    CHECK(voltage_q == 0.0f && voltage_angle == 0.0f);
    if (before == CS_MOTOR_R_START) CHECK(motor.state.Cs_State == CS_MOTOR_R_LOOP);
    if (before == CS_MOTOR_R_END) CHECK(motor.state.Cs_State == CS_MOTOR_L_START);
    if (before == CS_MOTOR_L_START) CHECK(motor.state.Cs_State == CS_MOTOR_L_LOOP);
    if (before == CS_MOTOR_L_END) CHECK(motor.state.Cs_State == CS_DIR_PP_START);
    if (trace_enabled) printf("scenario=%u cycle=%u before=%u after=%u result=%u Rcount=%u Lcount=%u progress=%u injections=%u brakes=%u gains=%u events=%u last=%u,%u,%u",
        scenario, cycles, before, motor.state.Cs_State, result,
        (unsigned)ctx.resistance.loop_count, (unsigned)ctx.inductance.loop_count,
        RSLSCalib_GetProgress(&ctx), injections, brakes, gain_updates, events,
        last_injection, last_brake, last_gain);
    F(dt); F(motor.algo_input.Ia); F(motor.parameters.Rs); F(motor.parameters.Ls);
    F(ctx.resistance.voltage); F(ctx.resistance.kI);
    F(ctx.inductance.Ialphas[0]); F(ctx.inductance.Ialphas[1]);
    F(ctx.inductance.voltages[0]); F(ctx.inductance.voltages[1]);
    F(voltage_d); F(voltage_q); F(voltage_angle); F(gain_rs); F(gain_ls);
    if (trace_enabled) putchar('\n');
    ++cycles;
  }
  CHECK(cycles == RS_CALIB_CYCLES + 2u * LS_CALIB_CYCLES + 4u);
  CHECK(injections == RS_CALIB_CYCLES + 2u * LS_CALIB_CYCLES + 1u);
  CHECK(brakes == 2 && gain_updates == 1);
  CHECK(last_gain > last_brake && last_brake > last_injection);
  CHECK(motor.parameters.Rs > 0.0f && motor.parameters.Ls > 0.0f);
  CHECK(gain_rs == motor.parameters.Rs && gain_ls == motor.parameters.Ls);
  if (scenario == 1) CHECK(motor.parameters.Ls == 0.00003f);
  else CHECK(motor.parameters.Ls != 0.00003f);
  CHECK(ctx.resistance.kI == 2.0f);
  CHECK(ctx.inductance.voltages[0] == -VOLTAGE_MAX_CALIB);
  CHECK(ctx.inductance.voltages[1] == VOLTAGE_MAX_CALIB);
  motor.state.Cs_State = CS_STATE_IDLE;
  CHECK(RSLSCalib_Update(&motor, &ctx, dt) == CALIB_FAILED_INVALID_PARAMS);
  CHECK(encoder_steps == 0);
  return cycles;
}

int main(int argc, char **argv) {
  trace_enabled = argc == 2 && strcmp(argv[1], "--trace") == 0;
  CHECK(argc == 1 || trace_enabled);
  unsigned cycles = 0;
  for (unsigned scenario = 0; scenario < 3; ++scenario) cycles += RunScenario(scenario);
  printf("PASS electrical_calibration scenarios=3 cycles=%u R_L_state_machine=real voltage_brake_gain=observed\n", cycles);
  return 0;
}
