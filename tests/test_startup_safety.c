/* These tests execute the production FSM, protection, startup calibration,
 * ADC startup and public PWM using six simulated output channels. Only peripheral I/O and logging are substituted.
 * They do not establish real-board gate levels or ADC/PWM timing. */
#include "startup_test_support.h"
#include "current_calibration.h"
#include "safety_manager.h"
#include "pwm_interface.h"
#include "board_adc.h"
#include "motor_sampling.h"
#include "foc/cogging_compensation.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
static bool permit_transition;
static bool precheck(MotorState state) {
  (void)state;
  return permit_transition && Safety_CanEnable(&motor_data);
}
static void reset(void) {
  CoggingComp_StopCalibration(&motor_data);
  TestHardware_Reset();
  permit_transition=true;
  motor_data.algo_input.Vbus=48.0f;
  motor_data.feedback.temperature=25.0f;
  Detection_Init(NULL);
  Safety_Init(NULL);
  CurrentCalib_BeginStartup(&motor_data,false);
  StateMachine_Init(&g_ds402_state_machine);
  StateMachine_SetPreCheckCallback(&g_ds402_state_machine,precheck);
  StateMachine_Update(&g_ds402_state_machine);
}
static bool start(void) {
  bool ok=adc_bsp_init()==0;
  if (ok) ok=MHAL_PWM_StartSampling()==0;
  CurrentCalib_BeginStartup(&motor_data,ok);
  return ok;
}
static void frames(unsigned count) {
  for (unsigned i=0;i<count;++i) {
    ++test_sample_count;
    CurrentCalib_UpdateStartup(&motor_data);
    Safety_Update_Fast(&motor_data,&g_ds402_state_machine);
    if (test_sample_count%100==0) Safety_Update_Slow(&motor_data,&g_ds402_state_machine);
    if (test_sample_count%20==0) StateMachine_Update(&g_ds402_state_machine);
    Motor_API_Cogging_Update(&motor_data);
    Motor_RunControlCycle(&motor_data);
  }
}
static void ready(void) { assert(start()); frames(CURRENT_CALIB_CYCLES); assert(CurrentCalib_IsReady()); }
static void settle(void) {
  for (int i=0;i<6;++i) {
    StateMachine_Update(&g_ds402_state_machine);
    Motor_API_Cogging_Update(&motor_data);
    Motor_RunControlCycle(&motor_data);
  }
}
static void enable(void) {
  assert(StateMachine_RequestState(&g_ds402_state_machine,STATE_OPERATION_ENABLED));
  settle();
  assert(StateMachine_GetState(&g_ds402_state_machine)==STATE_OPERATION_ENABLED);
  assert(test_power_on);
}
static void test_passive_startup(void) {
  reset(); assert(start());
  assert(!StateMachine_RequestState(&g_ds402_state_machine,STATE_OPERATION_ENABLED));
  StateMachine_SetControlword(&g_ds402_state_machine,0x000f);
  assert(MHAL_PWM_Enable()!=0 && MHAL_PWM_Brake()!=0);
  frames(CURRENT_CALIB_CYCLES-1);
  assert(!CurrentCalib_IsReady() && !test_power_on);
  frames(1);
  assert(CurrentCalib_IsReady());
  frames(200);
  assert(test_sampling_on && !test_power_on);
  assert(test_enable_calls==0 && TestHardware_PhaseMask()==0 && test_control_calls==0);
  assert(g_ds402_state_machine.current_state==STATE_SWITCH_ON_DISABLED);
  assert(startup_test_offsets[0]==2048.0f && startup_test_offsets[1]==2040.0f);
  hadc1.Instance->JDR1=1000;
  frames(CURRENT_CALIB_CYCLES);
  assert(startup_test_offsets[0]==2048.0f); /* startup must not repeat */
  enable();
}
static void test_invalid_samples_and_init_failures(void) {
  for (unsigned fail=1;fail<=4;++fail) {
    reset(); test_adc_fail_call=fail;
    assert(!start());
    frames(CURRENT_CALIB_CYCLES+20);
    assert(!CurrentCalib_IsReady() && !test_power_on && !test_sampling_on);
    assert(!StateMachine_RequestState(&g_ds402_state_machine,STATE_OPERATION_ENABLED));
  }
  reset(); test_sampling_result=-1; assert(!start());
  frames(CURRENT_CALIB_CYCLES); assert(!CurrentCalib_IsReady());
  reset(); assert(start()); hadc1.Instance->JDR1=0;
  frames(CURRENT_CALIB_CYCLES); assert(!CurrentCalib_IsReady());
  hadc1.Instance->JDR1=2048; frames(CURRENT_CALIB_CYCLES);
  assert(CurrentCalib_IsReady() && !test_power_on);
}
static void test_fault_clear_requires_new_command(void) {
  reset(); ready(); enable();
  motor_data.state.Control_Mode=CONTROL_MODE_OPEN;
  motor_data.algo_input.Ia=FAULT_OVER_CURRENT_A+1;
  Safety_Update_Fast(&motor_data,&g_ds402_state_machine);
  assert(!test_power_on && Safety_HasActiveFault()); settle();
  assert(g_ds402_state_machine.current_state==STATE_FAULT);
  assert(!Safety_ClearFaults(&g_ds402_state_machine));
  motor_data.algo_input.Ia=0;
  Motor_ClearFaults(&motor_data);
  assert(g_ds402_state_machine.current_state==STATE_SWITCH_ON_DISABLED);
  frames(100);
  assert(!Safety_HasActiveFault() && !test_power_on);
  assert(!g_ds402_state_machine.auto_advance && g_ds402_state_machine.controlword.word==0);
  enable();
  StateMachine_EnterFault(&g_ds402_state_machine,FAULT_DRIVER_CHIP); settle();
  StateMachine_SetControlword(&g_ds402_state_machine,0x0080); settle();
  assert(!test_power_on && g_ds402_state_machine.current_state==STATE_SWITCH_ON_DISABLED);
  frames(100); assert(!test_power_on); enable();
}
static void test_failed_enable_and_unconditional_stop(void) {
  reset(); ready(); test_enable_result=-1;
  assert(StateMachine_RequestState(&g_ds402_state_machine,STATE_OPERATION_ENABLED)); settle();
  assert(!test_power_on && g_ds402_state_machine.current_state==STATE_FAULT);
  assert(g_ds402_state_machine.active_fault_code&FAULT_DRIVER_CHIP);
  assert(g_ds402_state_machine.controlword.word==0 && !g_ds402_state_machine.auto_advance);
  test_enable_result=0; assert(Safety_ClearFaults(&g_ds402_state_machine));
  frames(100); assert(!test_power_on); enable();
  permit_transition=false;
  assert(StateMachine_RequestState(&g_ds402_state_machine,STATE_SWITCH_ON_DISABLED));
  assert(!test_power_on);
  StateMachine_EnterFault(&g_ds402_state_machine,FAULT_DRIVER_CHIP); settle();
  assert(g_ds402_state_machine.current_state==STATE_FAULT);
  assert(Safety_ClearFaults(&g_ds402_state_machine));
  assert(!StateMachine_RequestState(&g_ds402_state_machine,STATE_OPERATION_ENABLED));
  permit_transition=true; frames(100); assert(!test_power_on); enable();
}
static void test_quick_stop_retains_active_brake(void) {
  reset(); ready(); enable();
  StateMachine_SetControlword(&g_ds402_state_machine,0x0002);
  StateMachine_Update(&g_ds402_state_machine);
  Motor_RunControlCycle(&motor_data);
  assert(g_ds402_state_machine.current_state==STATE_QUICK_STOP_ACTIVE);
  assert(test_power_on && TestHardware_PhaseMask()==0x3f);
  assert(htim1.compare[0]==0 && htim1.compare[1]==0 && htim1.compare[2]==0);
  motor_data.algo_input.Ia=FAULT_OVER_CURRENT_A+1;
  Safety_Update_Fast(&motor_data,&g_ds402_state_machine);
  Motor_RunControlCycle(&motor_data);
  assert(!test_power_on);
  assert(MHAL_PWM_Brake()!=0);
  settle();
  motor_data.algo_input.Ia=0;
  assert(Safety_ClearFaults(&g_ds402_state_machine));
  assert(MHAL_PWM_Brake()!=0 && !test_power_on);
}
static void test_explicit_cogging_survives_intermediate_states(void) {
  reset(); ready();
  Motor_API_StartCoggingCalib(&motor_data);
  assert(motor_data.advanced.cogging_calib_request>0.5f);
  StateMachine_Update(&g_ds402_state_machine); /* READY */
  Motor_API_Cogging_Update(&motor_data); Motor_RunControlCycle(&motor_data);
  assert(motor_data.advanced.cogging_calib_request>0.5f);
  StateMachine_Update(&g_ds402_state_machine); /* SWITCHED_ON */
  Motor_API_Cogging_Update(&motor_data); Motor_RunControlCycle(&motor_data);
  assert(motor_data.advanced.cogging_calib_request>0.5f);
  StateMachine_Update(&g_ds402_state_machine); /* OPERATION_ENABLED */
  Motor_API_Cogging_Update(&motor_data);
  assert(!CoggingComp_IsCalibrating()); /* State_Mode changes later in the ISR. */
  Motor_RunControlCycle(&motor_data);
  Motor_API_Cogging_Update(&motor_data);
  assert(CoggingComp_IsCalibrating() && test_power_on);
  StateMachine_RequestState(&g_ds402_state_machine,STATE_SWITCH_ON_DISABLED);
  Motor_RunControlCycle(&motor_data);
  assert(!CoggingComp_IsCalibrating() && !test_power_on);
}
static void test_open_loop_protection_and_irq_restore(void) {
  reset(); assert(start());
  motor_data.state.Control_Mode=CONTROL_MODE_OPEN;
  motor_data.algo_input.Ia=10000;
  Safety_Update_Fast(&motor_data,&g_ds402_state_machine);
  assert(!CurrentCalib_IsReady() && !Safety_HasActiveFault());
  motor_data.algo_input.Vbus=1000;
  motor_data.feedback.temperature=1000;
  Safety_Update_Slow(&motor_data,&g_ds402_state_machine);
  assert(Safety_GetActiveFaultBits()&FAULT_OVER_VOLTAGE);
  assert(Safety_GetActiveFaultBits()&FAULT_OVER_TEMP);
  assert(!(Safety_GetActiveFaultBits()&FAULT_OVER_CURRENT));
  reset(); ready(); motor_data.state.Control_Mode=CONTROL_MODE_OPEN;
  motor_data.algo_input.Vbus=1000;
  for(int i=0;i<10;++i) Safety_Update_Slow(&motor_data,&g_ds402_state_machine);
  assert(Safety_GetActiveFaultBits()&FAULT_OVER_VOLTAGE);
  reset(); ready(); motor_data.state.Control_Mode=CONTROL_MODE_OPEN;
  motor_data.feedback.temperature=1000;
  for(int i=0;i<30;++i) Safety_Update_Slow(&motor_data,&g_ds402_state_machine);
  assert(Safety_GetActiveFaultBits()&FAULT_OVER_TEMP);
  test_irq_mask=1; (void)Safety_GetActiveFaultBits(); assert(test_irq_mask==1);
  test_irq_mask=0; (void)Safety_GetActiveFaultBits(); assert(test_irq_mask==0);
  reset(); StateMachine_SetPreCheckCallback(&g_ds402_state_machine,NULL); ready();
  assert(!StateMachine_RequestState(&g_ds402_state_machine,STATE_OPERATION_ENABLED));
}
static void assert_detection_unchanged(const DetectionState *before) {
  const DetectionState *after = Detection_GetState();
  assert(after->vbus_filtered == before->vbus_filtered);
  assert(after->temp_filtered == before->temp_filtered);
  assert(after->voltage_filter_initialized == before->voltage_filter_initialized);
  assert(after->temp_filter_initialized == before->temp_filter_initialized);
  assert(after->current_peak == before->current_peak);
  assert(after->stall_counter == before->stall_counter);
  assert(after->is_stall == before->is_stall);
  assert(after->encoder_err_consecutive == before->encoder_err_consecutive);
  assert(after->encoder_err_count == before->encoder_err_count);
  assert(after->last_can_time == before->last_can_time);
  assert(after->is_can_timeout == before->is_can_timeout);
}
static void test_enable_preview_preserves_periodic_detection_state(void) {
  reset(); ready(); enable();
  MT6816_Handle_t encoder = {0};
  encoder.last_status = MT6816_ERR_SPI;
  motor_data.components.encoder = &encoder;
  motor_data.state.Control_Mode = CONTROL_MODE_TORQUE;
  motor_data.algo_input.Vbus = 50.0f;
  motor_data.feedback.temperature = 30.0f;
  motor_data.algo_input.Ia = motor_data.algo_input.Ib = motor_data.algo_input.Ic =
      FAULT_STALL_CURRENT_A + 1.0f;
  Safety_Update_Slow(&motor_data, &g_ds402_state_machine);
  DetectionState before = *Detection_GetState();
  assert(before.stall_counter == 1 && before.encoder_err_consecutive == 1);
  for (unsigned i = 0; i < 100; ++i) {
    assert(StateMachine_RequestState(&g_ds402_state_machine, STATE_OPERATION_ENABLED));
    StateMachine_SetControlword(&g_ds402_state_machine, 0x000f);
    StateMachine_Update(&g_ds402_state_machine);
    assert(test_power_on && !Safety_HasActiveFault());
  }
  assert_detection_unchanged(&before);
  /* A real periodic update still advances each filter/counter exactly once. */
  Safety_Update_Slow(&motor_data, &g_ds402_state_machine);
  const DetectionState *after = Detection_GetState();
  assert(after->stall_counter == before.stall_counter + 1);
  assert(after->encoder_err_consecutive == before.encoder_err_consecutive + 1);
  assert(after->vbus_filtered > before.vbus_filtered);
  assert(after->temp_filtered > before.temp_filtered);
  before = *after;
  motor_data.algo_input.Ia = FAULT_OVER_CURRENT_A + 1;
  test_irq_mask = 1;
  assert(!Safety_CanEnable(&motor_data));
  assert(test_irq_mask == 1);
  assert_detection_unchanged(&before);
  test_irq_mask = 0;
  motor_data.components.encoder = NULL;
}
static void test_public_pwm_channels_and_failures(void) {
  reset(); ready();
  assert(test_sampling_on && TestHardware_PhaseMask()==0);
  unsigned stops = test_stop_calls;
  assert(MHAL_PWM_StartSampling()==0 && test_stop_calls==stops);
  assert(MHAL_PWM_SetDuty(0.25f,0.5f,0.75f)==0);
  assert(htim1.compare[0]==250 && htim1.compare[1]==500 && htim1.compare[2]==750);
  enable();
  assert(TestHardware_PhaseMask()==0x3f && test_enable_calls==6);
  assert(MHAL_PWM_Enable()==0 && test_enable_calls==6); /* already running */
  assert(MHAL_PWM_Disable()==0 && TestHardware_PhaseMask()==0 && test_sampling_on);
  assert(MHAL_PWM_Brake()==0 && TestHardware_PhaseMask()==0x3f);
  assert(htim1.compare[0]==0 && htim1.compare[1]==0 && htim1.compare[2]==0);
  assert(MHAL_PWM_Disable()==0);
  htim1.main_state[TIM_CHANNEL_4]=HAL_TIM_CHANNEL_STATE_READY;
  test_sampling_on=false;
  assert(MHAL_PWM_Enable()!=0 && !test_power_on);
  for (unsigned failure=1; failure<=6; ++failure) {
    reset(); ready(); test_enable_fail_call=failure;
    assert(StateMachine_RequestState(&g_ds402_state_machine,STATE_OPERATION_ENABLED));
    settle();
    assert(g_ds402_state_machine.current_state==STATE_FAULT);
    assert(test_enable_calls==failure && TestHardware_PhaseMask()==0 && test_sampling_on);
  }
  reset(); ready(); enable();
  test_stop_fail_call=test_stop_calls+1;
  assert(MHAL_PWM_Disable()!=0); /* caller sees a real peripheral failure */
  assert(test_stop_calls==test_stop_fail_call+5 && test_sampling_on);
  test_stop_fail_call=0;
  assert(MHAL_PWM_Disable()==0 && TestHardware_PhaseMask()==0 && test_sampling_on);
  reset(); test_stop_fail_call=test_stop_calls+1;
  assert(!start() && !test_sampling_on); /* cannot arm after an incomplete power-off */
}
int main(void) {
  test_passive_startup();
  test_invalid_samples_and_init_failures();
  test_fault_clear_requires_new_command();
  test_failed_enable_and_unconditional_stop();
  test_open_loop_protection_and_irq_restore();
  test_quick_stop_retains_active_brake();
  test_explicit_cogging_survives_intermediate_states();
  test_enable_preview_preserves_periodic_detection_state();
  test_public_pwm_channels_and_failures();
  puts("startup safety: 9 production-chain test groups passed (including six-channel PWM)");
  return 0;
}
