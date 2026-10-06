#ifndef STARTUP_TEST_SUPPORT_H
#define STARTUP_TEST_SUPPORT_H
#include "motor_runtime.h"
extern unsigned test_enable_calls, test_sample_count, test_control_calls;
extern unsigned test_adc_calls, test_adc_fail_call;
extern float startup_test_offsets[3];
extern int test_enable_result, test_sampling_result;
extern bool test_power_on, test_sampling_on;
extern uint32_t test_irq_mask;
extern unsigned test_enable_fail_call, test_stop_calls, test_stop_fail_call;
unsigned TestHardware_PhaseMask(void);
void TestHardware_Reset(void);
#endif
