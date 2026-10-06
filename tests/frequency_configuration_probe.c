#include "config/motor_configuration.h"

#if FOC_LOOP_FREQUENCY_HZ != PWM_FREQUENCY
#error "FOC loop frequency must follow the integer PWM frequency"
#endif

#if CURRENT_SAMPLE_FREQUENCY_HZ != FOC_LOOP_FREQUENCY_HZ
#error "current sampling must follow the FOC loop frequency"
#endif

#if COMMAND_RAMP_FREQUENCY_HZ != FOC_LOOP_FREQUENCY_HZ
#error "command ramp must follow the FOC loop frequency"
#endif

#if VELOCITY_LOOP_DECIMATION != (FOC_LOOP_FREQUENCY_HZ / VELOCITY_LOOP_FREQUENCY_HZ)
#error "velocity decimation does not match its configured frequency"
#endif

#if POSITION_LOOP_DECIMATION != (FOC_LOOP_FREQUENCY_HZ / POSITION_LOOP_FREQUENCY_HZ)
#error "position decimation does not match its configured frequency"
#endif

#if STATE_MACHINE_DECIMATION != (FOC_LOOP_FREQUENCY_HZ / STATE_MACHINE_UPDATE_FREQUENCY_HZ)
#error "state-machine decimation does not match its configured frequency"
#endif

#if TORQUE_COMPENSATION_UPDATE_DECIMATION != \
    (FOC_LOOP_FREQUENCY_HZ / TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ)
#error "torque-compensation decimation does not match its configured frequency"
#endif

#if SCOPE_LOG_DECIMATION != (FOC_LOOP_FREQUENCY_HZ / SCOPE_LOG_FREQUENCY_HZ)
#error "scope-log decimation does not match its configured frequency"
#endif

int main(void) { return 0; }
