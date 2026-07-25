#ifndef TEST_APP_INIT_BINDING_MOCKS_MOTOR_HAL_API_H
#define TEST_APP_INIT_BINDING_MOCKS_MOTOR_HAL_API_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { float i_a; float i_b; float i_c; float v_bus; float temp; } Motor_HAL_SensorData_t;
typedef struct { float position_rad; float angle_rad; float velocity_rad; float elec_angle; int32_t raw_value; } Motor_HAL_EncoderData_t;
typedef struct { void (*set_duty)(float,float,float); bool (*start_sampling)(void); bool (*enable)(void); void (*disable)(void); void (*brake)(void); } Motor_HAL_PwmInterface_t;
typedef struct { void (*update)(Motor_HAL_SensorData_t *); bool (*calibrate_offsets)(void); } Motor_HAL_AdcInterface_t;
typedef struct { bool (*update)(void); void (*get_data)(Motor_HAL_EncoderData_t *); void (*set_pole_pairs)(uint8_t); void (*zero_position)(void); void (*set_offset)(float); float (*get_offset)(void); } Motor_HAL_EncoderInterface_t;
typedef struct { const Motor_HAL_PwmInterface_t *pwm; const Motor_HAL_AdcInterface_t *adc; const Motor_HAL_EncoderInterface_t *encoder; } Motor_HAL_Handle_t;
#endif