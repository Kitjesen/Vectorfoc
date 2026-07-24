cmake_minimum_required(VERSION 3.10)

project(VectorFOC_Test C)

set(CMAKE_C_STANDARD 99)
enable_testing()

# Define project root relative to test dir
set(PROJ_ROOT "${CMAKE_SOURCE_DIR}/..")

# For PC test environment, use mocks directory FIRST to override real headers
include_directories(
    ${CMAKE_SOURCE_DIR}/mocks  # Mock headers first!
)

# Only include algorithm / UI / COMM directories (no HAL/CMSIS)
include_directories(
    ${PROJ_ROOT}/Src/ALGO
    ${PROJ_ROOT}/Src/ALGO/foc
    ${PROJ_ROOT}/Src/ALGO/motor
    ${PROJ_ROOT}/Src/ALGO/pid
    ${PROJ_ROOT}/Src/ALGO/control
    ${PROJ_ROOT}/Src/ALGO/observer
    ${PROJ_ROOT}/Src/ALGO/trajectory
    ${PROJ_ROOT}/Src/UI
    ${PROJ_ROOT}/Src/UI/error
    ${PROJ_ROOT}/Src/UI/parameter
    ${PROJ_ROOT}/Src/UI/led
    ${PROJ_ROOT}/Src/COMM
    ${PROJ_ROOT}/Src/COMM/manager
    ${PROJ_ROOT}/Src/COMM/protocol
    ${PROJ_ROOT}/Src/COMM/protocol/vector
    ${PROJ_ROOT}/Src/COMM/transport
    ${PROJ_ROOT}/Src/COMM/executor
    ${PROJ_ROOT}/Src/APP/rtos
    ${PROJ_ROOT}/Src/APP
    ${PROJ_ROOT}/Src/config
    ${PROJ_ROOT}/Src/HAL/interface
    ${PROJ_ROOT}/Src/HAL/bsp
    ${PROJ_ROOT}/Src/HAL/position_sensor
    ${PROJ_ROOT}/Lib/Core/Inc
)

# Mock definitions for test environment
add_definitions(-DTEST_ENV)

# Compiler flags for warnings
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unused-function)
elseif(MSVC)
    # The firmware sources contain UTF-8 comments.  Without this flag MSVC
    # falls back to the active Windows code page and can misparse a source
    # file after issuing C4819 encoding warnings.
    add_compile_options(/utf-8)
endif()

function(register_test_target target)
    if(NOT MSVC)
        target_link_libraries(${target} m)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endfunction()

# ===========================================================================
# Source sets
# ===========================================================================

set(FOC_ALGO_SOURCES
    ${PROJ_ROOT}/Src/ALGO/foc/clarke.c
    ${PROJ_ROOT}/Src/ALGO/foc/park.c
    ${PROJ_ROOT}/Src/ALGO/foc/svpwm.c
    ${PROJ_ROOT}/Src/ALGO/foc/trigonometry.c
)

set(MOTOR_ALGO_SOURCES
    ${PROJ_ROOT}/Src/ALGO/motor/calibration_context.c
    ${PROJ_ROOT}/Src/ALGO/motor/rsls_calib.c
)

# ===========================================================================
# Core tests
# ===========================================================================

# Core FOC unit tests (Clarke / Park / SVPWM)
add_executable(test_runner_core
    test_foc_core.c
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/Src/ALGO/foc/foc_algorithm.c
)
register_test_target(test_runner_core)

# Trajectory planner tests
add_executable(test_runner_traj
    test_trap_traj.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/trap_traj.c
)
register_test_target(test_runner_traj)

# PID controller tests
add_executable(test_runner_pid
    test_pid.c
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
)
register_test_target(test_runner_pid)

# Rate limiter tests
add_executable(test_runner_rate_limiter
    test_rate_limiter.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/rate_limiter.c
)
register_test_target(test_runner_rate_limiter)

# LADRC controller tests
add_executable(test_runner_ladrc
    test_ladrc.c
    ${PROJ_ROOT}/Src/ALGO/control/ladrc.c
)
register_test_target(test_runner_ladrc)

# Trigonometry / fast math tests
add_executable(test_runner_trig
    test_trigonometry.c
    ${PROJ_ROOT}/Src/ALGO/foc/trigonometry.c
)
register_test_target(test_runner_trig)

# Integration: full closed-loop simulation (internal plant model)
add_executable(test_runner_integration
    test_foc_integration.c
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
)
register_test_target(test_runner_integration)

# ===========================================================================
# Extended regression tests
# ===========================================================================

# SMO observer tests
add_executable(test_runner_smo
    test_smo_observer.c
    ${PROJ_ROOT}/Src/ALGO/observer/smo_observer.c
)
register_test_target(test_runner_smo)

# ErrorManager tests
add_executable(test_runner_error_manager
    test_error_manager.c
    ${PROJ_ROOT}/Src/UI/error/error_manager.c
)
register_test_target(test_runner_error_manager)

# Calibration context tests
# Note: rsls_calib.c -> calib_*.c -> ADC hardware; provide stubs via test_calib_stubs.
add_executable(test_runner_calibration
    test_calibration.c
    ${PROJ_ROOT}/Src/ALGO/motor/calibration_context.c
)
# Provide stubs for RSLSCalib_GetProgress (referenced by CalibContext_GetProgress)
target_compile_definitions(test_runner_calibration PRIVATE CALIB_STUB_RSLS)
register_test_target(test_runner_calibration)

# Sensor-independent direction/pole calibration boundary. In particular this
# protects 64-bit native counts and the MOTOR_PARAMETERS pole-pair source of
# truth consumed by the next control ISR.
add_executable(test_runner_encoder_calibration_boundary
    test_encoder_calibration_boundary.c
    ${PROJ_ROOT}/Src/ALGO/motor/calib_encoder.c
)
register_test_target(test_runner_encoder_calibration_boundary)

# LED logic tests (pure logic, no HAL)
add_executable(test_runner_led
    test_led.c
)
register_test_target(test_runner_led)

add_executable(test_runner_motor_status_led
    test_motor_status_led.c
    ${PROJ_ROOT}/Src/APP/rtos/motor_status_led.c
)
register_test_target(test_runner_motor_status_led)

# Feedforward + CoggingComp tests
add_executable(test_runner_ff_cogging
    test_feedforward_cogging.c
    ${PROJ_ROOT}/Src/ALGO/control/feedforward.c
    ${PROJ_ROOT}/Src/ALGO/control/torque_utils.c
    ${PROJ_ROOT}/Src/ALGO/control/cogging.c
)
register_test_target(test_runner_ff_cogging)

add_executable(test_runner_param_storage
    test_param_storage.c
    ${PROJ_ROOT}/Src/UI/parameter/param_storage.c
)
register_test_target(test_runner_param_storage)

add_executable(test_runner_bsp_flash
    test_bsp_flash.c
)
register_test_target(test_runner_bsp_flash)

add_executable(test_runner_boot_protocol
    test_boot_protocol.c
    ${PROJ_ROOT}/Src/BOOT/boot_protocol.c
)
target_include_directories(test_runner_boot_protocol PRIVATE
    ${PROJ_ROOT}/Src/BOOT
)
register_test_target(test_runner_boot_protocol)

add_executable(test_runner_control_compensation
    test_control_compensation.c
    ${PROJ_ROOT}/Src/ALGO/control/feedforward.c
    ${PROJ_ROOT}/Src/ALGO/control/field_weakening.c
    ${PROJ_ROOT}/Src/ALGO/control/torque_utils.c
)
register_test_target(test_runner_control_compensation)

add_executable(test_runner_control_commands
    test_control_commands.c
    ${PROJ_ROOT}/Src/ALGO/control/command_limiter.c
    ${PROJ_ROOT}/Src/ALGO/control/torque_utils.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/rate_limiter.c
)
register_test_target(test_runner_control_commands)

add_executable(test_runner_control_dispatch
    test_control_dispatch.c
    ${PROJ_ROOT}/Src/ALGO/control/control.c
)
register_test_target(test_runner_control_dispatch)

add_executable(test_runner_control_impl_safety
    test_control_impl_safety.c
    ${PROJ_ROOT}/Src/ALGO/control/impl.c
    ${PROJ_ROOT}/Src/ALGO/control/torque_utils.c
)
register_test_target(test_runner_control_impl_safety)

add_executable(test_runner_param_typed_access
    test_param_typed_access.c
    ${PROJ_ROOT}/Src/UI/parameter/param_access.c
    ${PROJ_ROOT}/Src/UI/parameter/param_encoder_calibration.c
)
register_test_target(test_runner_param_typed_access)

add_executable(test_runner_param_table_bindings
    test_param_table_bindings.c
    ${PROJ_ROOT}/Src/UI/parameter/param_table.c
)
register_test_target(test_runner_param_table_bindings)

add_executable(test_runner_parameter_bindings_settings
    test_parameter_bindings_settings.c
    ${PROJ_ROOT}/Src/APP/settings/parameter_bindings_settings.c
    ${PROJ_ROOT}/Src/UI/parameter/param_table.c
)
target_include_directories(test_runner_parameter_bindings_settings PRIVATE
    ${PROJ_ROOT}/Src/SAFE
)
register_test_target(test_runner_parameter_bindings_settings)

add_executable(test_runner_app_comm_bootstrap
    test_app_comm_bootstrap.c
    ${PROJ_ROOT}/Src/APP/init/app_comm_bootstrap.c
)
target_include_directories(test_runner_app_comm_bootstrap BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/app_comm_bootstrap_mocks
)
target_include_directories(test_runner_app_comm_bootstrap PRIVATE
    ${PROJ_ROOT}/Src/APP/init
)
register_test_target(test_runner_app_comm_bootstrap)

# App_Init must fail closed if an MCU/board adapter cannot bind or start.
add_executable(test_runner_app_init_bindings
    test_app_init_bindings.c
    ${PROJ_ROOT}/Src/APP/init/app_init.c
)
target_include_directories(test_runner_app_init_bindings BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/app_init_binding_mocks
)
target_include_directories(test_runner_app_init_bindings PRIVATE
    ${PROJ_ROOT}/Src/APP/init
)
register_test_target(test_runner_app_init_bindings)

add_executable(test_runner_runtime_settings
    test_runtime_settings.c
    ${PROJ_ROOT}/Src/APP/settings/runtime_settings.c
)
target_include_directories(test_runner_runtime_settings PRIVATE
    ${PROJ_ROOT}/Src/SAFE
)
register_test_target(test_runner_runtime_settings)

add_executable(test_runner_cmd_service_persistent_rollback
    test_cmd_service_persistent_rollback.c
    ${PROJ_ROOT}/Src/APP/rtos/cmd_service.c
)
register_test_target(test_runner_cmd_service_persistent_rollback)

add_executable(test_runner_vofa_commands
    test_vofa_commands.c
)
target_include_directories(test_runner_vofa_commands PRIVATE
    ${PROJ_ROOT}/Src/BOOT
    ${PROJ_ROOT}/Src/UI/vofa
)
register_test_target(test_runner_vofa_commands)

add_executable(test_runner_vofa_commands_xstar
    test_vofa_commands.c
)
target_include_directories(test_runner_vofa_commands_xstar PRIVATE
    ${PROJ_ROOT}/Src/BOOT
    ${PROJ_ROOT}/Src/UI/vofa
)
target_compile_definitions(test_runner_vofa_commands_xstar PRIVATE BOARD_XSTAR)
register_test_target(test_runner_vofa_commands_xstar)

# Vector CAN protocol encode/decode tests
# CalibContext_GetProgress and RSLSCalib_GetProgress are stubbed in test_comm_protocol.c
add_executable(test_runner_comm_protocol
    test_comm_protocol.c
    ${PROJ_ROOT}/Src/COMM/protocol/vector/vector_protocol.c
)
target_compile_definitions(test_runner_comm_protocol PRIVATE
    FW_VERSION_MAJOR=9
    FW_VERSION_MINOR=8
    FW_VERSION_PATCH=7
    MOCK_HAL_RESET_OBSERVABLE
)
register_test_target(test_runner_comm_protocol)

add_executable(test_runner_comm_protocol_xstar
    test_comm_protocol.c
    ${PROJ_ROOT}/Src/COMM/protocol/vector/vector_protocol.c
)
target_compile_definitions(test_runner_comm_protocol_xstar PRIVATE
    BOARD_XSTAR
    FW_VERSION_MAJOR=9
    FW_VERSION_MINOR=8
    FW_VERSION_PATCH=7
    MOCK_HAL_RESET_OBSERVABLE
)
register_test_target(test_runner_comm_protocol_xstar)

# Communication command execution and SI/internal unit boundary tests
add_executable(test_runner_comm_executor
    test_comm_executor.c
    ${PROJ_ROOT}/Src/COMM/executor/executor.c
)
register_test_target(test_runner_comm_executor)

# Protocol manager watchdog and dispatch behavior tests
add_executable(test_runner_comm_manager
    test_comm_manager.c
    ${PROJ_ROOT}/Src/COMM/manager/manager.c
)
register_test_target(test_runner_comm_manager)

# CAN transport adapter contract: generic frames never leak into the BSP and
# receive callbacks cross the seam only after validation.
add_executable(test_runner_can_transport
    test_can_transport.c
    ${PROJ_ROOT}/Src/COMM/transport/can_transport.c
)
register_test_target(test_runner_can_transport)

# Compile the concrete STM32 FDCAN BSP against host HAL doubles. This keeps
# filter/notification sequencing, frame validation, and tracked TX observable.
add_executable(test_runner_bsp_can
    test_bsp_can.c
)
target_include_directories(test_runner_bsp_can BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/bsp_can_mocks
)
register_test_target(test_runner_bsp_can)

# Typed ADC/PWM binding contract used by swappable MCU/board adapters.
add_executable(test_runner_hal_port_binding
    test_hal_port_binding.c
    ${PROJ_ROOT}/Src/HAL/stm32g4/hal_adc.c
    ${PROJ_ROOT}/Src/HAL/stm32g4/hal_pwm.c
)
target_include_directories(test_runner_hal_port_binding PRIVATE
    ${PROJ_ROOT}/Src/HAL/stm32g4
)
register_test_target(test_runner_hal_port_binding)

# Compile the actual board ports against register/HAL test doubles. These
# protect the ADC trigger channel, bridge enable rollback, phase mapping, and
# calibration timeout/interrupt-restore behavior.
add_executable(test_runner_motor_hal_g431
    test_motor_hal_g431.c
    ${PROJ_ROOT}/Src/HAL/stm32g4/motor_hal_g431.c
)
target_include_directories(test_runner_motor_hal_g431 BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/motor_hal_g431_mocks
)
target_compile_definitions(test_runner_motor_hal_g431 PRIVATE CORE_ADC_H)
if(MSVC)
    target_compile_options(test_runner_motor_hal_g431 PRIVATE
        /FI"${CMAKE_SOURCE_DIR}/motor_hal_g431_mocks/motor_adc.h"
    )
else()
    target_compile_options(test_runner_motor_hal_g431 PRIVATE
        -include "${CMAKE_SOURCE_DIR}/motor_hal_g431_mocks/motor_adc.h"
    )
endif()
register_test_target(test_runner_motor_hal_g431)

add_executable(test_runner_motor_hal_xstar
    test_motor_hal_xstar.c
    ${PROJ_ROOT}/Src/HAL/stm32g4/motor_hal_xstar.c
)
target_include_directories(test_runner_motor_hal_xstar BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/motor_hal_xstar_mocks
)
target_compile_definitions(test_runner_motor_hal_xstar PRIVATE BOARD_XSTAR)
register_test_target(test_runner_motor_hal_xstar)

add_executable(test_runner_app_freertos_runtime_stats
    test_app_freertos_runtime_stats.c
    ${PROJ_ROOT}/Lib/Core/Src/app_freertos.c
)
register_test_target(test_runner_app_freertos_runtime_stats)

# MIT/CANopen frame initialization tests
add_executable(test_runner_comm_frame_builders
    test_comm_frame_builders.c
    ${PROJ_ROOT}/Src/COMM/protocol/mit/mit_protocol.c
    ${PROJ_ROOT}/Src/COMM/protocol/canopen/canopen_protocol.c
)
register_test_target(test_runner_comm_frame_builders)

add_executable(test_runner_encoder_position
    test_encoder_position_tracker.c
)
target_include_directories(test_runner_encoder_position PRIVATE
    ${PROJ_ROOT}/Src/HAL/encoder
)
register_test_target(test_runner_encoder_position)

add_executable(test_runner_encoder_angle_math
    test_encoder_angle_math.c
)
target_include_directories(test_runner_encoder_angle_math PRIVATE
    ${PROJ_ROOT}/Src/HAL/encoder
)
register_test_target(test_runner_encoder_angle_math)

# The encoder HAL initializes the sensor through the PositionSensor selection
# seam. Compile that real path against explicit driver test doubles so a no-op
# adapter or a broken build-time selection cannot regress unnoticed.
set(POSITION_SENSOR_INIT_TEST_SOURCES
    test_hal_encoder_init.c
    ${PROJ_ROOT}/Src/HAL/stm32g4/hal_encoder.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_selection.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_mt6816.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_tmr3109.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_hall.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_abz.c
)

function(add_hal_encoder_init_test suffix)
    set(target "test_runner_hal_encoder_init_${suffix}")
    add_executable(${target} ${POSITION_SENSOR_INIT_TEST_SOURCES})
    target_include_directories(${target} BEFORE PRIVATE
        ${CMAKE_SOURCE_DIR}/encoder_init_mocks
        ${PROJ_ROOT}/Src/HAL/position_sensor
    )
    target_compile_definitions(${target} PRIVATE
        POSITION_SENSOR_TEST_USE_DRIVERS
        ${ARGN}
    )
    register_test_target(${target})
endfunction()

add_hal_encoder_init_test(mt6816
    HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_MT6816
)
add_hal_encoder_init_test(tmr3109
    HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_TMR3109
)
add_hal_encoder_init_test(xstar_hall
    BOARD_XSTAR
    HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_HALL
)
add_hal_encoder_init_test(xstar_abz
    BOARD_XSTAR
    HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_ABZ
)
add_hal_encoder_init_test(unsupported
    HW_POSITION_SENSOR_MODE=99u
)


add_executable(test_runner_encoder_failure_guard
    test_encoder_failure_guard.c
    ${PROJ_ROOT}/Src/APP/isr/encoder_failure_guard.c
)
target_include_directories(test_runner_encoder_failure_guard PRIVATE
    ${PROJ_ROOT}/Src/APP/isr
)
register_test_target(test_runner_encoder_failure_guard)

add_executable(test_runner_watchdog_supervisor
    test_watchdog_supervisor.c
    ${PROJ_ROOT}/Src/SAFE/watchdog_supervisor.c
)
target_include_directories(test_runner_watchdog_supervisor PRIVATE
    ${PROJ_ROOT}/Src/SAFE
)
register_test_target(test_runner_watchdog_supervisor)

add_executable(test_runner_adc_sample_guard
    test_adc_sample_guard.c
    ${PROJ_ROOT}/Src/APP/isr/adc_sample_guard.c
)
target_include_directories(test_runner_adc_sample_guard PRIVATE
    ${PROJ_ROOT}/Src/APP/isr
)
register_test_target(test_runner_adc_sample_guard)

add_executable(test_runner_adc_shared_irq
    test_adc_shared_irq.c
    ${PROJ_ROOT}/Lib/Core/Src/stm32g4xx_it.c
)
register_test_target(test_runner_adc_shared_irq)

add_executable(test_runner_foc_readiness_gate
    test_foc_readiness_gate.c
    ${PROJ_ROOT}/Src/APP/isr/isr_foc.c
    ${PROJ_ROOT}/Src/APP/isr/adc_sample_guard.c
    ${PROJ_ROOT}/Src/APP/isr/encoder_failure_guard.c
    ${PROJ_ROOT}/Src/SAFE/watchdog_supervisor.c
)
target_include_directories(test_runner_foc_readiness_gate BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/foc_readiness_mocks
)
target_include_directories(test_runner_foc_readiness_gate PRIVATE
    ${PROJ_ROOT}/Src/APP/isr
    ${PROJ_ROOT}/Src/SAFE
    ${PROJ_ROOT}/Src/APP/init
)
register_test_target(test_runner_foc_readiness_gate)

add_executable(test_runner_foc_readiness_gate_xstar
    test_foc_readiness_gate.c
    ${PROJ_ROOT}/Src/APP/isr/isr_foc.c
    ${PROJ_ROOT}/Src/APP/isr/adc_sample_guard.c
    ${PROJ_ROOT}/Src/APP/isr/encoder_failure_guard.c
    ${PROJ_ROOT}/Src/SAFE/watchdog_supervisor.c
)
target_include_directories(test_runner_foc_readiness_gate_xstar BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/foc_readiness_mocks
)
target_include_directories(test_runner_foc_readiness_gate_xstar PRIVATE
    ${PROJ_ROOT}/Src/APP/isr
    ${PROJ_ROOT}/Src/SAFE
    ${PROJ_ROOT}/Src/APP/init
)
target_compile_definitions(test_runner_foc_readiness_gate_xstar PRIVATE BOARD_XSTAR)
register_test_target(test_runner_foc_readiness_gate_xstar)

add_executable(test_runner_motor_adc_filters
    test_motor_adc_filters.c
    ${PROJ_ROOT}/Src/HAL/stm32g4/motor_adc.c
)
target_include_directories(test_runner_motor_adc_filters PRIVATE
    ${PROJ_ROOT}/Src/HAL/stm32g4
)
register_test_target(test_runner_motor_adc_filters)

if(CMAKE_VERSION VERSION_LESS "3.12")
    find_package(PythonInterp 3 REQUIRED)
    set(VECTORFOC_PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE})
else()
    find_package(Python3 3 REQUIRED COMPONENTS Interpreter)
    set(VECTORFOC_PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
endif()
add_test(
    NAME test_app_header_tools
    COMMAND ${VECTORFOC_PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/test_app_header_tools.py
)
set_tests_properties(test_app_header_tools PROPERTIES
    WORKING_DIRECTORY ${PROJ_ROOT}
)

add_test(
    NAME test_clock_configuration
    COMMAND ${VECTORFOC_PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/test_clock_configuration.py
)
set_tests_properties(test_clock_configuration PROPERTIES
    WORKING_DIRECTORY ${PROJ_ROOT}
)

add_test(
    NAME test_position_sensor_architecture
    COMMAND ${VECTORFOC_PYTHON_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/test_position_sensor_architecture.py
)
set_tests_properties(test_position_sensor_architecture PROPERTIES
    WORKING_DIRECTORY ${PROJ_ROOT}
)
add_test(
    NAME test_layer_architecture
    COMMAND ${VECTORFOC_PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/test_layer_architecture.py
)
set_tests_properties(test_layer_architecture PROPERTIES
    WORKING_DIRECTORY ${PROJ_ROOT}
)

# DS402 output safety contract: run/calibration states energize deliberately;
# idle and fault states remain de-energized.
add_executable(test_runner_fsm_safety
    test_fsm_safety.c
    ${PROJ_ROOT}/Src/ALGO/motor/fsm.c
)
register_test_target(test_runner_fsm_safety)

add_executable(test_runner_motor_state_safety
    test_motor_state_safety.c
    ${PROJ_ROOT}/Src/ALGO/motor/motor.c
    ${PROJ_ROOT}/Src/ALGO/motor/fsm.c
)
register_test_target(test_runner_motor_state_safety)

add_executable(test_runner_fault_detection_safety
    test_fault_detection_safety.c
    ${PROJ_ROOT}/Src/SAFE/fault_detection.c
    ${PROJ_ROOT}/Src/SAFE/fault_def.c
)
target_compile_definitions(test_runner_fault_detection_safety PRIVATE
    TEST_USE_REAL_SYSTEM_TICK
)
register_test_target(test_runner_fault_detection_safety)

# ===========================================================================
# MotorStateTask smoke tests
# ===========================================================================

# FSM + MotorStateTask smoke test with typed ADC register mocks.
add_executable(test_runner_foc_state
    test_foc_state.c
    mock_hal.c
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/Src/ALGO/foc/foc_algorithm.c
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
    ${PROJ_ROOT}/Src/ALGO/motor/motor.c
    ${PROJ_ROOT}/Src/ALGO/motor/fsm.c
    ${PROJ_ROOT}/Src/ALGO/motor/current_calib.c
    ${PROJ_ROOT}/Src/ALGO/motor/safety_control.c
    ${PROJ_ROOT}/Src/ALGO/control/ladrc.c
    ${PROJ_ROOT}/Src/ALGO/control/feedforward.c
    ${PROJ_ROOT}/Src/ALGO/control/cogging.c
    ${PROJ_ROOT}/Src/ALGO/control/control.c
    ${PROJ_ROOT}/Src/ALGO/control/command_limiter.c
    ${PROJ_ROOT}/Src/ALGO/control/torque_utils.c
    ${PROJ_ROOT}/Src/ALGO/control/impl.c
    ${PROJ_ROOT}/Src/ALGO/control/inner.c
    ${PROJ_ROOT}/Src/ALGO/control/outer.c
    ${PROJ_ROOT}/Src/ALGO/control/field_weakening.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/trap_traj.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/rate_limiter.c
)
if(NOT MSVC)
    target_link_libraries(test_runner_foc_state m)
endif()
register_test_target(test_runner_foc_state)

add_executable(test_runner_foc_closed_loop
    test_foc_closed_loop.c
    mock_hal.c
    motor_plant.c
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/Src/ALGO/foc/foc_algorithm.c
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
    ${PROJ_ROOT}/Src/ALGO/motor/motor.c
    ${PROJ_ROOT}/Src/ALGO/motor/fsm.c
    ${PROJ_ROOT}/Src/ALGO/motor/current_calib.c
    ${PROJ_ROOT}/Src/ALGO/motor/safety_control.c
    ${PROJ_ROOT}/Src/ALGO/control/ladrc.c
    ${PROJ_ROOT}/Src/ALGO/control/feedforward.c
    ${PROJ_ROOT}/Src/ALGO/control/cogging.c
    ${PROJ_ROOT}/Src/ALGO/control/control.c
    ${PROJ_ROOT}/Src/ALGO/control/command_limiter.c
    ${PROJ_ROOT}/Src/ALGO/control/torque_utils.c
    ${PROJ_ROOT}/Src/ALGO/control/impl.c
    ${PROJ_ROOT}/Src/ALGO/control/inner.c
    ${PROJ_ROOT}/Src/ALGO/control/outer.c
    ${PROJ_ROOT}/Src/ALGO/control/field_weakening.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/trap_traj.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/rate_limiter.c
)
register_test_target(test_runner_foc_closed_loop)

add_executable(test_runner_fault_injection
    test_fault_injection.c
    mock_hal.c
    motor_plant.c
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/Src/ALGO/foc/foc_algorithm.c
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
    ${PROJ_ROOT}/Src/ALGO/motor/motor.c
    ${PROJ_ROOT}/Src/ALGO/motor/fsm.c
    ${PROJ_ROOT}/Src/ALGO/motor/current_calib.c
    ${PROJ_ROOT}/Src/ALGO/motor/safety_control.c
    ${PROJ_ROOT}/Src/ALGO/control/ladrc.c
    ${PROJ_ROOT}/Src/ALGO/control/feedforward.c
    ${PROJ_ROOT}/Src/ALGO/control/cogging.c
    ${PROJ_ROOT}/Src/ALGO/control/control.c
    ${PROJ_ROOT}/Src/ALGO/control/command_limiter.c
    ${PROJ_ROOT}/Src/ALGO/control/torque_utils.c
    ${PROJ_ROOT}/Src/ALGO/control/impl.c
    ${PROJ_ROOT}/Src/ALGO/control/inner.c
    ${PROJ_ROOT}/Src/ALGO/control/outer.c
    ${PROJ_ROOT}/Src/ALGO/control/field_weakening.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/trap_traj.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/rate_limiter.c
)
register_test_target(test_runner_fault_injection)

# Position-sensor selection is a public, hardware-independent seam.  Compile
# the same behavioral contract for every supported sensor and one invalid mode.
set(POSITION_SENSOR_SELECTION_SOURCES
    test_position_sensor_selection.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_selection.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_mt6816.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_tmr3109.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_hall.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_abz.c
)

function(add_position_sensor_selection_test suffix mode)
    set(target "test_runner_position_sensor_selection_${suffix}")
    add_executable(${target} ${POSITION_SENSOR_SELECTION_SOURCES})
    target_include_directories(${target} PRIVATE
        ${PROJ_ROOT}/Src/HAL/position_sensor
    )
    target_compile_definitions(${target} PRIVATE
        HW_POSITION_SENSOR_HALL=1u
        HW_POSITION_SENSOR_ABZ=2u
        HW_POSITION_SENSOR_MT6816=3u
        HW_POSITION_SENSOR_TMR3109=4u
        HW_POSITION_SENSOR_MODE=${mode}
    )
    register_test_target(${target})
endfunction()

add_position_sensor_selection_test(mt6816 HW_POSITION_SENSOR_MT6816)
add_position_sensor_selection_test(tmr3109 HW_POSITION_SENSOR_TMR3109)
add_position_sensor_selection_test(hall HW_POSITION_SENSOR_HALL)
add_position_sensor_selection_test(abz HW_POSITION_SENSOR_ABZ)
add_position_sensor_selection_test(unsupported 99u)

# PositionSensor owns the real-time frame, health counters, and typed
# maintenance boundary. A private fake adapter makes the contract executable
# without exposing a concrete driver handle or touching host hardware.
add_executable(test_runner_position_sensor_runtime
    test_position_sensor_runtime.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor.c
)
target_include_directories(test_runner_position_sensor_runtime PRIVATE
    ${PROJ_ROOT}/Src/HAL/position_sensor
)
register_test_target(test_runner_position_sensor_runtime)

add_executable(test_runner_position_sensor_persistent_only
    test_position_sensor_runtime.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor.c
)
target_include_directories(test_runner_position_sensor_persistent_only PRIVATE
    ${PROJ_ROOT}/Src/HAL/position_sensor
)
target_compile_definitions(test_runner_position_sensor_persistent_only PRIVATE
    POSITION_SENSOR_TEST_PERSISTENT_ONLY
)
register_test_target(test_runner_position_sensor_persistent_only)

add_executable(test_runner_position_sensor_motor_hal
    test_position_sensor_motor_hal.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor_motor_hal.c
)
target_include_directories(test_runner_position_sensor_motor_hal PRIVATE
    ${PROJ_ROOT}/Src/HAL/position_sensor
)
register_test_target(test_runner_position_sensor_motor_hal)

# APP persistence mapping restores Flash calibration only after the selected
# driver has initialized and reset its runtime state.
add_executable(test_runner_encoder_calibration_settings
    test_encoder_calibration_settings.c
    ${PROJ_ROOT}/Src/APP/settings/encoder_calibration_settings.c
    ${PROJ_ROOT}/Src/UI/parameter/param_encoder_calibration.c
    ${PROJ_ROOT}/Src/HAL/position_sensor/position_sensor.c
)
target_include_directories(test_runner_encoder_calibration_settings PRIVATE
    ${PROJ_ROOT}/Src/APP/settings
    ${PROJ_ROOT}/Src/HAL/position_sensor
)
register_test_target(test_runner_encoder_calibration_settings)

# ===========================================================================
# Summary
# ===========================================================================
message(STATUS "VectorFOC Test targets registered:")
message(STATUS "  Automated: core, control, safety, communication, storage, HAL filters, tooling, foc_state, foc_readiness_gate, foc_closed_loop, fault_injection")
