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
    ${PROJ_ROOT}/Lib/Core/Inc
)

# Mock definitions for test environment
add_definitions(-DTEST_ENV)

# Compiler flags for warnings
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unused-function)
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

# LED logic tests (pure logic, no HAL)
add_executable(test_runner_led
    test_led.c
)
register_test_target(test_runner_led)

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
)
register_test_target(test_runner_param_typed_access)

add_executable(test_runner_vofa_commands
    test_vofa_commands.c
)
target_include_directories(test_runner_vofa_commands PRIVATE
    ${PROJ_ROOT}/Src/BOOT
    ${PROJ_ROOT}/Src/UI/vofa
)
register_test_target(test_runner_vofa_commands)

# Vector CAN protocol encode/decode tests
# CalibContext_GetProgress and RSLSCalib_GetProgress are stubbed in test_comm_protocol.c
add_executable(test_runner_comm_protocol
    test_comm_protocol.c
    ${PROJ_ROOT}/Src/COMM/protocol/vector/vector_protocol.c
)
register_test_target(test_runner_comm_protocol)

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
# Legacy/manual tests
# ===========================================================================

# FSM + MotorStateTask smoke test
# Note: EXCLUDE_FROM_ALL ? not part of default build due to deep HAL mock requirements.
# Build manually with: cmake --build . --target test_runner_foc_state
add_executable(test_runner_foc_state EXCLUDE_FROM_ALL
    test_foc_state.c
    mock_hal.c
    motor_plant.c
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
    ${PROJ_ROOT}/Src/ALGO/motor/motor.c
    ${PROJ_ROOT}/Src/ALGO/motor/motor_api.c
    ${PROJ_ROOT}/Src/ALGO/motor/motor_data.c
    ${PROJ_ROOT}/Src/ALGO/motor/fsm.c
    ${PROJ_ROOT}/Src/ALGO/motor/calibration_context.c
    ${PROJ_ROOT}/Src/ALGO/motor/rsls_calib.c
    ${PROJ_ROOT}/Src/ALGO/motor/calib_encoder.c
    ${PROJ_ROOT}/Src/ALGO/motor/calib_resistance.c
    ${PROJ_ROOT}/Src/ALGO/motor/calib_inductance.c
    ${PROJ_ROOT}/Src/ALGO/motor/current_calib.c
    ${PROJ_ROOT}/Src/ALGO/motor/flux_calib.c
    ${PROJ_ROOT}/Src/ALGO/motor/fault_def.c
    ${PROJ_ROOT}/Src/ALGO/motor/fault_detection.c
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
    ${PROJ_ROOT}/Src/ALGO/observer/smo_observer.c
    ${PROJ_ROOT}/Src/UI/error/error_manager.c
    ${PROJ_ROOT}/Src/UI/parameter/param_access.c
    ${PROJ_ROOT}/Src/UI/parameter/param_table.c
    ${PROJ_ROOT}/Src/UI/parameter/param_storage.c
)
if(NOT MSVC)
    target_link_libraries(test_runner_foc_state m)
endif()
# This legacy smoke target directly touches ADC register fields and therefore
# remains a manual compile target until its hardware dependencies are mocked.

# ===========================================================================
# Summary
# ===========================================================================
message(STATUS "VectorFOC Test targets registered:")
message(STATUS "  Automated: core, control, safety, communication, storage, HAL filters, tooling")
message(STATUS "  Manual: foc_state (requires deeper ADC register mocks)")
