cmake_minimum_required(VERSION 3.10)

project(VectorFOC_Test C)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(CMAKE_C_STANDARD 99)
enable_testing()
add_test(NAME test_pwm_configuration
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/check_pwm_configuration.cmake)
add_test(NAME test_app_header_tools
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_app_header_tools.py)

# Define project root relative to test dir
set(PROJ_ROOT "${CMAKE_SOURCE_DIR}/..")

# For PC test environment, use mocks directory FIRST to override real headers
include_directories(
    ${CMAKE_SOURCE_DIR}/mocks  # Mock headers first!
)

# First-party modules; STM32 peripheral headers are supplied by mocks.
include_directories(
    ${PROJ_ROOT}
    ${PROJ_ROOT}/src
    ${PROJ_ROOT}/src/foc
    ${PROJ_ROOT}/src/motor
    ${PROJ_ROOT}/src/safety
    ${PROJ_ROOT}/src/config
    ${PROJ_ROOT}/src/comm
    ${PROJ_ROOT}/src/app
    ${PROJ_ROOT}/src/boot
    ${PROJ_ROOT}/src/hal
    ${PROJ_ROOT}/src/hal/encoder
    ${PROJ_ROOT}/src/hal/bsp
    ${PROJ_ROOT}/algorithm
)

# Compiler flags for warnings
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unused-function)
endif()

function(enable_test_assertions target)
    # CMake defines NDEBUG for Release/RelWithDebInfo.  These executables use
    # assert() as their test oracle, so explicitly keep it active in every
    # configuration instead of allowing a Release build to become a no-op.
    if(MSVC)
        target_compile_options(${target} PRIVATE /UNDEBUG)
    else()
        target_compile_options(${target} PRIVATE -UNDEBUG)
    endif()
endfunction()

function(register_test_target target)
    target_compile_definitions(${target} PRIVATE
        TEST_ENV
        HW_POSITION_SENSOR_MODE=3
        HW_POSITION_SENSOR_MT6816=3
    )
    if(NOT MSVC)
        target_link_libraries(${target} PRIVATE m)
    endif()
    enable_test_assertions(${target})
    add_test(NAME ${target} COMMAND ${target})
endfunction()

# The timing header must accept valid integer divisions and reject every
# invalid schedule at compile time.  These checks run while configuring the
# host suite, so a broken guard cannot be hidden behind an unbuilt target.
function(check_frequency_configuration check_name expect_success)
    set(check_binary_dir "${CMAKE_BINARY_DIR}/frequency_checks/${check_name}")
    try_compile(check_succeeded
        "${check_binary_dir}"
        SOURCES "${CMAKE_SOURCE_DIR}/frequency_configuration_probe.c"
        COMPILE_DEFINITIONS
            -DTEST_ENV
            -I${CMAKE_SOURCE_DIR}/mocks
            -I${PROJ_ROOT}/src
            ${ARGN}
        OUTPUT_VARIABLE check_output
    )
    if(expect_success AND NOT check_succeeded)
        message(FATAL_ERROR
            "Frequency configuration '${check_name}' should compile:\n${check_output}")
    elseif(NOT expect_success AND check_succeeded)
        message(FATAL_ERROR
            "Frequency configuration '${check_name}' should be rejected")
    endif()
endfunction()

check_frequency_configuration(valid_integer_divisions TRUE
    -DVELOCITY_LOOP_FREQUENCY_HZ=4000
    -DPOSITION_LOOP_FREQUENCY_HZ=2000
    -DSTATE_MACHINE_UPDATE_FREQUENCY_HZ=1000
    -DTORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ=2500
    -DSCOPE_LOG_FREQUENCY_HZ=500)

foreach(schedule_macro IN ITEMS
        VELOCITY_LOOP_FREQUENCY_HZ
        POSITION_LOOP_FREQUENCY_HZ
        STATE_MACHINE_UPDATE_FREQUENCY_HZ
        TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ
        SCOPE_LOG_FREQUENCY_HZ)
    string(TOLOWER "${schedule_macro}" schedule_name)
    check_frequency_configuration(${schedule_name}_zero FALSE
        -D${schedule_macro}=0)
    check_frequency_configuration(${schedule_name}_above_base FALSE
        -D${schedule_macro}=40000)
    check_frequency_configuration(${schedule_name}_not_divisible FALSE
        -D${schedule_macro}=3000)
endforeach()

# ===========================================================================
# Source sets
# ===========================================================================

set(ALGORITHM_SOURCES
    ${PROJ_ROOT}/algorithm/foc_current_loop.c
    ${PROJ_ROOT}/algorithm/foc_transforms.c
    ${PROJ_ROOT}/algorithm/field_weakening.c
    ${PROJ_ROOT}/algorithm/torque_feedforward.c
    ${PROJ_ROOT}/algorithm/ladrc_controller.c
    ${PROJ_ROOT}/algorithm/pid_controller.c
    ${PROJ_ROOT}/algorithm/command_limiter.c
    ${PROJ_ROOT}/algorithm/sliding_mode_observer.c
    ${PROJ_ROOT}/algorithm/trapezoid_trajectory.c
)
add_library(vector_algorithm_host STATIC ${ALGORITHM_SOURCES})
target_include_directories(vector_algorithm_host PUBLIC
    ${PROJ_ROOT}
    ${PROJ_ROOT}/algorithm
)
if(NOT MSVC)
    target_link_libraries(vector_algorithm_host PUBLIC m)
endif()

set(MOTOR_ALGO_SOURCES
    ${PROJ_ROOT}/src/motor/calibration_state.c
    ${PROJ_ROOT}/src/motor/resistance_inductance_calibration.c
)

# ===========================================================================
# ── 原有测试（7 个，已在 CI 中运行）──────────────────────────────────────
# ===========================================================================

# Core FOC unit tests (Clarke / Park / SVPWM)
add_executable(test_runner_core
    test_foc_core.c
)
target_link_libraries(test_runner_core PRIVATE vector_algorithm_host)
register_test_target(test_runner_core)

# Trajectory planner tests
add_executable(test_runner_traj
    test_trap_traj.c
)
target_link_libraries(test_runner_traj PRIVATE vector_algorithm_host)
register_test_target(test_runner_traj)

# PID controller tests
add_executable(test_runner_pid
    test_pid.c
)
target_link_libraries(test_runner_pid PRIVATE vector_algorithm_host)
register_test_target(test_runner_pid)

# Rate limiter tests
add_executable(test_runner_rate_limiter
    test_rate_limiter.c
)
target_link_libraries(test_runner_rate_limiter PRIVATE vector_algorithm_host)
register_test_target(test_runner_rate_limiter)

# LADRC controller tests
add_executable(test_runner_ladrc
    test_ladrc.c
)
target_link_libraries(test_runner_ladrc PRIVATE vector_algorithm_host)
register_test_target(test_runner_ladrc)

# Trigonometry / fast math tests
add_executable(test_runner_trig
    test_trigonometry.c
)
target_link_libraries(test_runner_trig PRIVATE vector_algorithm_host)
register_test_target(test_runner_trig)

# Integration: full closed-loop simulation (internal plant model)
add_executable(test_runner_integration
    test_foc_integration.c
)
target_link_libraries(test_runner_integration PRIVATE vector_algorithm_host)
register_test_target(test_runner_integration)

# ===========================================================================
# ── 新增测试（本次补全）─────────────────────────────────────────────────
# ===========================================================================

# SMO observer tests
add_executable(test_runner_smo
    test_smo_observer.c
    ${PROJ_ROOT}/src/foc/observer_adapter.c
)
target_link_libraries(test_runner_smo PRIVATE vector_algorithm_host)
register_test_target(test_runner_smo)

# ErrorManager tests
add_executable(test_runner_error_manager
    test_error_manager.c
    ${PROJ_ROOT}/src/safety/error_manager.c
)
register_test_target(test_runner_error_manager)

# Calibration context tests
# This target tests shared calibration context; the electrical target below runs R/L.
add_executable(test_runner_calibration
    test_calibration.c
    ${PROJ_ROOT}/src/motor/calibration_state.c
)
# Provide stubs for RSLSCalib_GetProgress (referenced by CalibContext_GetProgress)
target_compile_definitions(test_runner_calibration PRIVATE CALIB_STUB_RSLS)
register_test_target(test_runner_calibration)

# Exercise real calibration and parameter storage with peripheral-only mocks.
add_executable(test_runner_electrical_calibration
    test_electrical_calibration.c
    ${PROJ_ROOT}/src/motor/resistance_inductance_calibration.c
)
register_test_target(test_runner_electrical_calibration)
add_executable(test_runner_parameter_bindings
    test_parameter_bindings.c
    ${PROJ_ROOT}/src/config/parameter_registry.c
)
register_test_target(test_runner_parameter_bindings)

add_executable(test_runner_parameter_typed_access
    test_parameter_typed_access.c
    ${PROJ_ROOT}/src/config/parameter_access.c
)
register_test_target(test_runner_parameter_typed_access)


# LED logic tests (pure logic, no HAL)
add_executable(test_runner_led
    test_led.c
)
register_test_target(test_runner_led)

# Feedforward + CoggingComp tests
add_executable(test_runner_ff_cogging
    test_feedforward_cogging.c
    ${PROJ_ROOT}/src/foc/runtime_compensation.c
    ${PROJ_ROOT}/src/foc/cogging_compensation.c
)
target_link_libraries(test_runner_ff_cogging PRIVATE vector_algorithm_host)
register_test_target(test_runner_ff_cogging)

# Vector CAN protocol encode/decode tests
# CalibContext_GetProgress and RSLSCalib_GetProgress are stubbed in test_comm_protocol.c
add_executable(test_runner_comm_protocol
    test_comm_protocol.c
    ${PROJ_ROOT}/src/comm/protocol_vector.c
)
register_test_target(test_runner_comm_protocol)

# Real CAN routing, codecs and task scheduling; only hardware edges are mocked.
add_executable(test_runner_comm_simplify
    test_comm_simplify.c
    ${PROJ_ROOT}/src/comm/protocol_dispatcher.c
    ${PROJ_ROOT}/src/comm/protocol_vector.c
    ${PROJ_ROOT}/src/comm/protocol_canopen.c
    ${PROJ_ROOT}/src/comm/protocol_mit.c
    ${PROJ_ROOT}/src/app/communication_task.c
)
target_include_directories(test_runner_comm_simplify BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/comm_support
    ${CMAKE_SOURCE_DIR}/safety_support
)
target_include_directories(test_runner_comm_simplify PRIVATE
    ${PROJ_ROOT}/src/hal/bsp
)
register_test_target(test_runner_comm_simplify)

# Exercise every control mode and observe private state without a public test API.
# control_probe.c includes control_dispatcher.c; do not compile control_dispatcher.c twice here.
add_executable(test_runner_control_modes
    test_control_modes.c
    control_support/control_probe.c
    ${PROJ_ROOT}/src/foc/outer_control.c
    ${PROJ_ROOT}/src/foc/runtime_compensation.c
    ${PROJ_ROOT}/src/foc/cogging_compensation.c
    ${PROJ_ROOT}/src/foc/observer_adapter.c
)
target_include_directories(test_runner_control_modes BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/control_support
)
target_link_libraries(test_runner_control_modes PRIVATE vector_algorithm_host)
register_test_target(test_runner_control_modes)

# Real FSM/protection/calibration/PWM wrappers; stubs stop at board peripherals.
add_executable(test_runner_startup
        test_startup_safety.c
        safety_support/startup_test_support.c
        ${PROJ_ROOT}/src/motor/motor_runtime.c
        ${PROJ_ROOT}/src/motor/drive_state_machine.c
        ${PROJ_ROOT}/src/motor/current_calibration.c
        ${PROJ_ROOT}/src/motor/calibration_state.c
        ${PROJ_ROOT}/src/foc/cogging_compensation.c
        ${PROJ_ROOT}/src/foc/runtime_compensation.c
        ${PROJ_ROOT}/src/foc/observer_adapter.c
        ${PROJ_ROOT}/src/safety/safety_manager.c
        ${PROJ_ROOT}/src/safety/fault_monitor.c
        ${PROJ_ROOT}/src/safety/fault_definition.c
        ${PROJ_ROOT}/src/hal/pwm_interface.c
        ${PROJ_ROOT}/src/hal/bsp/board_adc.c
)
target_compile_definitions(test_runner_startup PRIVATE STARTUP_TEST)
target_include_directories(test_runner_startup BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/safety_support
)
target_include_directories(test_runner_startup PRIVATE
    ${PROJ_ROOT}/src/hal/bsp
    ${PROJ_ROOT}/src/config
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_runner_startup PRIVATE -ffunction-sections -fdata-sections)
    target_link_libraries(test_runner_startup PRIVATE -Wl,--gc-sections)
endif()
target_link_libraries(test_runner_startup PRIVATE vector_algorithm_host)
register_test_target(test_runner_startup)

# Small deterministic guards used directly from interrupt/supervision paths.
add_executable(test_runner_adc_sample_guard
    test_adc_sample_guard.c
    ${PROJ_ROOT}/src/safety/adc_sample_guard.c
)
register_test_target(test_runner_adc_sample_guard)

add_executable(test_runner_encoder_failure_guard
    test_encoder_failure_guard.c
    ${PROJ_ROOT}/src/safety/encoder_failure_guard.c
)
register_test_target(test_runner_encoder_failure_guard)

add_executable(test_runner_watchdog_supervisor
    test_watchdog_supervisor.c
    ${PROJ_ROOT}/src/safety/watchdog_supervisor.c
)
register_test_target(test_runner_watchdog_supervisor)

# Encoder math is header-only so sensor drivers can share one exact wrapping
# and multi-turn contract without introducing another runtime layer.
add_executable(test_runner_encoder_angle_math
    test_encoder_angle_math.c
)
target_include_directories(test_runner_encoder_angle_math PRIVATE
    ${PROJ_ROOT}/src/hal/encoder
)
register_test_target(test_runner_encoder_angle_math)

add_executable(test_runner_encoder_position_tracker
    test_encoder_position_tracker.c
)
target_include_directories(test_runner_encoder_position_tracker PRIVATE
    ${PROJ_ROOT}/src/hal/encoder
)
register_test_target(test_runner_encoder_position_tracker)

add_executable(test_runner_position_sensor_runtime
    test_position_sensor_runtime.c
    ${PROJ_ROOT}/src/hal/encoder/position_sensor.c
)
target_include_directories(test_runner_position_sensor_runtime BEFORE PRIVATE
    ${PROJ_ROOT}/src/hal/encoder
)
register_test_target(test_runner_position_sensor_runtime)

add_executable(test_runner_position_sensor_selection
    test_position_sensor_selection.c
    ${PROJ_ROOT}/src/hal/encoder/position_sensor_selection.c
)
register_test_target(test_runner_position_sensor_selection)

add_executable(test_runner_boot_protocol
    test_boot_protocol.c
    ${PROJ_ROOT}/src/boot/boot_protocol.c
)
target_include_directories(test_runner_boot_protocol BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/boot_support
    ${PROJ_ROOT}/src/boot
)
register_test_target(test_runner_boot_protocol)

# ===========================================================================
# Summary
# ===========================================================================
message(STATUS "VectorFOC: 27 C tests, 2 tooling/configuration tests and 16 timing compilation checks")
