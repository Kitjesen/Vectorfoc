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
    ${PROJ_ROOT}/Src/COMM/protocol/vector
    ${PROJ_ROOT}/Src/COMM/transport
    ${PROJ_ROOT}/Src/COMM/executor
    ${PROJ_ROOT}/Src/APP/rtos
    ${PROJ_ROOT}/Src/APP
    ${PROJ_ROOT}/Src/config
    ${PROJ_ROOT}/Src/HAL/interface
)

# Mock definitions for test environment
add_definitions(-DTEST_ENV -DHW_POSITION_SENSOR_MODE=3 -DHW_POSITION_SENSOR_MT6816=3)

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
# ── 原有测试（7 个，已在 CI 中运行）──────────────────────────────────────
# ===========================================================================

# Core FOC unit tests (Clarke / Park / SVPWM)
add_executable(test_runner_core
    test_foc_core.c
    ${FOC_ALGO_SOURCES}
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
# ── 新增测试（本次补全）─────────────────────────────────────────────────
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
    ${PROJ_ROOT}/Src/ALGO/control/cogging.c
)
register_test_target(test_runner_ff_cogging)

# Vector CAN protocol encode/decode tests
# CalibContext_GetProgress and RSLSCalib_GetProgress are stubbed in test_comm_protocol.c
add_executable(test_runner_comm_protocol
    test_comm_protocol.c
    ${PROJ_ROOT}/Src/COMM/protocol/vector/vector_protocol.c
)
register_test_target(test_runner_comm_protocol)

# ===========================================================================
# ── 已有但之前未接入 CI 的测试（接入 ctest，但标记 WILL_FAIL=OFF）──────
# ===========================================================================

# FSM + MotorStateTask smoke test
# Note: EXCLUDE_FROM_ALL — not part of default build due to deep HAL mock requirements.
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
# add_test registered but binary not in default ALL target
add_test(NAME test_runner_foc_state COMMAND test_runner_foc_state)

# ===========================================================================
# Summary
# ===========================================================================
message(STATUS "VectorFOC Test targets registered:")
message(STATUS "  Original (7): core, traj, pid, rate_limiter, ladrc, trig, integration")
message(STATUS "  New      (6): smo, error_manager, calibration, led, ff_cogging, comm_protocol")
message(STATUS "  Extended (1): foc_state (FSM smoke test)")
