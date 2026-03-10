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

# Only include algorithm directories (no HAL/CMSIS)
include_directories(
    ${PROJ_ROOT}/Src/ALGO
    ${PROJ_ROOT}/Src/ALGO/foc
    ${PROJ_ROOT}/Src/ALGO/motor
    ${PROJ_ROOT}/Src/ALGO/pid
    ${PROJ_ROOT}/Src/ALGO/control
    ${PROJ_ROOT}/Src/ALGO/observer
    ${PROJ_ROOT}/Src/ALGO/trajectory
    ${PROJ_ROOT}/Src/COMM
    ${PROJ_ROOT}/Src/COMM/executor
    ${PROJ_ROOT}/Src/COMM/manager
    ${PROJ_ROOT}/Src/COMM/protocol
    ${PROJ_ROOT}/Src/COMM/protocol/canopen
    ${PROJ_ROOT}/Src/COMM/protocol/inovxio
    ${PROJ_ROOT}/Src/COMM/protocol/mit
    ${PROJ_ROOT}/Src/COMM/transport
    ${PROJ_ROOT}/Src/HAL/interface
    ${PROJ_ROOT}/Src/UI/parameter
    ${PROJ_ROOT}/Src/UI/vofa
)

# Mock definitions for test environment
add_definitions(-DTEST_ENV)

# Compiler flags for warnings
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
elseif(MSVC)
    add_compile_options(/utf-8)
endif()

function(register_test_target target)
    if(NOT MSVC)
        target_link_libraries(${target} m)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endfunction()

# Source Files for Core FOC Algorithm (pure math, no HAL dependency)
set(FOC_ALGO_SOURCES
    ${PROJ_ROOT}/Src/ALGO/foc/clarke.c
    ${PROJ_ROOT}/Src/ALGO/foc/park.c
    ${PROJ_ROOT}/Src/ALGO/foc/svpwm.c
    ${PROJ_ROOT}/Src/ALGO/foc/trigonometry.c
)

# --- Target: Core Unit Tests (FOC transforms only) ---
add_executable(test_runner_core
    test_foc_core.c
    ${FOC_ALGO_SOURCES}
)
register_test_target(test_runner_core)

# --- Target: Trajectory Planner Tests ---
add_executable(test_runner_traj
    test_trap_traj.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/trap_traj.c
)
register_test_target(test_runner_traj)

# --- Target: PID Tests ---
add_executable(test_runner_pid
    test_pid.c
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
)
register_test_target(test_runner_pid)

# --- Target: Rate Limiter Tests ---
add_executable(test_runner_rate_limiter
    test_rate_limiter.c
    ${PROJ_ROOT}/Src/ALGO/trajectory/rate_limiter.c
)
register_test_target(test_runner_rate_limiter)

# --- Target: LADRC Tests ---
add_executable(test_runner_ladrc
    test_ladrc.c
    ${PROJ_ROOT}/Src/ALGO/control/ladrc.c
)
register_test_target(test_runner_ladrc)

# --- Target: Trigonometry Tests ---
add_executable(test_runner_trig
    test_trigonometry.c
    ${PROJ_ROOT}/Src/ALGO/foc/trigonometry.c
)
register_test_target(test_runner_trig)

# --- Target: Integration Tests (Full closed-loop simulation) ---
add_executable(test_runner_integration
    test_foc_integration.c
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
)
register_test_target(test_runner_integration)

# --- Target: Protocol Parser Tests ---
add_executable(test_runner_protocols
    test_protocols.c
    ${PROJ_ROOT}/Src/COMM/protocol/mit/mit_protocol.c
    ${PROJ_ROOT}/Src/COMM/protocol/canopen/canopen_protocol.c
)
register_test_target(test_runner_protocols)

# --- Target: VOFA bus scan helper tests ---
add_executable(test_runner_vofa_scan
    test_vofa_bus_scan.c
    ${PROJ_ROOT}/Src/UI/vofa/vofa_bus_scan.c
)
register_test_target(test_runner_vofa_scan)

# --- Target: Executor command tests ---
add_executable(test_runner_executor
    test_executor.c
    ${PROJ_ROOT}/Src/COMM/executor/executor.c
)
register_test_target(test_runner_executor)

# --- Target: Protocol manager tests ---
add_executable(test_runner_manager
    test_manager.c
    ${PROJ_ROOT}/Src/COMM/manager/manager.c
)
register_test_target(test_runner_manager)

# Note: Full motor/fsm tests require more extensive mocking
# and are better run on target hardware or with a full simulation environment
