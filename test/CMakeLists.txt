cmake_minimum_required(VERSION 3.10)

project(VectorFOC_Test C)

set(CMAKE_C_STANDARD 99)

# Define project root relative to test dir
set(PROJ_ROOT "${CMAKE_SOURCE_DIR}/..")

# Include Directories (Mirroring compiled firmware includes)
include_directories(
    ${CMAKE_SOURCE_DIR}/mocks
    ${PROJ_ROOT}/Core/Inc
    ${PROJ_ROOT}/Drivers/STM32G4xx_HAL_Driver/Inc
    ${PROJ_ROOT}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${PROJ_ROOT}/Drivers/CMSIS/Include

    ${PROJ_ROOT}/APP
    ${PROJ_ROOT}/BSP
    ${PROJ_ROOT}/MODULES/motor

    ${PROJ_ROOT}/MODULES/motor/hal
    ${PROJ_ROOT}/MODULES/algorithm
    ${PROJ_ROOT}/MODULES/communication
)

# Mock Definitions to satisfy strict includes if needed
add_definitions(-DTEST_ENV)
add_definitions(-DSTM32G431xx)
# We might need to mock stm32 header if it is strictly required by some files. 
# For pure algo it shouldn't be.

# Source Files for Core FOC Algorithm
set(FOC_ALGO_SOURCES
    ${PROJ_ROOT}/MODULES/motor/foc/foc_algorithm.c
    ${PROJ_ROOT}/MODULES/motor/foc/clarke.c
    ${PROJ_ROOT}/MODULES/motor/foc/park.c
    ${PROJ_ROOT}/MODULES/motor/foc/svpwm.c
    ${PROJ_ROOT}/MODULES/motor/foc/trigonometry.c
)

# --- Target: Core Unit Tests ---
add_executable(test_runner_core
    test_foc_core.c
    ${FOC_ALGO_SOURCES}
)

# --- Target: State Machine / Integration Tests ---
# Requires more sources (fsm, core motor logic, pid, etc)
set(MOTOR_CORE_SOURCES
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/MODULES/motor/core/motor.c
    ${PROJ_ROOT}/MODULES/motor/fsm/fsm.c
    ${PROJ_ROOT}/MODULES/algorithm/pid.c
    # Mock HAL implementation
    mock_hal.c
    # Motor Plant Simulation
    motor_plant.c
)

add_executable(test_runner_state
    test_foc_state.c
    ${MOTOR_CORE_SOURCES}
)

add_executable(test_runner_closed_loop
    test_foc_closed_loop.c
    ${MOTOR_CORE_SOURCES}
)

# Link math library
target_link_libraries(test_runner_core m)
target_link_libraries(test_runner_state m)
target_link_libraries(test_runner_closed_loop m)

add_executable(test_runner_fault
    test_fault_injection.c
    ${MOTOR_CORE_SOURCES}
)
target_link_libraries(test_runner_fault m)

# --- Target: Trajectory Planner Tests ---
add_executable(test_runner_traj
    test_trap_traj.c
    ${PROJ_ROOT}/MODULES/algorithm/trap_traj.c
)
target_link_libraries(test_runner_traj m)
