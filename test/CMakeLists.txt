cmake_minimum_required(VERSION 3.10)

project(VectorFOC_Test C)

set(CMAKE_C_STANDARD 99)

# Define project root relative to test dir
set(PROJ_ROOT "${CMAKE_SOURCE_DIR}/..")

# Include Directories
include_directories(
    ${CMAKE_SOURCE_DIR}/mocks
    ${PROJ_ROOT}/Lib/Core/Inc
    ${PROJ_ROOT}/Lib/Drivers/STM32G4xx_HAL_Driver/Inc
    ${PROJ_ROOT}/Lib/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${PROJ_ROOT}/Lib/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${PROJ_ROOT}/Lib/Drivers/CMSIS/Include

    ${PROJ_ROOT}/Src/config
    ${PROJ_ROOT}/Src/APP
    ${PROJ_ROOT}/Src/APP/init
    ${PROJ_ROOT}/Src/APP/isr
    ${PROJ_ROOT}/Src/APP/rtos
    ${PROJ_ROOT}/Src/HAL/interface
    ${PROJ_ROOT}/Src/HAL/encoder
    ${PROJ_ROOT}/Src/HAL/bsp
    ${PROJ_ROOT}/Src/HAL/stm32g4
    ${PROJ_ROOT}/Src/ALGO
    ${PROJ_ROOT}/Src/ALGO/foc
    ${PROJ_ROOT}/Src/ALGO/motor
    ${PROJ_ROOT}/Src/ALGO/pid
    ${PROJ_ROOT}/Src/ALGO/control
    ${PROJ_ROOT}/Src/ALGO/observer
    ${PROJ_ROOT}/Src/ALGO/trajectory
    ${PROJ_ROOT}/Src/COMM/manager
    ${PROJ_ROOT}/Src/COMM/executor
    ${PROJ_ROOT}/Src/COMM/transport
    ${PROJ_ROOT}/Src/UI/error
    ${PROJ_ROOT}/Src/UI/parameter
    ${PROJ_ROOT}/Src/UI/vofa
    ${PROJ_ROOT}/Src/UI/led
)

# Mock definitions for test environment
add_definitions(-DTEST_ENV)
add_definitions(-DSTM32G431xx)

# Source Files for Core FOC Algorithm
set(FOC_ALGO_SOURCES
    ${PROJ_ROOT}/Src/ALGO/foc/foc_algorithm.c
    ${PROJ_ROOT}/Src/ALGO/foc/clarke.c
    ${PROJ_ROOT}/Src/ALGO/foc/park.c
    ${PROJ_ROOT}/Src/ALGO/foc/svpwm.c
    ${PROJ_ROOT}/Src/ALGO/foc/trigonometry.c
)

# --- Target: Core Unit Tests ---
add_executable(test_runner_core
    test_foc_core.c
    ${FOC_ALGO_SOURCES}
)

# --- Target: State Machine / Integration Tests ---
set(MOTOR_CORE_SOURCES
    ${FOC_ALGO_SOURCES}
    ${PROJ_ROOT}/Src/ALGO/motor/motor.c
    ${PROJ_ROOT}/Src/ALGO/motor/fsm.c
    ${PROJ_ROOT}/Src/ALGO/pid/pid.c
    mock_hal.c
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
    ${PROJ_ROOT}/Src/ALGO/trajectory/trap_traj.c
)
target_link_libraries(test_runner_traj m)
