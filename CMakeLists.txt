cmake_minimum_required(VERSION 3.20)

# Project setup
project(VectorFoc C CXX ASM)

# Toolchain setup is expected to be passed via -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

set(POSITION_SENSOR "MT6816" CACHE STRING "Position sensor: MT6816 or TMR3109")
set_property(CACHE POSITION_SENSOR PROPERTY STRINGS MT6816 TMR3109)
if(NOT POSITION_SENSOR MATCHES "^(MT6816|TMR3109)$")
    message(FATAL_ERROR "POSITION_SENSOR must be MT6816 or TMR3109")
endif()

set(FW_VERSION_FALLBACK "1.0.0" CACHE STRING
    "Fallback firmware version when git tags are unavailable")

# =============================================================================
# Version extraction from Git
# =============================================================================
# Try to get version from git tag (e.g., v1.2.3 -> 1.2.3)
execute_process(
    COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

# Get short git hash
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(NOT GIT_HASH)
    set(GIT_HASH "unknown")
endif()

# Check if working directory is dirty
execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DIRTY_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(GIT_DIRTY_OUTPUT)
    set(GIT_DIRTY 1)
else()
    set(GIT_DIRTY 0)
endif()

# Parse version from tag (v1.2.3 -> MAJOR=1, MINOR=2, PATCH=3)
if(GIT_TAG MATCHES "^v?([0-9]+)\\.([0-9]+)\\.([0-9]+)")
    set(FW_VERSION_MAJOR ${CMAKE_MATCH_1})
    set(FW_VERSION_MINOR ${CMAKE_MATCH_2})
    set(FW_VERSION_PATCH ${CMAKE_MATCH_3})
elseif(FW_VERSION_FALLBACK MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
    set(FW_VERSION_MAJOR ${CMAKE_MATCH_1})
    set(FW_VERSION_MINOR ${CMAKE_MATCH_2})
    set(FW_VERSION_PATCH ${CMAKE_MATCH_3})
    message(WARNING
        "No semver git tag found; using fallback firmware version ${FW_VERSION_FALLBACK}")
else()
    message(FATAL_ERROR
        "FW_VERSION_FALLBACK must use MAJOR.MINOR.PATCH format, got: ${FW_VERSION_FALLBACK}")
endif()

message(STATUS "Firmware Version: ${FW_VERSION_MAJOR}.${FW_VERSION_MINOR}.${FW_VERSION_PATCH}")
message(STATUS "Git Hash: ${GIT_HASH} (dirty: ${GIT_DIRTY})")

# Firmware-only compiler definitions.  The algorithm library below deliberately
# receives none of these MCU or board definitions.
set(FW_VERSION_DEFINITIONS
    "FW_VERSION_MAJOR=${FW_VERSION_MAJOR}"
    "FW_VERSION_MINOR=${FW_VERSION_MINOR}"
    "FW_VERSION_PATCH=${FW_VERSION_PATCH}"
    "FW_GIT_HASH=\"${GIT_HASH}\""
    "FW_GIT_DIRTY=${GIT_DIRTY}"
)

set(MCU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
separate_arguments(MCU_FLAG_LIST NATIVE_COMMAND "${MCU_FLAGS}")

# Build types
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS}")

set(FW_COMPILE_OPTIONS
    ${MCU_FLAG_LIST}
    -Wall
    -fdata-sections
    -ffunction-sections
)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND FW_COMPILE_OPTIONS -Os -g)
else()
    list(APPEND FW_COMPILE_OPTIONS -Os)
endif()

# Include directories
include_directories(
    # platform - 第三方/CubeMX 生成代码
    platform/Core/Inc
    platform/Drivers/STM32G4xx_HAL_Driver/Inc
    platform/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    platform/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    platform/Drivers/CMSIS/Include
    platform/Middlewares/Third_Party/FreeRTOS/Source/include
    platform/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    platform/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    platform/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    platform/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    platform/USB_Device
    platform/USB_Device/App
    platform/USB_Device/Target
    # src - 自定义代码
    src/config
    src/app
    src/boot
    src/hal
    src/hal/bsp
    src/hal/encoder
    src
    src/motor
    src/foc
    src/safety
    src/comm
    ${CMAKE_SOURCE_DIR}
)

# This target is the chip-independent algorithm boundary.  It can be compiled
# by a host compiler or linked into any firmware adapter without STM32/HAL
# headers or board macros.
set(ALGORITHM_SOURCES
    "algorithm/foc_current_loop.c"
    "algorithm/foc_transforms.c"
    "algorithm/field_weakening.c"
    "algorithm/torque_feedforward.c"
    "algorithm/ladrc_controller.c"
    "algorithm/pid_controller.c"
    "algorithm/command_limiter.c"
    "algorithm/sliding_mode_observer.c"
    "algorithm/trapezoid_trajectory.c"
)
add_library(vector_algorithm STATIC ${ALGORITHM_SOURCES})
target_include_directories(vector_algorithm PUBLIC
    ${CMAKE_SOURCE_DIR}/algorithm
)
target_compile_options(vector_algorithm PRIVATE
    -Wall
    -fdata-sections
    -ffunction-sections
)
if(CMAKE_CROSSCOMPILING)
    target_compile_options(vector_algorithm PRIVATE ${MCU_FLAG_LIST} -Os)
endif()

# Source files
file(GLOB_RECURSE SOURCES
    "platform/Core/Src/*.c"
    "platform/Drivers/STM32G4xx_HAL_Driver/Src/*.c"
    "platform/Middlewares/Third_Party/FreeRTOS/Source/*.c"
    "platform/USB_Device/*.c"
    "platform/Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c"
    "platform/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c"
)

# Keep the vendor glob readable while excluding modules that have no caller in
# this application. Their sources are intentionally absent from the checkout.
list(FILTER SOURCES EXCLUDE REGEX ".*stm32g4xx_hal_(crc|crc_ex|rng|exti|dma_ex|spi_ex|flash_ramfunc)\\.c$")
list(FILTER SOURCES EXCLUDE REGEX ".*stm32g4xx_ll_adc\\.c$")
list(FILTER SOURCES EXCLUDE REGEX ".*/(croutine|event_groups|stream_buffer|timers)\\.c$")

# First-party sources: keep this list aligned with module ownership.
list(APPEND SOURCES
    # app
    "src/app/application_header.c"
    "src/app/application_init.c"
    "src/app/foc_isr.c"
    "src/app/status_led.c"
    "src/app/communication_task.c"
    "src/app/diagnostics_task.c"
    "src/app/safety_task.c"
    # foc
    "src/foc/cogging_compensation.c"
    "src/foc/control_dispatcher.c"
    "src/foc/runtime_compensation.c"
    "src/foc/outer_control.c"
    "src/foc/observer_adapter.c"
    # motor
    "src/motor/encoder_calibration.c"
    "src/motor/calibration_state.c"
    "src/motor/current_calibration.c"
    "src/motor/flux_calibration.c"
    "src/motor/drive_state_machine.c"
    "src/motor/motor_runtime.c"
    "src/motor/motor_defaults.c"
    "src/motor/resistance_inductance_calibration.c"
    # safety
    "src/safety/error_manager.c"
    "src/safety/fault_definition.c"
    "src/safety/fault_monitor.c"
    "src/safety/safety_manager.c"
    "src/safety/adc_sample_guard.c"
    "src/safety/encoder_failure_guard.c"
    "src/safety/watchdog_supervisor.c"
    # comm
    "src/comm/protocol_canopen.c"
    "src/comm/command_executor.c"
    "src/comm/protocol_mit.c"
    "src/comm/protocol_dispatcher.c"
    "src/comm/protocol_vector.c"
    "src/comm/telemetry_vofa.c"
    # config
    "src/config/parameter_access.c"
    "src/config/parameter_storage.c"
    "src/config/parameter_registry.c"
    "src/config/encoder_calibration_storage.c"
    "src/config/encoder_calibration_settings.c"
    "src/config/parameter_bindings.c"
    "src/config/runtime_settings.c"
    # hal
    "src/hal/bsp/board_adc.c"
    "src/hal/bsp/board_can.c"
    "src/hal/bsp/board_cycle_counter.c"
    "src/hal/bsp/board_flash.c"
    "src/hal/bsp/board_log.c"
    "src/hal/encoder/mt6816_encoder.c"
    "src/hal/encoder/tmr3109_encoder.c"
    "src/hal/encoder/position_sensor.c"
    "src/hal/encoder/position_sensor_selection.c"
    "src/hal/encoder/position_sensor_mt6816.c"
    "src/hal/encoder/position_sensor_tmr3109.c"
    "src/hal/hardware_interface.c"
    "src/hal/encoder_interface.c"
    "src/hal/pwm_interface.c"
    "src/hal/motor_sampling.c"
    "src/hal/system_calls.c"
    # Application-side boot handoff; boot_main has a separate build.
    "src/boot/bootloader.c"
)

# Exclude template/example files if necessary, or specific files that shouldn't be compiled
# Remove all portable files first
list(FILTER SOURCES EXCLUDE REGEX "platform/Middlewares/Third_Party/FreeRTOS/Source/portable/.*")

# Manually adding FreeRTOS portable files
list(APPEND SOURCES
    "platform/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
    "platform/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
)

# Startup file
list(APPEND SOURCES "platform/startup_stm32g431xx.s")

# Linker script: application starts after the bootloader at 0x08004000.
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/platform/stm32g431xx_app.ld")
message(STATUS "Linker script: stm32g431xx_app.ld (app with bootloader, 0x08004000)")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${CMAKE_PROJECT_NAME}.map")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs --specs=nosys.specs")

# Executable
add_executable(${CMAKE_PROJECT_NAME}.elf ${SOURCES})
target_compile_options(${CMAKE_PROJECT_NAME}.elf PRIVATE ${FW_COMPILE_OPTIONS})
target_compile_definitions(${CMAKE_PROJECT_NAME}.elf PRIVATE
    ${FW_VERSION_DEFINITIONS}
    STM32G431xx
    USE_HAL_DRIVER
    STM32G4xx
    HSE_VALUE=8000000U
    "HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_${POSITION_SENSOR}"
)
target_link_libraries(${CMAKE_PROJECT_NAME}.elf PRIVATE vector_algorithm)

# Package a real payload length and CRC at the fixed application-header slot.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(APP_RAW_BIN "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.raw.bin")
set(APP_HEADER_BIN "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.app_header.bin")
set(APP_PATCHED_ELF "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.patched.elf")
set(APP_VERSION "${FW_VERSION_MAJOR}.${FW_VERSION_MINOR}.${FW_VERSION_PATCH}")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf>" "${APP_RAW_BIN}"
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_SOURCE_DIR}/scripts/patch_app_header.py"
            generate --image "${APP_RAW_BIN}" --header-out "${APP_HEADER_BIN}"
            --version "${APP_VERSION}"
    COMMAND ${CMAKE_OBJCOPY} --update-section ".app_header=${APP_HEADER_BIN}"
            "$<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf>" "${APP_PATCHED_ELF}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${APP_PATCHED_ELF}" "$<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf>"
    COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf>" "${CMAKE_PROJECT_NAME}.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf>" "${CMAKE_PROJECT_NAME}.bin"
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_SOURCE_DIR}/scripts/patch_app_header.py"
            verify --image "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
    COMMENT "Packaging bootloader-compatible ${CMAKE_PROJECT_NAME} image"
    VERBATIM
)
