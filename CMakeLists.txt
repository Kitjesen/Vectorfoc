cmake_minimum_required(VERSION 3.20)

option(BOOTLOADER_BUILD "Build the 16 KiB USB bootloader instead of the app" OFF)
if(BOOTLOADER_BUILD)
    include("${CMAKE_CURRENT_LIST_DIR}/CMakeLists_Bootloader.txt")
    return()
endif()

# Project setup
project(VectorFoc C CXX ASM)

# Toolchain setup is expected to be passed via -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

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

# Pass version to compiler
add_definitions(
    -DFW_VERSION_MAJOR=${FW_VERSION_MAJOR}
    -DFW_VERSION_MINOR=${FW_VERSION_MINOR}
    -DFW_VERSION_PATCH=${FW_VERSION_PATCH}
    -DFW_GIT_HASH="${GIT_HASH}"
    -DFW_GIT_DIRTY=${GIT_DIRTY}
)

# MCU specific definitions
set(MCU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
add_definitions(-DSTM32G431xx -DUSE_HAL_DRIVER -DSTM32G4xx)
# Keep CMSIS SystemCoreClockUpdate and HAL clock calculations aligned with the
# VectorFOC board's physical HSE crystal.
add_compile_definitions(HSE_VALUE=8000000U)

# ---- Board selection ----
option(BOARD_XSTAR "Build firmware for X-STAR-S STM32G431RBT6 board" OFF)
set(POSITION_SENSOR "AUTO" CACHE STRING
    "Position sensor: AUTO, MT6816, TMR3109, HALL, or ABZ")
set_property(CACHE POSITION_SENSOR PROPERTY STRINGS
    AUTO MT6816 TMR3109 HALL ABZ)
string(TOUPPER "${POSITION_SENSOR}" POSITION_SENSOR_NORMALIZED)

if(BOARD_XSTAR)
    add_definitions(-DBOARD_XSTAR)
    set(TARGET_BOARD_NAME "X-STAR-S STM32G431RBT6")
    if(POSITION_SENSOR_NORMALIZED STREQUAL "AUTO")
        set(RESOLVED_POSITION_SENSOR "HALL")
    elseif(POSITION_SENSOR_NORMALIZED STREQUAL "HALL" OR
           POSITION_SENSOR_NORMALIZED STREQUAL "ABZ")
        set(RESOLVED_POSITION_SENSOR "${POSITION_SENSOR_NORMALIZED}")
    else()
        message(FATAL_ERROR
            "X-STAR supports POSITION_SENSOR=AUTO, HALL, or ABZ; got ${POSITION_SENSOR}")
    endif()
else()
    set(TARGET_BOARD_NAME "VectorFOC STM32G431CBU6")
    if(POSITION_SENSOR_NORMALIZED STREQUAL "AUTO")
        set(RESOLVED_POSITION_SENSOR "MT6816")
    elseif(POSITION_SENSOR_NORMALIZED STREQUAL "MT6816" OR
           POSITION_SENSOR_NORMALIZED STREQUAL "TMR3109")
        set(RESOLVED_POSITION_SENSOR "${POSITION_SENSOR_NORMALIZED}")
    else()
        message(FATAL_ERROR
            "VectorFOC supports POSITION_SENSOR=AUTO, MT6816, or TMR3109; got ${POSITION_SENSOR}")
    endif()
endif()

if(RESOLVED_POSITION_SENSOR STREQUAL "MT6816")
    add_compile_definitions(HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_MT6816)
elseif(RESOLVED_POSITION_SENSOR STREQUAL "TMR3109")
    add_compile_definitions(HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_TMR3109)
elseif(RESOLVED_POSITION_SENSOR STREQUAL "HALL")
    add_compile_definitions(HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_HALL)
elseif(RESOLVED_POSITION_SENSOR STREQUAL "ABZ")
    add_compile_definitions(HW_POSITION_SENSOR_MODE=HW_POSITION_SENSOR_ABZ)
endif()

message(STATUS "Target board: ${TARGET_BOARD_NAME}")
message(STATUS "Position sensor: ${RESOLVED_POSITION_SENSOR}")

# Build types
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -DNDEBUG")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -g")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os")
endif()

# Include directories
include_directories(
    # Lib - 第三方/CubeMX 生成代码
    Lib/Core/Inc
    Lib/Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Lib/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Lib/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Lib/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Lib/Drivers/CMSIS/Include
    Lib/Middlewares/Third_Party/FreeRTOS/Source/include
    Lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Lib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    Lib/USB_Device
    Lib/USB_Device/App
    Lib/USB_Device/Target
    # Src - 自定义代码
    Src/config
    Src/APP
    Src/APP/init
    Src/APP/rtos
    Src/APP/isr
    Src/BOOT
    Src/HAL/interface
    Src/HAL/stm32g4
    Src/HAL/bsp
    Src/HAL/bsp/xstar
    Src/HAL/encoder
    Src/HAL/position_sensor
    Src/ALGO
    Src/ALGO/motor
    Src/ALGO/control
    Src/SAFE
    Src/ALGO/foc
    Src/ALGO/pid
    Src/ALGO/observer
    Src/ALGO/trajectory
    Src/COMM
    Src/COMM/manager
    Src/COMM/executor
    Src/COMM/protocol
    Src/COMM/protocol/canopen
    Src/COMM/protocol/vector
    Src/COMM/protocol/mit
    Src/COMM/transport
    Src/UI
    Src/UI/parameter
    Src/UI/led
    Src/UI/vofa
    Src/UI/error
)

# Source files.  CONFIGURE_DEPENDS lets supported generators discover a new
# source file on the next build instead of linking a stale source list.
set(SOURCE_GLOBS
    "Lib/Core/Src/*.c"
    "Lib/Drivers/STM32G4xx_HAL_Driver/Src/*.c"
    "Lib/Middlewares/Third_Party/FreeRTOS/Source/*.c"
    "Lib/USB_Device/*.c"
    "Lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c"
    "Lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c"
    "Src/APP/*.c"
    "Src/HAL/*.c"
    "Src/ALGO/*.c"
    "Src/SAFE/*.c"
    "Src/COMM/*.c"
    "Src/UI/*.c"
    "Src/BOOT/bootloader.c"
)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.12")
    file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS ${SOURCE_GLOBS})
else()
    file(GLOB_RECURSE SOURCES ${SOURCE_GLOBS})
endif()

list(APPEND SOURCES "Src/APP/isr/encoder_failure_guard.c")

# list(FILTER SOURCES EXCLUDE REGEX "MODULES/hal/.*")

# Exclude template/example files if necessary, or specific files that shouldn't be compiled
# Remove all portable files first
list(FILTER SOURCES EXCLUDE REGEX "Lib/Middlewares/Third_Party/FreeRTOS/Source/portable/.*")

# Manually adding FreeRTOS portable files
list(APPEND SOURCES
    "Lib/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
    "Lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
)

# Startup file
list(APPEND SOURCES "Lib/startup_stm32g431xx.s")

# Linker script
# BOARD_XSTAR: X-STAR-S 无自定义 bootloader，从 0x08000000 直接烧录
# 默认 VectorFOC: 使用 bootloader，app 从 0x08004000 起始
if(BOARD_XSTAR)
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/Lib/stm32g431xx_flash.ld")
    message(STATUS "Linker script: stm32g431xx_flash.ld (standalone, 0x08000000)")
else()
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/Lib/stm32g431xx_app.ld")
    message(STATUS "Linker script: stm32g431xx_app.ld (app with bootloader, 0x08004000)")
endif()

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${CMAKE_PROJECT_NAME}.map")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs --specs=nosys.specs")

# Executable
add_executable(${CMAKE_PROJECT_NAME}.elf ${SOURCES})

# Hex and Bin generation. VectorFOC bootloader images must contain a real
# payload size and CRC in the fixed 0x200 application-header slot.
if(BOARD_XSTAR)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf>" "${CMAKE_PROJECT_NAME}.hex"
        COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf>" "${CMAKE_PROJECT_NAME}.bin"
        COMMENT "Building standalone ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
        VERBATIM
    )
else()
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
endif()
