cmake_minimum_required(VERSION 3.20)

# Project setup
project(VectorFoc C CXX ASM)

# Toolchain setup is expected to be passed via -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

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
else()
    # Default version if no git tag
    set(FW_VERSION_MAJOR 0)
    set(FW_VERSION_MINOR 0)
    set(FW_VERSION_PATCH 0)
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

# Build types
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

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
    Src/HAL/interface
    Src/HAL/stm32g4
    Src/HAL/bsp
    Src/HAL/encoder
    Src/ALGO
    Src/ALGO/motor
    Src/ALGO/control
    Src/ALGO/foc
    Src/ALGO/pid
    Src/ALGO/observer
    Src/ALGO/trajectory
    Src/COMM
    Src/COMM/manager
    Src/COMM/executor
    Src/COMM/protocol
    Src/COMM/protocol/canopen
    Src/COMM/protocol/inovxio
    Src/COMM/protocol/mit
    Src/COMM/transport
    Src/UI
    Src/UI/parameter
    Src/UI/led
    Src/UI/vofa
    Src/UI/error
)

# Source files
file(GLOB_RECURSE SOURCES
    "Lib/Core/Src/*.c"
    "Lib/Drivers/STM32G4xx_HAL_Driver/Src/*.c"
    "Lib/Middlewares/Third_Party/FreeRTOS/Source/*.c"
    "Lib/USB_Device/*.c"
    "Lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c"
    "Lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c"
    "Src/APP/*.c"
    "Src/HAL/*.c"
    "Src/ALGO/*.c"
    "Src/COMM/*.c"
    "Src/UI/*.c"
)

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
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/Lib/stm32g431xx_flash.ld")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${CMAKE_PROJECT_NAME}.map")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs --specs=nosys.specs")

# Executable
add_executable(${CMAKE_PROJECT_NAME}.elf ${SOURCES})

# Hex and Bin generation
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Building ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)
