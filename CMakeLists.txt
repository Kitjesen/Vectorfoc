cmake_minimum_required(VERSION 3.20)

# Project setup
project(VectorFoc C CXX ASM)

# Toolchain setup is expected to be passed via -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

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
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    APP
    APP/init
    APP/rtos
    APP/isr
    BSP
    BSP/adc
    BSP/can
    BSP/dwt
    BSP/flash
    BSP/log
    BSP/usart
    MODULES
    MODULES/hal
    MODULES/led
    MODULES/algorithm
    MODULES/communication
    MODULES/communication/canopen
    MODULES/communication/inovxio
    MODULES/communication/mit
    MODULES/communication/executor
    MODULES/encoder
    MODULES/encoder/MT6816
    MODULES/motor
    MODULES/motor/api
    MODULES/motor/calibration
    MODULES/motor/control
    MODULES/motor/control/loops
    MODULES/motor/control/modes

    MODULES/motor/core
    MODULES/motor/foc

    MODULES/motor/fsm
    MODULES/motor/hal

    MODULES/motor/safety
    MODULES/parameter
    MODULES/vofa
    MODULES/error
    USB_Device
    USB_Device/App
    USB_Device/Target
)

# Source files
file(GLOB_RECURSE SOURCES
    "Core/Src/*.c"
    "Drivers/STM32G4xx_HAL_Driver/Src/*.c"
    "Middlewares/Third_Party/FreeRTOS/Source/*.c"
    "APP/*.c"
    "BSP/*.c"
    "MODULES/*.c"
    "USB_Device/*.c"
    "Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c"
    "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c"
)

# Exclude problematic files with HAL naming conflicts
list(FILTER SOURCES EXCLUDE REGEX "BSP/bsp_adc_stm32g4.c")
list(FILTER SOURCES EXCLUDE REGEX "BSP/bsp_pwm_stm32g4.c")
list(FILTER SOURCES EXCLUDE REGEX "BSP/bsp_encoder_mt6816.c")
list(FILTER SOURCES EXCLUDE REGEX "MODULES/motor/control/dispatcher.c")
# list(FILTER SOURCES EXCLUDE REGEX "MODULES/hal/.*")

# Exclude template/example files if necessary, or specific files that shouldn't be compiled
# Remove all portable files first
list(FILTER SOURCES EXCLUDE REGEX "Middlewares/Third_Party/FreeRTOS/Source/portable/.*")

# Manually adding FreeRTOS portable files
list(APPEND SOURCES
    "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
    "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
)

# Startup file
list(APPEND SOURCES "startup_stm32g431xx.s")

# Linker script
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/stm32g431xx_flash.ld")

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
