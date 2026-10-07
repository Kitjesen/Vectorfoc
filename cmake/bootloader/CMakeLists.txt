cmake_minimum_required(VERSION 3.20)

# Bootloader Project
project(VectorFoc_Bootloader C CXX ASM)

# This is a standalone CMake source directory under firmware/cmake/.
get_filename_component(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

# MCU specific definitions
set(MCU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
add_definitions(-DSTM32G431xx -DUSE_HAL_DRIVER -DSTM32G4xx -DBOOTLOADER_BUILD)

# Build type - always optimize for size
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -DNDEBUG")
add_compile_definitions(HSE_VALUE=8000000U)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections -Os")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections -Os")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS}")

# Include directories
include_directories(
    # platform - HAL/CMSIS
    ${FIRMWARE_ROOT}/platform/Core/Inc
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Inc
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${FIRMWARE_ROOT}/platform/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${FIRMWARE_ROOT}/platform/Drivers/CMSIS/Include
    # USB
    ${FIRMWARE_ROOT}/platform/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${FIRMWARE_ROOT}/platform/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${FIRMWARE_ROOT}/platform/USB_Device/App
    ${FIRMWARE_ROOT}/platform/USB_Device/Target
    # Bootloader
    ${FIRMWARE_ROOT}/src/boot
    ${FIRMWARE_ROOT}/src/config
    ${FIRMWARE_ROOT}/src/hal
)

# Bootloader source files
set(BOOT_SOURCES
    ${FIRMWARE_ROOT}/src/boot/boot_main.c
    ${FIRMWARE_ROOT}/src/boot/bootloader.c
    ${FIRMWARE_ROOT}/src/boot/flash_ops.c
    ${FIRMWARE_ROOT}/src/boot/boot_protocol.c
)

# HAL sources (minimal set for bootloader)
set(HAL_SOURCES
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    ${FIRMWARE_ROOT}/platform/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
)

# USB sources
set(USB_SOURCES
    ${FIRMWARE_ROOT}/platform/USB_Device/App/usbd_cdc_if.c
    ${FIRMWARE_ROOT}/platform/USB_Device/App/usbd_desc.c
    ${FIRMWARE_ROOT}/platform/USB_Device/App/usb_device.c
    ${FIRMWARE_ROOT}/platform/USB_Device/Target/usbd_conf.c
    ${FIRMWARE_ROOT}/platform/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${FIRMWARE_ROOT}/platform/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${FIRMWARE_ROOT}/platform/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${FIRMWARE_ROOT}/platform/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

# System sources
set(SYSTEM_SOURCES
    ${FIRMWARE_ROOT}/platform/Core/Src/system_stm32g4xx.c
    ${FIRMWARE_ROOT}/platform/Core/Src/stm32g4xx_hal_msp.c
    ${FIRMWARE_ROOT}/src/hal/system_calls.c
)

# Startup file
set(STARTUP_SOURCE "${FIRMWARE_ROOT}/platform/startup_stm32g431xx.s")

# All sources
set(ALL_SOURCES
    ${BOOT_SOURCES}
    ${HAL_SOURCES}
    ${USB_SOURCES}
    ${SYSTEM_SOURCES}
    ${STARTUP_SOURCE}
)

# Linker script
set(LINKER_SCRIPT "${FIRMWARE_ROOT}/platform/stm32g431xx_bootloader.ld")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${CMAKE_PROJECT_NAME}.map")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs --specs=nosys.specs")

# Executable
add_executable(${CMAKE_PROJECT_NAME}.elf ${ALL_SOURCES})

# Hex and Bin generation
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Building ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)

# Print size
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE_UTIL} ${CMAKE_PROJECT_NAME}.elf
    COMMENT "Firmware size:"
)
