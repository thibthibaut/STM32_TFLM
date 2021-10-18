cmake_minimum_required(VERSION 3.15.3)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake)

# This is your project statement. You should always list languages; Listing the
# version is nice here since it sets lots of useful variables
project(
  STM32TFLM
  VERSION 0.1
  LANGUAGES ASM C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Drivers & BSPs
set(DRIVERS
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dcmi.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dfsdm.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma2d.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dsi.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_ltdc.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_ltdc_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_nor.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_qspi.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sai.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sai_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sdram.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_fmc.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_sdmmc.c
  Drivers/BSP/Components/is42s32800j/is42s32800j.c
  Drivers/BSP/Components/otm8009a/otm8009a.c
  Drivers/BSP/Components/otm8009a/otm8009a_reg.c
  Drivers/BSP/Components/ov9655/ov9655.c
  Drivers/BSP/Components/ov9655/ov9655_reg.c
  Drivers/BSP/Components/ov5640/ov5640.c
  Drivers/BSP/Components/ov5640/ov5640_reg.c
  Drivers/BSP/STM32H747I-DISCO/stm32h747i_discovery.c
  Drivers/BSP/STM32H747I-DISCO/stm32h747i_discovery_bus.c
  Drivers/BSP/STM32H747I-DISCO/stm32h747i_discovery_camera.c
  Drivers/BSP/STM32H747I-DISCO/stm32h747i_discovery_lcd.c
  Drivers/BSP/STM32H747I-DISCO/stm32h747i_discovery_qspi.c
  Drivers/BSP/STM32H747I-DISCO/stm32h747i_discovery_sd.c
  Drivers/BSP/STM32H747I-DISCO/stm32h747i_discovery_sdram.c
  Drivers/BSP/STM32H747I-DISCO/stm32h747i_discovery_ts.c
)

set(APP
  Src/camera.c
  Src/joystick.c
  Src/lcd.c
  Src/main.c
  Src/qspi.c
  Src/sd.c
  Src/sdram.c
  Src/stm32h7xx_it.c
  Src/touchscreen.c
  startup_stm32h747xx.s
  Utilities/lcd/stm32_lcd.c
  Src/system_stm32h7xx.c
)

add_executable(
  ${PROJECT_NAME}
  ${DRIVERS}
  ${APP}
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE
  -DUSE_HAL_DRIVER
  -DSTM32H747xx
  -DUSE_STM32H747I_DISCO
  -DCORE_CM7
  -DTS_MULTI_TOUCH_SUPPORTED
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  Inc
  Drivers/CMSIS/Device/ST/STM32H7xx/Include
  Drivers/STM32H7xx_HAL_Driver/Inc
  Drivers/BSP/STM32H747I-DISCO
  Drivers/BSP/Components/Common
  Utilities/lcd
  Utilities/Fonts
  Drivers/CMSIS/Include
)


target_compile_options(
  ${PROJECT_NAME}
  PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -fdata-sections
    -ffunction-sections
    -Wall
    -Wextra
    -fno-strict-aliasing
    $<$<CONFIG:Debug>:-O0>
)

target_link_options(
  ${PROJECT_NAME}
  PRIVATE
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -specs=nosys.specs
  -lnosys
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--gc-sections
  -u
  _printf_float
  -Wl,--print-memory-usage
  -T${CMAKE_SOURCE_DIR}/STM32H747XIHx_FLASH_CM7.ld
)

# Print executable size
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND arm-none-eabi-size ${PROJECT_NAME})

# Create hex file
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND arm-none-eabi-objcopy -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex
  COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME} ${PROJECT_NAME}.bin)

# Uncomment to link with a library
#target_link_libraries(
#  ${PROJECT_NAME}
# Add lib here )
