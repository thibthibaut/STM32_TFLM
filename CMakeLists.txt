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

add_subdirectory(Middlewares/Third_Party/tflite-micro/)
add_subdirectory(Drivers)

set(APP
  Src/main.cpp
  Src/board.cpp
  Src/stm32h7xx_it.c
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

include_directories(
  Middlewares/Third_Party/tflite-micro/third_party/flatbuffers/include/
  Middlewares/Third_Party/tflite-micro
  Middlewares/Third_Party/third_party/gemmlowp/
  Middlewares/Third_Party/third_party/ruy/
  Middlewares/Third_Party/third_party/cmsis/
  Middlewares/Third_Party/third_party/cmsis/CMSIS/NN/Include
  Middlewares/Third_Party/third_party/cmsis/CMSIS/DSP/Include
  Middlewares/Third_Party/third_party/cmsis/CMSIS/Core/Include
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
    -ffreestanding
    -fno-strict-aliasing
    -fno-exceptions
    -fno-non-call-exceptions
    -fno-common
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Release>:-O3>
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

target_link_libraries(
  ${PROJECT_NAME}
  tflm)

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
