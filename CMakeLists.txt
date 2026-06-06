cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED PICO_BOARD)
  set(PICO_BOARD pico2 CACHE STRING "Pico board type")
endif()

if(NOT PICO_SDK_PATH)
  if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
  elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../MASTER_CLOCK/pico-sdk/pico_sdk_init.cmake")
    set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../../MASTER_CLOCK/pico-sdk")
  elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../MASTER_CLOCK/pico-sdk/pico_sdk_init.cmake")
    set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../MASTER_CLOCK/pico-sdk")
  elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../pico-sdk/pico_sdk_init.cmake")
    set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../pico-sdk")
  endif()
endif()

if(NOT EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
  message(FATAL_ERROR
    "pico-sdk not found. Set PICO_SDK_PATH or put pico-sdk in ../../MASTER_CLOCK/pico-sdk")
endif()

include("${PICO_SDK_PATH}/pico_sdk_init.cmake")

project(hrdw_test C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(hrdw_test
  src/hrdw_test.c
  src/hal_io.c
  src/hal_mux_adc.c
  src/clock_input.c
  src/trigger_output.c
  src/calibration.c
  src/app_settings.c
  src/app_presets.c
  src/grids_engine.c
  src/trigseq_engine.c
)

target_include_directories(hrdw_test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(hrdw_test
  pico_stdlib
  hardware_gpio
  hardware_spi
  hardware_i2c
  hardware_adc
  hardware_flash
  hardware_sync
)

pico_enable_stdio_usb(hrdw_test 1)
pico_enable_stdio_uart(hrdw_test 0)

target_compile_definitions(hrdw_test PRIVATE
  PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
  PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
)

pico_add_extra_outputs(hrdw_test)
