cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(gbda-pico C CXX ASM)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_subdirectory(lib)

add_compile_options(-Wall -O3)

add_executable(gbda-pico gbda-pico.c gbda-pico.h)

#target_include_directories(gbda-pico PRIVATE ${CMAKE_SOURCE_DIR}/lib/ili9341)
target_include_directories(gbda-pico PRIVATE ${CMAKE_SOURCE_DIR}/lib/roms)
target_include_directories(gbda-pico PRIVATE ${CMAKE_SOURCE_DIR}/lib/ili9225)

target_link_libraries(gbda-pico ili9225 m) 
target_link_libraries(gbda-pico pico_stdlib hardware_dma hardware_spi pico_multicore hardware_timer hardware_sync hardware_clocks hardware_gpio)

pico_enable_stdio_uart(gbda-pico 1)

pico_add_extra_outputs(gbda-pico)