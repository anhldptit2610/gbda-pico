add_library(ili9341 ili9341.c)

target_link_libraries(ili9341 pico_stdlib
                              hardware_spi)