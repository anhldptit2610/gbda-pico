add_library(gbdacore    sm83.c
                        cartridge.c
                        bus.c
                        interrupt.c
                        timer.c
                        serial.c
                        # ppu.c
                        # dma.c
                        # joypad.c
                        mbc.c)
#                        apu.c)

target_link_libraries(gbdacore pico_stdlib)
target_include_directories(gbdacore PUBLIC ${CMAKE_SOURCE_DIR}/gbda/)