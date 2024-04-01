#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "ili9341.h"
#include "tetris.h"
#include "Dr_Mario.h"
#include "gbda-pico.h"

#include <stdio.h>

/* core1 will deal with graphics rendering while core0
   is in charge of running the emulator 
*/

// enum CORE0_CMD {
//     CORE0_CMD_RESET,
//     CORE0_CMD_DISPLAY_LINE,
// };

struct gb gb;
struct ili9341 ili9341;

// typedef union core0_command {
//     uint32_t val;
//     struct {
//         uint8_t cmd;
//         uint8_t ly;
//         uint16_t unused;
//     };
// } core0_command_t; 

// void main_core1(void)
// {
//     struct ili9341 ili9341;
//     core0_command_t rx;

//     //grab a unused DMA channel and configure it
//     ili9341.dma_tx = dma_claim_unused_channel(true);
//     ili9341.dma_config = dma_channel_get_default_config(ili9341.dma_tx);
//     channel_config_set_transfer_data_size(&ili9341.dma_config, DMA_SIZE_8);
//     channel_config_set_write_increment(&ili9341.dma_config, false);
//     channel_config_set_read_increment(&ili9341.dma_config, true);
//     channel_config_set_dreq(&ili9341.dma_config, spi_get_dreq(ILI9341_SPI_PORT, true));

//     /* LCD init */
//     ili9341_init(&ili9341, spi_default, ILI9341_CLK_PIN, ILI9341_SDA_PIN,
//                  ILI9341_CS_PIN, ILI9341_RST_PIN, ILI9341_DC_PIN);    
//     ili9341_set_display_region(&ili9341, 0, SCREEN_HEIGHT - 1, 0, SCREEN_WIDTH - 1);
//     ili9341_write_command(&ili9341, ILI9341_RAMWR);
//     while (1) {
//         rx.val = multicore_fifo_pop_blocking();
//         if (rx.cmd == CORE0_CMD_DISPLAY_LINE) {
//             ili9341_draw_bitmap_dma(&ili9341, gb.line_buffer);
//             if (rx.ly == 143) {
//                 ili9341_write_command(&ili9341, ILI9341_NOP);
//                 ili9341_write_command(&ili9341, ILI9341_RAMWR);
//             }
//         }
//     }
// }

int main() {
    stdio_init_all();
    set_sys_clock_khz(270000, true);

    spi_init(ILI9341_SPI_PORT, 10000 * 1000);
    gpio_set_function(ILI9341_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ILI9341_SDA_PIN, GPIO_FUNC_SPI);
    gpio_init(ILI9341_DC_PIN);
    gpio_set_dir(ILI9341_DC_PIN, GPIO_OUT);
    gpio_init(ILI9341_RST_PIN);
    gpio_set_dir(ILI9341_RST_PIN, GPIO_OUT);
    gpio_init(ILI9341_CS_PIN);
    gpio_set_dir(ILI9341_CS_PIN, GPIO_OUT);
    gpio_put(ILI9341_CS_PIN, 1);

    //grab a unused DMA channel and configure it
    ili9341.dma_tx = dma_claim_unused_channel(true);
    ili9341.dma_config = dma_channel_get_default_config(ili9341.dma_tx);
    channel_config_set_transfer_data_size(&ili9341.dma_config, DMA_SIZE_8);
    channel_config_set_write_increment(&ili9341.dma_config, false);
    channel_config_set_read_increment(&ili9341.dma_config, true);
    channel_config_set_dreq(&ili9341.dma_config, spi_get_dreq(ILI9341_SPI_PORT, true));

    /* LCD init */
    ili9341_init(&ili9341, spi_default, ILI9341_CLK_PIN, ILI9341_SDA_PIN,
                 ILI9341_CS_PIN, ILI9341_RST_PIN, ILI9341_DC_PIN);    
    ili9341_set_display_region(&ili9341, 0, SCREEN_HEIGHT - 1, 0, SCREEN_WIDTH - 1);

    /* GameBoy init */
    sm83_init(&gb);
    cartridge_load(&gb, tetris, tetris_rom_size);
    load_state_after_booting(&gb);
    while (1) {
        while (!gb.ppu.frame_ready)
            sm83_step(&gb);
        gb.ppu.frame_ready = false;
        ili9341_draw_bitmap_dma(&ili9341, gb.frame_buffer);
    }
}

/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "pico/multicore.h"

// #define FLAG_VALUE 123

// enum CORE0_CMD {
//     CORE0_CMD_RESET,
//     CORE0_CMD_DISPLAY_LINE,
// };

// typedef union core0_command {
//     uint32_t val;
//     struct {
//         uint8_t cmd;
//         uint8_t ly;
//         uint16_t unused;
//     };
// } core0_command_t; 

// void core1_entry() {
//     core0_command_t rx;

//     rx.val = multicore_fifo_pop_blocking();

//     if (rx.cmd == CORE0_CMD_DISPLAY_LINE)
//         printf("displayline\n");
//     else
//         printf("Its all gone not well on core 1!");

//     while (1)
//         tight_loop_contents();
// }

// int main() {
//     core0_command_t tx;

//     stdio_init_all();
//     printf("Hello, multicore!\n");

//     multicore_launch_core1(core1_entry);

//     // Wait for it to start up

//     tx.cmd = CORE0_CMD_DISPLAY_LINE;
//     multicore_fifo_push_blocking(tx.val);

//     while (1) {
//         tight_loop_contents();
//     }
//     /// \end::setup_multicore[]
// }