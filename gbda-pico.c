#include "gbda-pico.h"

struct pixel_buffer line_buffer[2];

struct gb gb;
struct ili9341 ili9341;

void core1_entry() {
    struct pixel_buffer *buffer_ptr;
    command_t rx;

    printf("Hello from core 1\n");

    // configure the buffer ptr, point it to the second buffer
//    buffer_ptr = &line_buffer[0];

    spi_init(ILI9341_SPI_PORT, 62500 * 1000);
    spi_set_format(ili9341.spidev, 8, 0, 0, SPI_MSB_FIRST);
    gpio_set_function(ILI9341_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ILI9341_SDA_PIN, GPIO_FUNC_SPI);
    gpio_init(ILI9341_DC_PIN);
    gpio_set_dir(ILI9341_DC_PIN, GPIO_OUT);
    gpio_init(ILI9341_RST_PIN);
    gpio_set_dir(ILI9341_RST_PIN, 1);
    gpio_put(ILI9341_RST_PIN, 1);
    gpio_init(ILI9341_CS_PIN);
    gpio_set_dir(ILI9341_CS_PIN, GPIO_OUT);
    gpio_put(ILI9341_CS_PIN, 1);
    gpio_set_slew_rate(ILI9341_CLK_PIN, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(ILI9341_SDA_PIN, GPIO_SLEW_RATE_FAST);
    hw_write_masked(&spi_get_hw(spi_default)->cr0, (1 - 1) << SPI_SSPCR0_SCR_LSB, SPI_SSPCR0_SCR_BITS);

    //grab a unused DMA channel and configure it
    ili9341.spi_dma = dma_claim_unused_channel(true);
    ili9341.spi_dma_config = dma_channel_get_default_config(ili9341.spi_dma);
    channel_config_set_transfer_data_size(&ili9341.spi_dma_config, DMA_SIZE_16);
    channel_config_set_write_increment(&ili9341.spi_dma_config, false);
    channel_config_set_read_increment(&ili9341.spi_dma_config, true);
    channel_config_set_dreq(&ili9341.spi_dma_config, spi_get_dreq(ILI9341_SPI_PORT, true));
    printf("dma channel configured from core1\n");

    // /* LCD init */
    ili9341_init(&ili9341, ILI9341_SPI_PORT, ILI9341_CLK_PIN, ILI9341_SDA_PIN,
                 ILI9341_CS_PIN, ILI9341_RST_PIN, ILI9341_DC_PIN);    
    printf("LCD inited from core1\n");
    ili9341_set_display_region(&ili9341, 0, SCREEN_HEIGHT - 1, 0, SCREEN_WIDTH - 1);
    printf("LCD set display region from core1\n");
    while (1) {
        rx.val = multicore_fifo_pop_blocking();
        switch (rx.cmd) {
        case CMD_DISPLAY_LINE:
            if (rx.ly <= 143) {
                spin_lock_unsafe_blocking(line_buffer[rx.buffer].lock);
                ili9341_draw_scanline_dma(&ili9341, line_buffer[rx.buffer].buffer);
                spin_unlock_unsafe(line_buffer[rx.buffer].lock);
            }
            break;
        case CMD_RESET:
            break;
        case CMD_START_DMA:
            ili9341_start_dma_transfer(&ili9341);
            break;
        case CMD_STOP_DMA:
            ili9341_stop_dma_transfer(&ili9341);
            break;
        default:
            break;
        }
    }
}

int main() {
    command_t tx;
    uint32_t data;

    set_sys_clock_khz(270000, true);
    stdio_init_all();
    sleep_ms(1000);

    gpio_init(15);
    gpio_set_dir(15, 1);
    gpio_put(15, 1);

    // init line_buffer[0]
    line_buffer[0].lock_num = spin_lock_claim_unused(true);
    line_buffer[0].lock = spin_lock_init(line_buffer[0].lock_num);
    for (int i = 0; i < SCREEN_WIDTH; i++)
        line_buffer[0].buffer[i] = COLOR_BLACK;
    line_buffer[0].code = 0;

    // init line_buffer[1]
    line_buffer[1].lock_num = spin_lock_claim_unused(true);
    line_buffer[1].lock = spin_lock_init(line_buffer[1].lock_num);
    for (int i = 0; i < SCREEN_WIDTH; i++)
        line_buffer[1].buffer[i] = COLOR_BLACK;
    line_buffer[1].code = 1;

    gb.buffer_ptr = &line_buffer[0];
    multicore_launch_core1(core1_entry);

    sm83_init(&gb);
    cartridge_load(&gb, tetris, tetris_rom_size);
    load_state_after_booting(&gb);
    while (1) {
        if (!gb.ppu.ly) {
            tx.cmd = CMD_STOP_DMA;
            multicore_fifo_push_blocking(tx.val);
            tx.cmd = CMD_START_DMA;
            multicore_fifo_push_blocking(tx.val);
        }
        gpio_put(15, 0);
        while (!gb.ppu.scan_line_ready)
            sm83_step(&gb);
        gpio_put(15, 1);
        gb.ppu.scan_line_ready = false;
        gb.buffer_ptr = (gb.buffer_ptr->code == 0) ? &line_buffer[1] : &line_buffer[0];
        tx.cmd = CMD_DISPLAY_LINE;
        tx.ly = (!gb.ppu.ly) ? 153 : gb.ppu.ly - 1;
        tx.buffer = gb.buffer_ptr->code;
        multicore_fifo_push_blocking(tx.val);
    }
}

// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"
// #include "hardware/dma.h"
// #include "hardware/sync.h"
// #include "hardware/spi.h"
// #include "hardware/timer.h"
// #include "hardware/clocks.h"
// #include "pico/multicore.h"
// #include "ili9341.h"
// #include "tetris.h"
// #include "gbda-pico.h"

// enum CMD {
//     CMD_RESET,
//     CMD_DISPLAY_LINE,
//     CMD_START_DMA,
//     CMD_STOP_DMA,
// };

// struct gb gb;
// struct ili9341 ili9341;
// // uint lock_num;
// // spin_lock_t *lock;

// // #define LED_PIN     25

// typedef union command {
//     uint32_t val;
//     struct {
//         uint8_t cmd;
//         uint8_t ly;
//         uint8_t buffer;
//         uint8_t unused;
//     };
// } command_t; 

// void core1_entry() {
//     command_t rx;

//     printf("Hello from core 1\n");

//     spi_init(ILI9341_SPI_PORT, 62500 * 1000);
//     spi_set_format(ili9341.spidev, 8, 0, 0, SPI_MSB_FIRST);
//     gpio_set_function(ILI9341_CLK_PIN, GPIO_FUNC_SPI);
//     gpio_set_function(ILI9341_SDA_PIN, GPIO_FUNC_SPI);
//     gpio_init(ILI9341_DC_PIN);
//     gpio_set_dir(ILI9341_DC_PIN, GPIO_OUT);
//     gpio_init(ILI9341_RST_PIN);
//     gpio_set_dir(ILI9341_RST_PIN, 1);
//     gpio_put(ILI9341_RST_PIN, 1);
//     gpio_init(ILI9341_CS_PIN);
//     gpio_set_dir(ILI9341_CS_PIN, GPIO_OUT);
//     gpio_put(ILI9341_CS_PIN, 1);
//     gpio_set_slew_rate(ILI9341_CLK_PIN, GPIO_SLEW_RATE_FAST);
//     gpio_set_slew_rate(ILI9341_SDA_PIN, GPIO_SLEW_RATE_FAST);
//     hw_write_masked(&spi_get_hw(spi_default)->cr0, (1 - 1) << SPI_SSPCR0_SCR_LSB, SPI_SSPCR0_SCR_BITS);

//     //grab a unused DMA channel and configure it
//     ili9341.spi_dma = dma_claim_unused_channel(true);
//     ili9341.spi_dma_config = dma_channel_get_default_config(ili9341.spi_dma);
//     channel_config_set_transfer_data_size(&ili9341.spi_dma_config, DMA_SIZE_16);
//     channel_config_set_write_increment(&ili9341.spi_dma_config, false);
//     channel_config_set_read_increment(&ili9341.spi_dma_config, true);
//     channel_config_set_dreq(&ili9341.spi_dma_config, spi_get_dreq(ILI9341_SPI_PORT, true));
//     printf("dma channel configured from core1\n");

//     // /* LCD init */
//     ili9341_init(&ili9341, ILI9341_SPI_PORT, ILI9341_CLK_PIN, ILI9341_SDA_PIN,
//                  ILI9341_CS_PIN, ILI9341_RST_PIN, ILI9341_DC_PIN);    
//     printf("LCD inited from core1\n");
//     ili9341_set_display_region(&ili9341, 0, SCREEN_HEIGHT - 1, 0, SCREEN_WIDTH - 1);
//     printf("LCD set display region from core1\n");
//     while (1) {
//         rx.val = multicore_fifo_pop_blocking();
//         switch (rx.cmd) {
//         case CMD_DISPLAY_LINE:
//             ili9341_draw_bitmap_dma(&ili9341, gb.frame_buffer);
//             break;
//         case CMD_RESET:
//             break;
//         case CMD_START_DMA:
//             ili9341_start_dma_transfer(&ili9341);
//             break;
//         case CMD_STOP_DMA:
//             ili9341_stop_dma_transfer(&ili9341);
//             break;
//         default:
//             break;
//         }
//     }
// }

// int main() {
//     command_t tx;
//     uint32_t data;
//     uint64_t start, end;
//     uint32_t fps;

//     set_sys_clock_khz(270000, true);
//     stdio_init_all();
//     sleep_ms(1000);

//     gpio_init(15);
//     gpio_set_dir(15, 1);
//     gpio_put(15, 1);
//     printf("Hello, multicore!\n");

//     multicore_launch_core1(core1_entry);

//     sm83_init(&gb);
//     cartridge_load(&gb, tetris, tetris_rom_size);
//     load_state_after_booting(&gb);
//     while (1) {
//         if (!gb.ppu.ly) {
//             tx.cmd = CMD_STOP_DMA;
//             multicore_fifo_push_blocking(tx.val);
//             tx.cmd = CMD_START_DMA;
//             multicore_fifo_push_blocking(tx.val);
//         }
//         while (!gb.ppu.frame_ready)
//             sm83_step(&gb);
//         gb.ppu.frame_ready = false;
//         tx.cmd = CMD_DISPLAY_LINE;
//         multicore_fifo_push_blocking(tx.val);
//     }
// }