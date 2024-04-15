#include "gbda-pico.h"

struct pixel_buffer line_buffer[2];

struct gb gb;
struct ili9225 ili9225;

void core1_entry() {
    struct pixel_buffer *buffer_ptr;
    command_t rx;

    printf("Hello from core 1\n");

    spi_init(ILI9225_SPI_PORT, 62500 * 1000);
    gpio_set_function(ILI9225_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ILI9225_SDA_PIN, GPIO_FUNC_SPI);
    gpio_init(ILI9225_DC_PIN);
    gpio_set_dir(ILI9225_DC_PIN, GPIO_OUT);
    gpio_init(ILI9225_RST_PIN);
    gpio_set_dir(ILI9225_RST_PIN, GPIO_OUT);
    gpio_init(ILI9225_CS_PIN);
    gpio_set_dir(ILI9225_CS_PIN, GPIO_OUT);
    gpio_set_slew_rate(ILI9225_CLK_PIN, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(ILI9225_SDA_PIN, GPIO_SLEW_RATE_FAST);
    hw_write_masked(&spi_get_hw(spi_default)->cr0, (1 - 1) << SPI_SSPCR0_SCR_LSB, SPI_SSPCR0_SCR_BITS);

    /* LCD init */
    ili9225_init(&ili9225, ILI9225_SPI_PORT, ILI9225_CLK_PIN, ILI9225_SDA_PIN,
                 ILI9225_DC_PIN, ILI9225_RST_PIN, ILI9225_CS_PIN);
    ili9225_set_gram_addr(&ili9225, 15, 30);
    while (1) {
        rx.val = multicore_fifo_pop_blocking();
        switch (rx.cmd) {
        case CMD_DISPLAY_LINE:
            if (rx.ly <= 143) {
                ili9225_set_gram_addr(&ili9225, 153 - rx.ly, 0);
                ili9225_start_drawing(&ili9225);
                spin_lock_unsafe_blocking(line_buffer[rx.buffer].lock);
                ili9225_draw_scanline(&ili9225, line_buffer[rx.buffer].buffer, SCREEN_WIDTH);
                ili9225_stop_drawing(&ili9225);
                spin_unlock_unsafe(line_buffer[rx.buffer].lock);
            }
            break;
        case CMD_RESET:
            break;
        case CMD_START_DMA:
            ili9225_set_gram_addr(&ili9225, 153 - rx.ly, 0);
            ili9225_start_drawing(&ili9225);
            break;
        case CMD_STOP_DMA:
            ili9225_stop_drawing(&ili9225);
            break;
        default:
            break;
        }
    }
}

int main() {
    command_t tx;

    set_sys_clock_khz(270000, true);
    stdio_init_all();
    sleep_ms(1000);

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
        // if (!gb.ppu.ly) {
        //     tx.cmd = CMD_STOP_DMA;
        //     multicore_fifo_push_blocking(tx.val);
        //     tx.cmd = CMD_START_DMA;
        //     multicore_fifo_push_blocking(tx.val);
        // }
        tx.ly = gb.ppu.ly;
        while (!gb.ppu.scan_line_ready)
            sm83_step(&gb);
        gb.ppu.scan_line_ready = false;
        tx.cmd = CMD_DISPLAY_LINE;
        tx.buffer = gb.buffer_ptr->code;
        multicore_fifo_push_blocking(tx.val);
        __compiler_memory_barrier();
        gb.buffer_ptr = (gb.buffer_ptr->code == 0) ? &line_buffer[1] : &line_buffer[0];
    }
}
// int main(void)
// {
//     struct ili9225 ili9225;
//     uint16_t bitmap[SCREEN_WIDTH * SCREEN_HEIGHT];
//     uint64_t start, end, diff;

//     set_sys_clock_khz(270000, true);
//     stdio_init_all();
//     sleep_ms(1000);

//     spi_init(ILI9225_SPI_PORT, 62500 * 1000);
//     gpio_set_function(ILI9225_CLK_PIN, GPIO_FUNC_SPI);
//     gpio_set_function(ILI9225_SDA_PIN, GPIO_FUNC_SPI);
//     gpio_init(ILI9225_DC_PIN);
//     gpio_set_dir(ILI9225_DC_PIN, GPIO_OUT);
//     gpio_init(ILI9225_RST_PIN);
//     gpio_set_dir(ILI9225_RST_PIN, GPIO_OUT);
//     gpio_init(ILI9225_CS_PIN);
//     gpio_set_dir(ILI9225_CS_PIN, GPIO_OUT);
//     gpio_set_slew_rate(ILI9225_CLK_PIN, GPIO_SLEW_RATE_FAST);
//     gpio_set_slew_rate(ILI9225_SDA_PIN, GPIO_SLEW_RATE_FAST);
//     hw_write_masked(&spi_get_hw(spi_default)->cr0, (1 - 1) << SPI_SSPCR0_SCR_LSB, SPI_SSPCR0_SCR_BITS);

//     /* LCD init */
//         for (int j = 0; j < SCREEN_WIDTH * SCREEN_HEIGHT; j++) {
//             bitmap[j] = LIGHTGRAY;
//         }
//     printf("ILI9225 LCD init\n");
//     ili9225_init(&ili9225, ILI9225_SPI_PORT, ILI9225_CLK_PIN, ILI9225_SDA_PIN,
//                  ILI9225_DC_PIN, ILI9225_RST_PIN, ILI9225_CS_PIN);
//     ili9225_set_gram_addr(&ili9225, 0, 0);
//     start = time_us_64();
//     ili9225_start_drawing(&ili9225);
//     for (int i = 0; i < 144; i++) {
//         ili9225_draw_scanline(&ili9225, bitmap, SCREEN_WIDTH);
//         sleep_ms(100);
//     }
//     ili9225_stop_drawing(&ili9225);
//     // ili9225_draw_bitmap(&ili9225, bitmap, SCREEN_WIDTH, SCREEN_HEIGHT);
//     diff = time_us_64() - start;
//     printf("diff: %lld\n", diff);
//     while (1) {
//         tight_loop_contents();
//     }
// }