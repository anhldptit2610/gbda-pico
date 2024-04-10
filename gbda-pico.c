#include "gbda-pico.h"

struct pixel_buffer line_buffer[2];

struct gb gb;
struct ili9341 ili9341;

void core1_entry() {
    struct pixel_buffer *buffer_ptr;
    command_t rx;

    printf("Hello from core 1\n");

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

    /* LCD init */
    ili9341_init(&ili9341, ILI9341_SPI_PORT, ILI9341_CLK_PIN, ILI9341_SDA_PIN,
                 ILI9341_CS_PIN, ILI9341_RST_PIN, ILI9341_DC_PIN);    
    ili9341_set_display_region(&ili9341, 0, SCREEN_HEIGHT - 1, 0, SCREEN_WIDTH - 1);

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
        if (!gb.ppu.ly) {
            tx.cmd = CMD_STOP_DMA;
            multicore_fifo_push_blocking(tx.val);
            tx.cmd = CMD_START_DMA;
            multicore_fifo_push_blocking(tx.val);
        }
        while (!gb.ppu.scan_line_ready)
            sm83_step(&gb);
        gb.ppu.scan_line_ready = false;
        tx.cmd = CMD_DISPLAY_LINE;
        tx.ly = (!gb.ppu.ly) ? 153 : gb.ppu.ly - 1;
        tx.buffer = gb.buffer_ptr->code;
        multicore_fifo_push_blocking(tx.val);
        __compiler_memory_barrier();
        gb.buffer_ptr = (gb.buffer_ptr->code == 0) ? &line_buffer[1] : &line_buffer[0];
    }
}