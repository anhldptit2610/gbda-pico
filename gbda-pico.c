#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "gbda/gb.h"
#include "ili9341.h"
#include "05-op_rp.gb.h"
#include "tetris.h"
#include "sm83.h"
#include "bus.h"
#include "cartridge.h"
#include <stdio.h>

void gbda_pico_setup(struct gb *gb, struct ili9341 *ili9341)
{
    stdio_init_all();
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
    ili9341->dma_tx = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(ili9341->dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_write_increment(&c, false);
    channel_config_set_read_increment(&c, true);
    channel_config_set_dreq(&c, spi_get_dreq(ILI9341_SPI_PORT, true));
    dma_channel_configure(ili9341->dma_tx, &c,
                        &spi_get_hw(ILI9341_SPI_PORT)->dr,       // write addr
                        gb->frame_buffer,
                        320 * 240 * 2,
                        false);

    /* LCD init */
    ili9341_init(ili9341, spi_default, ILI9341_CLK_PIN, ILI9341_SDA_PIN,
                 ILI9341_CS_PIN, ILI9341_RST_PIN, ILI9341_DC_PIN);    

    /* GameBoy init */
    sm83_init(gb);
    cartridge_load(gb, __05_op_rp_gb, __05_op_rp_gb_len);
    load_state_after_booting(gb);

    ili9341_set_display_region(ili9341, 0, 0x013f, 0, 0x00ef);
    ili9341_draw_bitmap_dma(ili9341, gb->frame_buffer);
    //ili9341_draw_bitmap_plainspi(ili9341, NULL, 0);
}

int main() {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    struct ili9341 ili9341;
    struct gb gb;
    int cycles;
    bool is_interrupt;

    gbda_pico_setup(&gb, &ili9341);
    while(1) {
        // cycles = sm83_step(&gb);
        // is_interrupt = sm83_cycle(&gb, cycles);
        // if (is_interrupt)
        //     sm83_cycle_when_interrupt(&gb);
    }

}