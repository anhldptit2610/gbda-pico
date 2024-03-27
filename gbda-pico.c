#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "gbda/gb.h"
#include "ili9341.h"
#include "tetris.h"
// #include "01-special.h" passed
#include "02-interrupt.h"
// #include "03-op_sp_hl.h" passed
// #include "05-op_rp.gb.h" passed
// #include "06-ld_r_r.h" passed
// #include "07-jr_jp_call_ret_rst.h" passed
// #include "08-misc_instrs.h" passed
// #include "09-op_r_r.h" passed
// #include "10-bit_ops.h" passed
// #include "11-op_a_ihl.h" passed
// #include "04-op_r_imm.h" passed
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

    /* LCD init */
    ili9341_init(ili9341, spi_default, ILI9341_CLK_PIN, ILI9341_SDA_PIN,
                 ILI9341_CS_PIN, ILI9341_RST_PIN, ILI9341_DC_PIN);    
    
    /* GameBoy init */
    sm83_init(gb);
    cartridge_load(gb, __02_interrupts_gb, __02_interrupts_gb_len);
    load_state_after_booting(gb);
}

int main() {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    struct ili9341 ili9341;
    struct gb gb;
    int cycles;
    bool is_interrupt;

    gbda_pico_setup(&gb, &ili9341);
    // for (int i = 0; i < 50; i++) {
    //     cycles = sm83_step(&gb);
    //     sm83_cycle(&gb, cycles);
    // }
    while(1) {
        cycles = sm83_step(&gb);
        is_interrupt = sm83_cycle(&gb, cycles);
        if (is_interrupt)
            sm83_cycle_when_interrupt(&gb);
    }

}