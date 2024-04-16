#include "gbda-pico.h"

struct gb gb;
bool button[8];

void gbda_set_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2) {
    if (!running_on_fpga()) {
        clock_configure(clk_sys,
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        USB_CLK_KHZ * KHZ,
                        USB_CLK_KHZ * KHZ);

        pll_init(pll_sys, PLL_COMMON_REFDIV, vco_freq, post_div1, post_div2);
        uint32_t freq = vco_freq / (post_div1 * post_div2);

        // Configure clocks
        // CLK_REF is the XOSC source
        clock_configure(clk_ref,
                        CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                        0, // No aux mux
                        XOSC_KHZ * KHZ,
                        XOSC_KHZ * KHZ);

        // CLK SYS = PLL SYS (usually) 125MHz / 1 = 125MHz
        clock_configure(clk_sys,
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                        freq, freq);
    }
}

/* a sucker version of set_sys_clock_khz, only works with gbda-pico. */
static inline bool gbda_set_sys_clock_khz(uint32_t freq_khz, bool required) {
    uint vco, postdiv1, postdiv2;
    if (check_sys_clock_khz(freq_khz, &vco, &postdiv1, &postdiv2)) {
        gbda_set_sys_clock_pll(vco, postdiv1, postdiv2);
        return true;
    } else if (required) {
        panic("System clock of %u kHz cannot be exactly achieved", freq_khz);
        return false;
    }
}

void joypad_callback(uint gpio, uint32_t events)
{
    switch (gpio) {
    case JOYPAD_LEFT_PIN:
        button[JOYPAD_LEFT] = 0;
        break;
    case JOYPAD_RIGHT_PIN:
        button[JOYPAD_RIGHT] = 0;
        break;
    case JOYPAD_UP_PIN:
        button[JOYPAD_UP] = 0;
        break;
    case JOYPAD_DOWN_PIN:
        button[JOYPAD_DOWN] = 0;
        break;
    case JOYPAD_A_PIN:
        button[JOYPAD_A] = 0;
        break;
    case JOYPAD_B_PIN:
        button[JOYPAD_B] = 0;
        break;
    case JOYPAD_SELECT_PIN:
        button[JOYPAD_SELECT] = 0;
        break;
    case JOYPAD_START_PIN:
        button[JOYPAD_START] = 0;
        break;
    default:
        break;
    }
}

static inline void joypad_check_button(int i)
{
    if (!button[i]) {
        gb.joypad.button[i] = 0;
        gb.interrupt.flag |= INTR_SRC_JOYPAD;
    } else {
        gb.joypad.button[i] = 1;
    }
}

void joypad_init(void)
{
    gpio_init_mask((1U << JOYPAD_A_PIN) | (1U << JOYPAD_B_PIN) | (1U << JOYPAD_SELECT_PIN) | (1U << JOYPAD_START_PIN) |
                    (1U << JOYPAD_RIGHT_PIN) | (1U << JOYPAD_UP_PIN) | (1U << JOYPAD_DOWN_PIN) | (1U << JOYPAD_LEFT_PIN));
    gpio_set_dir_in_masked((1U << JOYPAD_A_PIN) | (1U << JOYPAD_B_PIN) | (1U << JOYPAD_SELECT_PIN) | (1U << JOYPAD_START_PIN) |
                    (1U << JOYPAD_RIGHT_PIN) | (1U << JOYPAD_UP_PIN) | (1U << JOYPAD_DOWN_PIN) | (1U << JOYPAD_LEFT_PIN));
    gpio_set_irq_enabled_with_callback(JOYPAD_LEFT_PIN, GPIO_IRQ_EDGE_FALL, true, &joypad_callback);
    gpio_set_irq_enabled_with_callback(JOYPAD_RIGHT_PIN, GPIO_IRQ_EDGE_FALL, true, &joypad_callback);
    gpio_set_irq_enabled_with_callback(JOYPAD_UP_PIN, GPIO_IRQ_EDGE_FALL, true, &joypad_callback);
    gpio_set_irq_enabled_with_callback(JOYPAD_DOWN_PIN, GPIO_IRQ_EDGE_FALL, true, &joypad_callback);
    gpio_set_irq_enabled_with_callback(JOYPAD_A_PIN, GPIO_IRQ_EDGE_FALL, true, &joypad_callback);
    gpio_set_irq_enabled_with_callback(JOYPAD_B_PIN, GPIO_IRQ_EDGE_FALL, true, &joypad_callback);
    gpio_set_irq_enabled_with_callback(JOYPAD_SELECT_PIN, GPIO_IRQ_EDGE_FALL, true, &joypad_callback);
    gpio_set_irq_enabled_with_callback(JOYPAD_START_PIN, GPIO_IRQ_EDGE_FALL, true, &joypad_callback);
}

struct pixel_buffer line_buffer[2];
struct ili9225 ili9225;

void core1_entry(void) 
{
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
                ili9225_draw_scanline(&ili9225, line_buffer[rx.buffer].buffer, SCREEN_WIDTH, PLAINSPI);
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

int main(void) 
{
    command_t tx;

    gbda_set_sys_clock_khz(SYS_FREQ * KHZ, false);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, SYS_FREQ * MHZ, SYS_FREQ * MHZ);
    stdio_init_all();
    sleep_ms(1000);

    joypad_init();
    memset(button, 0, 8);

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
        tx.ly = gb.ppu.ly;
        while (!gb.ppu.scan_line_ready)
            sm83_step(&gb);
        gb.ppu.scan_line_ready = false;
        if (gb.ppu.frame_ready) {
            gb.ppu.frame_ready = false;
            for (int i = 0; i < 8; i++)
                joypad_check_button(i);
            memset(button, 1, 8);
        }
        tx.cmd = CMD_DISPLAY_LINE;
        tx.buffer = gb.buffer_ptr->code;
        multicore_fifo_push_blocking(tx.val);
        __compiler_memory_barrier();
        gb.buffer_ptr = (gb.buffer_ptr->code == 0) ? &line_buffer[1] : &line_buffer[0];
    }
}

