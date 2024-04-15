#include "ili9225.h"

struct commandAndData {
    uint16_t command;
    uint16_t data;
};

#define ARRAYSIZE(array)    (sizeof(array)/sizeof(array[0]))

void ili9225_set_rst(struct ili9225 *ili9225, int state)
{
    gpio_put(ili9225->rstPin, state);
}

void ili9225_set_cs(struct ili9225 *ili9225, csState state)
{
    gpio_put(ili9225->csPin, state);
}

void ili9225_set_dc(struct ili9225 *ili9225, dcState state)
{
    gpio_put(ili9225->dcPin, state);
}

void ili9225_write_16(struct ili9225 *ili9225, uint16_t val)
{
    spi_write16_blocking(ili9225->spiDev, (uint16_t *)&val, 1);
}

void ili9225_sleep_ms(uint16_t ms)
{
    sleep_ms(ms);
}

void ili9225_write_register(struct ili9225 *ili9225, uint16_t reg)
{
    ili9225_set_cs(ili9225, ENABLE);
    ili9225_set_dc(ili9225, COMMAND);
    ili9225_write_16(ili9225, reg);
    ili9225_set_cs(ili9225, DISABLE);
}

void ili9225_write_command(struct ili9225 *ili9225, uint16_t cmd, uint16_t val)
{
    ili9225_set_cs(ili9225, ENABLE);
    ili9225_set_dc(ili9225, COMMAND);
    ili9225_write_16(ili9225, cmd);
    ili9225_set_dc(ili9225, DATA);
    ili9225_write_16(ili9225, val);
    ili9225_set_cs(ili9225, DISABLE);
}

void ili9225_set_window(struct ili9225 *ili9225, uint verticalStart, uint verticalEnd, uint horizontalStart, uint horizontalEnd)
{
    ili9225_write_command(ili9225, ILI9225_HORIZONTAL_WINDOW_ADDR2, horizontalStart);
    ili9225_write_command(ili9225, ILI9225_HORIZONTAL_WINDOW_ADDR1, horizontalEnd);
    ili9225_write_command(ili9225, ILI9225_VERTICAL_WINDOW_ADDR2, verticalStart);
    ili9225_write_command(ili9225, ILI9225_VERTICAL_WINDOW_ADDR1, verticalEnd);
}

void ili9225_set_gram_addr(struct ili9225 *ili9225, uint horizontal, uint vertical)
{
    ili9225_write_command(ili9225, ILI9225_RAM_ADDR_SET1, horizontal);
    ili9225_write_command(ili9225, ILI9225_RAM_ADDR_SET2, vertical);
}

void ili9225_start_drawing(struct ili9225 *ili9225)
{
    ili9225_write_register(ili9225, ILI9225_GRAM_DATA_REG);
    ili9225_set_cs(ili9225, ENABLE);
    ili9225_set_dc(ili9225, DATA);
}

void ili9225_stop_drawing(struct ili9225 *ili9225)
{
    ili9225_set_cs(ili9225, DISABLE);
}

void ili9225_draw_bitmap(struct ili9225 *ili9225, uint16_t *bitmap, uint width, uint height)
{
    ili9225_write_register(ili9225, ILI9225_GRAM_DATA_REG);
    ili9225_set_cs(ili9225, ENABLE);
    ili9225_set_dc(ili9225, DATA);
    spi_write16_blocking(ili9225->spiDev, bitmap, width * height);
    ili9225_set_cs(ili9225, DISABLE);
}

void ili9225_draw_scanline(struct ili9225 *ili9225, uint16_t *scanline, uint width)
{
    spi_write16_blocking(ili9225->spiDev, scanline, 160);
}

/* ALl init codes are taken from https://github.com/deltabeard/RP2040-GB/blob/master/src/mk_ili9225.c */
void ili9225_init(struct ili9225 *ili9225, spi_inst_t *spiDev, uint clkPin, uint sdaPin, uint dcPin, uint rstPin, uint csPin)
{
    ili9225->spiDev = spiDev;
    ili9225->clkPin = clkPin;
    ili9225->sdaPin = sdaPin;
    ili9225->dcPin = dcPin;
    ili9225->rstPin = rstPin;
    ili9225->csPin = csPin;

    spi_set_format(ili9225->spiDev, 16, 0, 0, SPI_MSB_FIRST);

    ili9225_set_rst(ili9225, DISABLE);
    ili9225_set_cs(ili9225, DISABLE);
    ili9225_set_dc(ili9225, COMMAND);
    ili9225_sleep_ms(1);

    ili9225_set_rst(ili9225, ENABLE);
    ili9225_sleep_ms(10);
    ili9225_set_rst(ili9225, DISABLE);
    ili9225_sleep_ms(50);

    {
        struct commandAndData commands[] = {
            {ILI9225_POWER_CTRL1, 0x0000},
            {ILI9225_POWER_CTRL2, 0x0000},
            {ILI9225_POWER_CTRL3, 0x0000},
            {ILI9225_POWER_CTRL4, 0x0000},
            {ILI9225_POWER_CTRL5, 0x0000},
        };
        for (int i = 0; i < 5; i++)
            ili9225_write_command(ili9225, commands[i].command, commands[i].data);
        ili9225_sleep_ms(40);
    }

    {
        struct commandAndData commands[] = {
            {ILI9225_POWER_CTRL2, 0x0018},
            {ILI9225_POWER_CTRL3, 0x6121},
            {ILI9225_POWER_CTRL4, 0x006f},
            {ILI9225_POWER_CTRL5, 0x495f},
            {ILI9225_POWER_CTRL1, 0x0800},
        };
        for (int i = 0; i < 5; i++)
            ili9225_write_command(ili9225, commands[i].command, commands[i].data);
        ili9225_sleep_ms(10);
    }

    ili9225_write_command(ili9225, ILI9225_POWER_CTRL2, 0x103b);
    ili9225_sleep_ms(50);

    {
        struct commandAndData commands[] = {
			{ILI9225_DRIVER_OUTPUT_CTRL,	0x011C},
			/* Set LCD inversion to disabled. */
			{ILI9225_LCD_AC_DRIVING_CTRL, 0x0100},
			/* Increment vertical and horizontal address.
			 * Use vertical image. */
			{ILI9225_ENTRY_MODE, 0x1028},
			/* Turn off all display outputs. */
			{ILI9225_DISP_CTRL1,	0x0000},
			/* Set porches to 8 lines. */
			{ILI9225_BLANK_PERIOD_CTRL1, 0x0808},
			/* Use 1-clock delay to gate output and edge. */
			{ILI9225_FRAME_CYCLE_CTRL, 0x1100},
			/* Ignore RGB interface settings. */
			{ILI9225_INTERFACE_CTRL, 0x0000},
			/* Set oscillation frequency to 266.6 kHz. */
			{ILI9225_OSC_CTRL, 0x0701},
			/* Set VCI recycling to 2 clocks. */
			{ILI9225_VCI_RECYCLING, 0x0020},
			/* Initialise RAM Address to 0x0 px. */
			{ILI9225_RAM_ADDR_SET1, 0x0000},
			{ILI9225_RAM_ADDR_SET2, 0x0000},

			/* Set scanning position to full screen. */
			{ILI9225_GATE_SCAN_CTRL,	0x0000},
			/* Set end scan position to 219 + 1 px (0xDB). */
			{ILI9225_VERTICAL_SCROLL_CTRL1,	0x00DB},
			/* Set start scan position to 0 px. */
			{ILI9225_VERTICAL_SCROLL_CTRL2,	0x0000},
			/* Set scroll step to 0 px (no scrolling). */
			{ILI9225_VERTICAL_SCROLL_CTRL3,	0x0000},
			/* Set partial screen driving end to 219 + 1 px
			 * (0xDB). */
			{ILI9225_PARTIAL_DRIVING_POS1, 0x00DB},
			/* Set partial screen driving start to 0 px. */
			{ILI9225_PARTIAL_DRIVING_POS2, 0x0000},
			/* Set window to 176 x 220 px (full screen). */
			// {ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF},
			// {ILI9225_HORIZONTAL_WINDOW_ADDR2,	0x0000},
			// {ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB},
			// {ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000},
			{ILI9225_HORIZONTAL_WINDOW_ADDR1, 153},
			{ILI9225_HORIZONTAL_WINDOW_ADDR2,	9},
			{ILI9225_VERTICAL_WINDOW_ADDR1, 158},
			{ILI9225_VERTICAL_WINDOW_ADDR2, 0},
			/* Gamma curve data. */
			{ILI9225_GAMMA_CTRL1, 0x0000},
			{ILI9225_GAMMA_CTRL2, 0x0808},
			{ILI9225_GAMMA_CTRL3, 0x080A},
			{ILI9225_GAMMA_CTRL4, 0x000A},
			{ILI9225_GAMMA_CTRL5, 0x0A08},
			{ILI9225_GAMMA_CTRL6, 0x0808},
			{ILI9225_GAMMA_CTRL7, 0x0000},
			{ILI9225_GAMMA_CTRL8, 0x0A00},
			{ILI9225_GAMMA_CTRL9, 0x0710},
			{ILI9225_GAMMA_CTRL10, 0x0710},

			/* Enable full colour display. */
			{ILI9225_DISP_CTRL1, 0x0012}
        };
        for (int i = 0; i < ARRAYSIZE(commands); i++)
            ili9225_write_command(ili9225, commands[i].command, commands[i].data);
        ili9225_sleep_ms(50);
    }

    ili9225_write_command(ili9225, ILI9225_DISP_CTRL1, 0x1017);
    ili9225_sleep_ms(50);
}