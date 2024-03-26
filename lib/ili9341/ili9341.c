#include "ili9341.h"

void ili9341_select(struct ili9341 *ili9341)
{
    gpio_put(ili9341->cs_pin, 0);
}

void ili9341_deselect(struct ili9341 *ili9341)
{
    gpio_put(ili9341->cs_pin, 1);
}

void ili9341_write_command(struct ili9341 *ili9341, uint8_t cmd)
{
    ili9341_select(ili9341);
    gpio_put(ili9341->dc_pin, CMD);
    spi_write_blocking(ili9341->spidev, (uint8_t *)&cmd, 1);
    ili9341_deselect(ili9341);
}

void ili9341_write_data(struct ili9341 *ili9341, uint8_t data)
{
    ili9341_select(ili9341);
    gpio_put(ili9341->dc_pin, DATA);
    spi_write_blocking(ili9341->spidev, (uint8_t *)&data, 1);
    ili9341_deselect(ili9341);
}

void ili9341_reset(struct ili9341 *ili9341)
{
    gpio_put(ili9341->rst_pin, 0);
    sleep_ms(50);
    gpio_put(ili9341->rst_pin, 1);
    sleep_ms(50);
}

// void ili9341_write_word(struct ili9341 *ili9341, uint16_t word, ILI9341_DC dc)
// {
//     ili9341_select(ili9341);
//     spi_write16_blocking(ili9341->spidev, (uint16_t *)&word, 1);
//     ili9341_deselect(ili9341);
// }
void ili9341_set_display_region(struct ili9341 *ili9341, uint16_t sp, uint16_t ep, uint16_t sc, uint16_t ec)
{
    ili9341_write_command(ili9341, ILI9341_CASET);
    ili9341_write_data(ili9341, MSB(sc));
    ili9341_write_data(ili9341, LSB(sc));
    ili9341_write_data(ili9341, MSB(ec));
    ili9341_write_data(ili9341, LSB(ec));

    ili9341_write_command(ili9341, ILI9341_PASET);
    ili9341_write_data(ili9341, MSB(sp));
    ili9341_write_data(ili9341, LSB(sp));
    ili9341_write_data(ili9341, MSB(ep));
    ili9341_write_data(ili9341, LSB(ep));
}

void ili9341_draw_bitmap(struct ili9341 *ili9341, uint16_t *bitmap, int len)
{
    ili9341_write_command(ili9341, ILI9341_RAMWR);
    for (int i = 0; i < 320; i++) {
        for (int j = 0; j < 240; j++) {
            ili9341_write_data(ili9341, MSB(GBDA_BLACK));
            ili9341_write_data(ili9341, LSB(GBDA_BLACK));
        }
    }
    sleep_ms(100);
    ili9341_write_command(ili9341, ILI9341_NOP);
}

void ili9341_init(struct ili9341 *ili9341, spi_inst_t *spidev, uint clk_pin, uint sda_pin, uint cs_pin, uint rst_pin, uint dc_pin)
{
    ili9341->spidev = spidev;
    ili9341->clk_pin = clk_pin;
    ili9341->cs_pin = cs_pin;
    ili9341->dc_pin = dc_pin;
    ili9341->rst_pin = rst_pin;
    ili9341->sda_pin = sda_pin;

    // do a hardware reset
    ili9341_reset(ili9341);

    // then send a list of commands to the LCD.
    // taken from AdaFruit's ILI9341 driver
    ili9341_write_command(ili9341, 0xef);   // 0xef 0x03 0x80 0x02
    ili9341_write_data(ili9341, 0x03);
    ili9341_write_data(ili9341, 0x80);
    ili9341_write_data(ili9341, 0x02);

    ili9341_write_command(ili9341, 0xcf);   // 0xcf 0x00 0xc1 0x30
    ili9341_write_data(ili9341, 0x00);
    ili9341_write_data(ili9341, 0xc1);
    ili9341_write_data(ili9341, 0x30);

    ili9341_write_command(ili9341, 0xed);   // 0xed 0x64 0x03 0x12 0x82
    ili9341_write_data(ili9341, 0x64);
    ili9341_write_data(ili9341, 0x03);
    ili9341_write_data(ili9341, 0x12);
    ili9341_write_data(ili9341, 0x81);

    ili9341_write_command(ili9341, 0xe8);   // 0xe8 0x85 0x00 0x78
    ili9341_write_data(ili9341, 0x85);
    ili9341_write_data(ili9341, 0x00);
    ili9341_write_data(ili9341, 0x78);

    ili9341_write_command(ili9341, 0xcb);   // 0xcb 0x39 0x2c 0x00 0x34 0x02
    ili9341_write_data(ili9341, 0x39);
    ili9341_write_data(ili9341, 0x2c);
    ili9341_write_data(ili9341, 0x00);
    ili9341_write_data(ili9341, 0x34);
    ili9341_write_data(ili9341, 0x02);

    ili9341_write_command(ili9341, 0xf7);   // 0xf7 0x20
    ili9341_write_data(ili9341, 0x20);

    ili9341_write_command(ili9341, 0xea);   // 0xea 0x00 0x00
    ili9341_write_data(ili9341, 0x00);
    ili9341_write_data(ili9341, 0x00);

    ili9341_write_command(ili9341, ILI9341_PWCTR1);
    ili9341_write_data(ili9341, 0x23);

    ili9341_write_command(ili9341, ILI9341_PWCTR2);
    ili9341_write_data(ili9341, 0x10);

    ili9341_write_command(ili9341, ILI9314_VMCTR1);
    ili9341_write_data(ili9341, 0x3e);
    ili9341_write_data(ili9341, 0x28);

    ili9341_write_command(ili9341, ILI9341_VMCTR2);
    ili9341_write_data(ili9341, 0x86);

    ili9341_write_command(ili9341, ILI9341_MADCTL);
    ili9341_write_data(ili9341, 0x48);

    ili9341_write_command(ili9341, ILI9341_VSCRSADD);
    ili9341_write_data(ili9341, 0x00);

    ili9341_write_command(ili9341, ILI9341_PIXFMT);
    ili9341_write_data(ili9341, 0x55);

    ili9341_write_command(ili9341, ILI9341_FRMCTR1);
    ili9341_write_data(ili9341, 0x00);
    ili9341_write_data(ili9341, 0x18);

    ili9341_write_command(ili9341, ILI9341_DFUNCTR);
    ili9341_write_data(ili9341, 0x08);
    ili9341_write_data(ili9341, 0x82);
    ili9341_write_data(ili9341, 0x27);

    ili9341_write_command(ili9341, 0xf2);
    ili9341_write_data(ili9341, 0x00);

    ili9341_write_command(ili9341, ILI9341_GAMSET);
    ili9341_write_data(ili9341, 0x01);

    ili9341_write_command(ili9341, ILI9341_GMCTRP1);
    ili9341_write_data(ili9341, 0x0f);
    ili9341_write_data(ili9341, 0x31);
    ili9341_write_data(ili9341, 0x2b);
    ili9341_write_data(ili9341, 0x0c);
    ili9341_write_data(ili9341, 0x0e);
    ili9341_write_data(ili9341, 0x08);
    ili9341_write_data(ili9341, 0x4e);
    ili9341_write_data(ili9341, 0xf1);
    ili9341_write_data(ili9341, 0x37);
    ili9341_write_data(ili9341, 0x07);
    ili9341_write_data(ili9341, 0x10);
    ili9341_write_data(ili9341, 0x03);
    ili9341_write_data(ili9341, 0x0e);
    ili9341_write_data(ili9341, 0x09);
    ili9341_write_data(ili9341, 0x00);

    ili9341_write_command(ili9341, ILI9341_GMCTRN1);
    ili9341_write_data(ili9341, 0x00);
    ili9341_write_data(ili9341, 0x0e);
    ili9341_write_data(ili9341, 0x14);
    ili9341_write_data(ili9341, 0x03);
    ili9341_write_data(ili9341, 0x11);
    ili9341_write_data(ili9341, 0x07);
    ili9341_write_data(ili9341, 0x31);
    ili9341_write_data(ili9341, 0xc1);
    ili9341_write_data(ili9341, 0x48);
    ili9341_write_data(ili9341, 0x08);
    ili9341_write_data(ili9341, 0x0f);
    ili9341_write_data(ili9341, 0x0c);
    ili9341_write_data(ili9341, 0x31);
    ili9341_write_data(ili9341, 0x36);
    ili9341_write_data(ili9341, 0x0f);

    ili9341_write_command(ili9341, ILI9341_SPLOUT);

    ili9341_write_command(ili9341, 0x80);

    ili9341_write_command(ili9341, ILI9341_DISPON);

    ili9341_write_command(ili9341, 0x80);

    ili9341_write_command(ili9341, ILI9341_NOP);
}