#pragma once

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

#define ILI9341_SPI_PORT            spi_default
#define ILI9341_CLK_PIN             PICO_DEFAULT_SPI_SCK_PIN 
#define ILI9341_SDA_PIN             PICO_DEFAULT_SPI_TX_PIN
#define ILI9341_CS_PIN              PICO_DEFAULT_SPI_CSN_PIN
#define ILI9341_DC_PIN              20
#define ILI9341_RST_PIN             21 

#ifndef MACROS
#define MACROS
#define MSB(x)                      (((x) >> 8) & 0x00ff)
#define LSB(x)                      (((x) >> 0) & 0x00ff)
#endif

// color definitions
#define ILI9341_BLACK 0x0000       //   0,   0,   0
#define ILI9341_NAVY 0x000F        //   0,   0, 123
#define ILI9341_DARKGREEN 0x03E0   //   0, 125,   0
#define ILI9341_DARKCYAN 0x03EF    //   0, 125, 123
#define ILI9341_MAROON 0x7800      // 123,   0,   0
#define ILI9341_PURPLE 0x780F      // 123,   0, 123
#define ILI9341_OLIVE 0x7BE0       // 123, 125,   0
#define ILI9341_LIGHTGREY 0xC618   // 198, 195, 198
#define ILI9341_DARKGREY 0x7BEF    // 123, 125, 123
#define ILI9341_BLUE 0x001F        //   0,   0, 255
#define ILI9341_GREEN 0x07E0       //   0, 255,   0
#define ILI9341_CYAN 0x07FF        //   0, 255, 255
#define ILI9341_RED 0xF800         // 255,   0,   0
#define ILI9341_MAGENTA 0xF81F     // 255,   0, 255
#define ILI9341_YELLOW 0xFFE0      // 255, 255,   0
#define ILI9341_WHITE 0xFFFF       // 255, 255, 255
#define ILI9341_ORANGE 0xFD20      // 255, 165,   0
#define ILI9341_GREENYELLOW 0xAFE5 // 173, 255,  41
#define ILI9341_PINK 0xFC18        // 255, 130, 198

// gbda colors
#define GBDA_WHITE          0x9dc2
#define GBDA_LIGHTGRAY      0x8d42
#define GBDA_DARKGRAY       0x3306
#define GBDA_BLACK          0x11c2

typedef enum {
    CMD,
    DATA,
} ILI9341_DC;

struct ili9341 {
    spi_inst_t *spidev;
    uint clk_pin;
    uint sda_pin;
    uint cs_pin;        /* 0 = enable */
    uint rst_pin;       /* 0 = reset */
    uint dc_pin;        /* 1 = data, 0 = command */
    uint dma_tx;
};

// commands
#define ILI9341_NOP             0x00            /* no operation */
#define ILI9341_SWRESET         0x01            /* software reset */
#define ILI9341_SPLIN           0x10            /* sleep in */
#define ILI9341_SPLOUT          0x11            /* sleep out */
#define ILI9341_PTLON           0x12            /* partial mode on*/
#define ILI9341_NORON           0x13            /* normal mode on */
#define ILI9341_DINVOFF         0x20            /* display inversion off */
#define ILI9341_DINVON          0x21            /* display inversion on */
#define ILI9341_GAMSET          0x26            /* gamma set */
#define ILI9341_DISPOFF         0x28            /* display off */
#define ILI9341_DISPON          0x29            /* display on */
#define ILI9341_CASET           0x2a            /* column address set */
#define ILI9341_PASET           0x2b            /* page address set */
#define ILI9341_RAMWR           0x2c            /* ram write */
#define ILI9341_RGBSET          0x2d            /* color set */
#define ILI9341_PTLAR           0x30            /* partial area */
#define ILI9341_VSCRDEF         0x33            /* vertical scroll definition */
#define ILI9314_TEOFF           0x34            /* tearing off */
#define ILI9314_TEON            0x35            /* tearing on */
#define ILI9341_MADCTL          0x36            /* memory access control */
#define ILI9341_VSCRSADD        0x37            /* vertical scrolling start addr */
#define ILI9341_IDMOFF          0x38            /* idle mode off */
#define ILI9341_IDMON           0x39            /* idle mode on */
#define ILI9341_PIXFMT          0x3a            /* pixel format set */
#define ILI9341_WRMEMCONT       0x3c            /* write memory continue */
#define ILI9341_SETTESCL        0x44            /* set tear scanline */
#define ILI9341_WRDISBV         0x51            /* write display brightness */
#define ILI9341_WRCTRLD         0x53            /* write ctrl display */
#define ILI9341_WRCABC          0x55            
#define ILI9341_RDCABC          0x56
#define ILI9341_FRMCTR1         0xb1            /* frame rate control (normal mode) */
#define ILI9341_FRMCTR2         0xb2            /* frame rate control (idle mode) */
#define ILI9341_FRMCTR3         0xb3            /* frame rate control(partial mode ) */
#define ILI9341_INVCTR          0xb4            /* display inversion control */
#define ILI9341_DFUNCTR         0xb6            /* display function control */
#define ILI9341_PWCTR1          0xc0            /* power control 1 */
#define ILI9341_PWCTR2          0xc1            /* power control 2 */
#define ILI9341_PWCTR3          0xc2            /* power control 3 */
#define ILI9341_PWCTR4          0xc3            /* power control 4 */
#define ILI9341_PWCTR5          0xc4            /* power control 5 */
#define ILI9314_VMCTR1          0xc5            /* VCOM control 1 */
#define ILI9341_VMCTR2          0xc7            /* VCOM control 2 */
#define ILI9341_RDID1           0xda            /* read ID 1 */
#define ILI9341_RDID2           0xdb            /* read ID 2 */
#define ILI9341_RDID3           0xdc            /* read ID 3 */
#define ILI9341_RDID4           0xdd            /* read ID 4 */
#define ILI9341_GMCTRP1         0xe0            /* positive gamma correction */
#define ILI9341_GMCTRN1         0xe0            /* negative gamma correction */

void ili9341_init(struct ili9341 *ili9341, spi_inst_t *spidev, uint clk_pin, uint sda_pin, uint cs_pin, uint rst_pin, uint rs_pin);
void ili9341_write_byte(struct ili9341 *ili9341, uint8_t byte, ILI9341_DC dc);
void ili9341_select(struct ili9341 *ili9341);
void ili9341_deselect(struct ili9341 *ili9341);
void ili9341_reset(struct ili9341 *ili9341);        
void ili9341_write_command(struct ili9341 *ili9341, uint8_t cmd);
void ili9341_write_data(struct ili9341 *ili9341, uint8_t data);
void ili9341_set_display_region(struct ili9341 *ili9341, uint16_t sp, uint16_t ep, uint16_t sc, uint16_t ec);
void ili9341_draw_bitmap_plainspi(struct ili9341 *ili9341, uint16_t *bitmap, int len);
void ili9341_draw_bitmap_dma(struct ili9341 *ili9341, uint16_t *bitmap);