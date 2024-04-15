#pragma once

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

struct ili9225 {
    spi_inst_t *spiDev;
    uint clkPin;
    uint sdaPin;
    uint dcPin;
    uint rstPin;
    uint csPin;
    uint testPin;
};

typedef enum {
    ENABLE,
    DISABLE
} csState;

typedef enum {
    COMMAND,
    DATA,
} dcState;

#define ILI9225_SPI_PORT                        spi_default
#define ILI9225_CLK_PIN                         PICO_DEFAULT_SPI_SCK_PIN
#define ILI9225_SDA_PIN                         PICO_DEFAULT_SPI_TX_PIN
#define ILI9225_DC_PIN                          20
#define ILI9225_RST_PIN                         21
#define ILI9225_CS_PIN                          PICO_DEFAULT_SPI_CSN_PIN
#define ILI9225_TEST_PIN                        15

#define WHITE          0x9dc2
#define LIGHTGRAY      0x8d42
#define DARKGRAY       0x3306
#define BLACK          0x11c2

#define ILI9225_DRIVER_OUTPUT_CTRL      (0x01u)  // Driver Output Control
#define ILI9225_LCD_AC_DRIVING_CTRL     (0x02u)  // LCD AC Driving Control
#define ILI9225_ENTRY_MODE              (0x03u)  // Entry Mode
#define ILI9225_DISP_CTRL1              (0x07u)  // Display Control 1
#define ILI9225_BLANK_PERIOD_CTRL1      (0x08u)  // Blank Period Control
#define ILI9225_FRAME_CYCLE_CTRL        (0x0Bu)  // Frame Cycle Control
#define ILI9225_INTERFACE_CTRL          (0x0Cu)  // Interface Control
#define ILI9225_OSC_CTRL                (0x0Fu)  // Osc Control
#define ILI9225_POWER_CTRL1             (0x10u)  // Power Control 1
#define ILI9225_POWER_CTRL2             (0x11u)  // Power Control 2
#define ILI9225_POWER_CTRL3             (0x12u)  // Power Control 3
#define ILI9225_POWER_CTRL4             (0x13u)  // Power Control 4
#define ILI9225_POWER_CTRL5             (0x14u)  // Power Control 5
#define ILI9225_VCI_RECYCLING           (0x15u)  // VCI Recycling
#define ILI9225_RAM_ADDR_SET1           (0x20u)  // Horizontal GRAM Address Set
#define ILI9225_RAM_ADDR_SET2           (0x21u)  // Vertical GRAM Address Set
#define ILI9225_GRAM_DATA_REG           (0x22u)  // GRAM Data Register
#define ILI9225_GATE_SCAN_CTRL          (0x30u)  // Gate Scan Control Register
#define ILI9225_VERTICAL_SCROLL_CTRL1   (0x31u)  // Vertical Scroll Control 1 Register
#define ILI9225_VERTICAL_SCROLL_CTRL2   (0x32u)  // Vertical Scroll Control 2 Register
#define ILI9225_VERTICAL_SCROLL_CTRL3   (0x33u)  // Vertical Scroll Control 3 Register
#define ILI9225_PARTIAL_DRIVING_POS1    (0x34u)  // Partial Driving Position 1 Register
#define ILI9225_PARTIAL_DRIVING_POS2    (0x35u)  // Partial Driving Position 2 Register
#define ILI9225_HORIZONTAL_WINDOW_ADDR1 (0x36u)  // Horizontal Address Start Position
#define ILI9225_HORIZONTAL_WINDOW_ADDR2 (0x37u)  // Horizontal Address End Position
#define ILI9225_VERTICAL_WINDOW_ADDR1   (0x38u)  // Vertical Address Start Position
#define ILI9225_VERTICAL_WINDOW_ADDR2   (0x39u)  // Vertical Address End Position
#define ILI9225_GAMMA_CTRL1             (0x50u)  // Gamma Control 1
#define ILI9225_GAMMA_CTRL2             (0x51u)  // Gamma Control 2
#define ILI9225_GAMMA_CTRL3             (0x52u)  // Gamma Control 3
#define ILI9225_GAMMA_CTRL4             (0x53u)  // Gamma Control 4
#define ILI9225_GAMMA_CTRL5             (0x54u)  // Gamma Control 5
#define ILI9225_GAMMA_CTRL6             (0x55u)  // Gamma Control 6
#define ILI9225_GAMMA_CTRL7             (0x56u)  // Gamma Control 7
#define ILI9225_GAMMA_CTRL8             (0x57u)  // Gamma Control 8
#define ILI9225_GAMMA_CTRL9             (0x58u)  // Gamma Control 9
#define ILI9225_GAMMA_CTRL10            (0x59u)  // Gamma Control 10

void ili9225_sleep_ms(uint16_t ms);
void ili9225_set_cs(struct ili9225 *ili9225, csState state);
void ili9225_set_rst(struct ili9225 *ili9225, int state);
void ili9225_set_dc(struct ili9225 *ili9225, dcState state);
void ili9225_write_16(struct ili9225 *ili9225, uint16_t val);
void ili9225_write_command(struct ili9225 *ili9225, uint16_t cmd, uint16_t val);
void ili9225_set_window(struct ili9225 *ili9225, uint verticalStart, uint verticalEnd, uint horizontalStart, uint horizontalEnd);
void ili9225_set_gram_addr(struct ili9225 *ili9225, uint horizontal, uint vertical);
void ili9225_start_drawing(struct ili9225 *ili9225);
void ili9225_stop_drawing(struct ili9225 *ili9225);
void ili9225_draw_scanline(struct ili9225 *ili9225, uint16_t *scanline, uint width);
void ili9225_draw_bitmap(struct ili9225 *ili9225, uint16_t *bitmap, uint width, uint height);
void ili9225_init(struct ili9225 *ili9225, spi_inst_t *spiDev, uint clkPin, uint sdaPin, uint dcPin, uint rstPin, uint csPin);