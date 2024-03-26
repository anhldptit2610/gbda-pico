#include "serial.h"

void serial_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    switch (addr) {
    case SERIAL_REG_SB:
        gb->serial.sb = val; 
        break;
    case SERIAL_REG_SC:
        gb->serial.sc = val; 
        break;
    default:
        break;
    }
}

uint8_t serial_read(struct gb *gb, uint16_t addr)
{
    uint8_t ret = 0xff;

    switch (addr) {
    case SERIAL_REG_SB:
        ret = gb->serial.sb;
        break;
    case SERIAL_REG_SC:
        ret = gb->serial.sc;
        break;
    default:
        break;
    }
    return ret;
}