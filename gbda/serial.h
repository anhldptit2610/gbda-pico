#pragma once

#include "gb.h"

#define SERIAL_REG_SB           0xff01
#define SERIAL_REG_SC           0xff02

void serial_write(struct gb *gb, uint16_t addr, uint8_t val);
uint8_t serial_read(struct gb *gb, uint16_t addr);

