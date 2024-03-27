#pragma once

#include "gb.h"
#include "bus.h"
#include "interrupt.h"
#include "timer.h"
#include "ppu.h"
#include "apu.h"

int sm83_step(struct gb *gb);
void sm83_init(struct gb *gb);
bool sm83_cycle(struct gb *gb, int cycles);
void sm83_cycle_when_interrupt(struct gb *gb);
void sm83_push_word(struct gb *gb, uint16_t val);