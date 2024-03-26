#include "bus.h"

enum BUS_REGION {
    ROM = (1U << 0),
    VRAM = (1U << 1),
    ExRAM = (1U << 2),
    WRAM = (1U << 3),
    EcRAM = (1U << 4),
    OAM = (1U << 5),
    UNUSED = (1U << 6),
    IO = (1U << 7),
    HRAM = (1U << 8),
};

bool is_interrupt_reg(uint16_t addr)
{
    return (addr == INTR_REG_IE || addr == INTR_REG_IF);
}

bool is_timer_reg(uint16_t addr)
{
    return (addr == TIM_REG_DIV || addr == TIM_REG_TAC ||
            addr == TIM_REG_TIMA || addr == TIM_REG_TMA);
}

bool is_ppu_reg(uint16_t addr)
{
    return (addr == PPU_REG_LCDC || addr == PPU_REG_STAT || addr == PPU_REG_SCY ||
            addr == PPU_REG_SCX || addr == PPU_REG_LY || addr == PPU_REG_LYC ||
            addr == PPU_REG_BGP || addr == PPU_REG_OBP0 || addr == PPU_REG_OBP1 ||
            addr == PPU_REG_WY || addr == PPU_REG_WX);
}

bool is_apu_reg(uint16_t addr)
{
    return (addr == APU_REG_NR10 || addr == APU_REG_NR11 || addr == APU_REG_NR12 || addr == APU_REG_NR13 || addr == APU_REG_NR14 ||
            addr == APU_REG_NR21 || addr == APU_REG_NR22 || addr == APU_REG_NR23 || addr == APU_REG_NR24 ||
            addr == APU_REG_NR30 || addr == APU_REG_NR31 || addr == APU_REG_NR32 || addr == APU_REG_NR33 || addr == APU_REG_NR34 ||
            addr == APU_REG_NR41 || addr == APU_REG_NR42 || addr == APU_REG_NR43 || addr == APU_REG_NR44 ||
            addr == APU_REG_NR50 || addr == APU_REG_NR51 || addr == APU_REG_NR52);
}

bool is_serial_reg(uint16_t addr)
{
    return (addr == SERIAL_REG_SB || addr == SERIAL_REG_SC);
}

/* write functions */
void vram_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    gb->vram[addr - 0x8000] = val;
    gb->mem[addr] = val;
}

void exram_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    // switch (gb->cart.infos.type) {
    // case MBC1_RAM:
    // case MBC1_RAM_BATTERY:
    //     mbc1_ram_write(gb, addr, val);
    //     break;
    // case MBC3_RAM_BATTERY:
    //     mbc3_write(gb, addr, val);
    //     break;
    // default:
    //     break;
    // }
    gb->extern_ram[addr - 0xa000] = val;
    gb->mem[addr] = val;
}

void wram_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    gb->wram[addr - 0xc000] = val;
    gb->mem[addr] = val;
}

void ecram_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    addr &= 0xddff;
    wram_write(gb, addr, val);
    gb->mem[addr] = val;
}

void oam_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    gb->oam[addr - 0xfe00] = val;
    gb->mem[addr] = val;
}

void unused_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    gb->unused[addr - 0xfea0] = val;
    gb->mem[addr] = val;
}

void io_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    if (is_interrupt_reg(addr))
        interrupt_write(gb, addr, val);
    else if (is_timer_reg(addr))
        timer_write(gb, addr, val);
    // else if (is_serial_reg(addr))
    //     serial_write(gb, addr, val);
    else
    // else if (is_ppu_reg(addr))
    //     ppu_write(gb, addr, val);
    // else if (addr == DMA_REG_DMA)
    //     dma_write(gb, val);
    // else if (addr == JOYPAD_REG_JOYP)
    //     joypad_write(gb, val);
    // else if (is_apu_reg(addr))
    //     apu_regs_write(gb, addr, val);
    // else if (IN_RANGE(addr, 0xff30, 0xff3f))
    //     apu_ram_write(gb, addr, val);
        gb->mem[addr] = val;
}

void hram_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    gb->hram[addr - 0xff80] = val;
    gb->mem[addr] = val;
}

/* read functions */
uint8_t vram_read(struct gb *gb, uint16_t addr)
{
    //return gb->vram[addr - 0x8000];
    return gb->mem[addr];
}

uint8_t exram_read(struct gb *gb, uint16_t addr)
{
    uint8_t ret = 0xff;

    // switch (gb->cart.infos.type) {
    // case MBC1_RAM:
    // case MBC1_RAM_BATTERY:
    //     ret = mbc1_ram_read(gb, addr);
    //     break;
    // case MBC3_RAM_BATTERY:
    //     ret = mbc3_read(gb, addr);
    //     break;
    // default:
    //     ret = 0xff;
    //     break;
    // }
    // return ret;
    //ret = gb->extern_ram[addr - 0xa000];
    ret = gb->mem[addr];
    return ret;
}

uint8_t wram_read(struct gb *gb, uint16_t addr)
{
    //return gb->wram[addr - 0xc000];
    return gb->mem[addr];
}

uint8_t ecram_read(struct gb *gb, uint16_t addr)
{
    addr &= 0xddff;
    return wram_read(gb, addr);
}

uint8_t oam_read(struct gb *gb, uint16_t addr)
{
//    return gb->oam[addr - 0xfe00];
    return gb->mem[addr];
}

uint8_t unused_read(struct gb *gb, uint16_t addr)
{
    return gb->mem[addr];
    //return gb->unused[addr - 0xfea0];
}

uint8_t io_read(struct gb *gb, uint16_t addr)
{
    uint8_t ret = 0xff;

    if (is_interrupt_reg(addr))
        ret = interrupt_read(gb, addr);
    else if (is_timer_reg(addr))
        ret = timer_read(gb, addr);
    //else if (is_serial_reg(addr))
    //    ret = serial_read(gb, addr);
    else 
        ret = gb->mem[addr];
    // else if (is_ppu_reg(addr))
    //     ret = ppu_read(gb, addr);
    // else if (addr == DMA_REG_DMA)
    //     ret = dma_read(gb);
    // else if (addr == JOYPAD_REG_JOYP)
    //     ret = joypad_read(gb);
    // else if (is_apu_reg(addr))
    //     ret = apu_regs_read(gb, addr);
    // else if (IN_RANGE(addr, 0xff30, 0xff3f))
    //     ret = apu_ram_read(gb, addr);
    return ret;
}

uint8_t hram_read(struct gb *gb, uint16_t addr)
{
    return gb->mem[addr];
}

uint16_t bus_get_mem_region(uint16_t addr)
{
    return ((IN_RANGE(addr, 0x0000, 0x7fff) << 0) |
            (IN_RANGE(addr, 0x8000, 0x9fff) << 1) |
            (IN_RANGE(addr, 0xa000, 0xbfff) << 2) |
            (IN_RANGE(addr, 0xc000, 0xdfff) << 3) |
            (IN_RANGE(addr, 0xe000, 0xfdff) << 4) |
            (IN_RANGE(addr, 0xfe00, 0xfe9f) << 5) |
            (IN_RANGE(addr, 0xfea0, 0xfeff) << 6) |
            (IN_RANGE(addr, 0xff00, 0xff7f) << 7) |
            (IN_RANGE(addr, 0xff80, 0xfffe) << 8) |
            ((addr == 0xffff) << 7));
}

void (*write_function[])(struct gb *gb, uint16_t addr, uint8_t val) = {
    [ROM] = rom_write,
    [VRAM] = vram_write,
    [ExRAM] = exram_write,
    [WRAM] = wram_write,
    [EcRAM] = ecram_write,
    [OAM] = oam_write,
    [UNUSED] = unused_write,
    [IO] = io_write,
    [HRAM] = hram_write,
};

uint8_t (*read_function[])(struct gb *gb, uint16_t addr) = {
    [ROM] = rom_read,
    [VRAM] = vram_read,
    [ExRAM] = exram_read,
    [WRAM] = wram_read,
    [EcRAM] = ecram_read,
    [OAM] = oam_read,
    [IO] = io_read,
    [UNUSED] = unused_read,
    [HRAM] = hram_read,
};

uint8_t bus_read(struct gb *gb, uint16_t addr)
{
    uint8_t ret = 0xff;

    if (addr >= 0x0000 && addr <= 0x7fff)
        ret = rom_read(gb, addr);
    else if (addr >= 0x8000 && addr <= 0x9fff)
        ret = vram_read(gb, addr);
    else if (addr >= 0xa000 && addr <= 0xbfff)
        ret = exram_read(gb, addr);
    else if (addr >= 0xc000 && addr <= 0xdfff)
        ret = wram_read(gb, addr);
    else if (addr >= 0xe000 && addr <= 0xfdff)
        ret = wram_read(gb, addr & 0xddff);
    else if (addr >= 0xfe00 && addr <= 0xfe9f)
        ret = oam_read(gb, addr);
    else if (addr >= 0xfea0 && addr <= 0xfeff)
        ret = unused_read(gb, addr);
    else if (addr >= 0xff00 && addr <= 0xff7f)
        ret = io_read(gb, addr);
    else if (addr >= 0xff80 && addr <= 0xfffe)
        ret = hram_read(gb, addr);
    else if (addr == 0xffff)
        ret = interrupt_read(gb, addr);
    return ret;
}

uint8_t dma_get_data(struct gb *gb, uint16_t addr)
{
    return read_function[bus_get_mem_region(addr)](gb, addr);
}

void bus_write(struct gb *gb, uint16_t addr, uint8_t val)
{

    if (addr >= 0x0000 && addr <= 0x7fff)
        rom_write(gb, addr, val);
    else if (addr >= 0x8000 && addr <= 0x9fff)
        vram_write(gb, addr, val);
    else if (addr >= 0xa000 && addr <= 0xbfff)
        exram_write(gb, addr, val);
    else if (addr >= 0xc000 && addr <= 0xdfff)
        wram_write(gb, addr, val);
    else if (addr >= 0xe000 && addr <= 0xfdff)
        wram_write(gb, addr & 0xddff, val);
    else if (addr >= 0xfe00 && addr <= 0xfe9f)
        oam_write(gb, addr, val);
    else if (addr >= 0xfea0 && addr <= 0xfeff)
        unused_write(gb, addr, val);
    else if (addr >= 0xff00 && addr <= 0xff7f)
        io_write(gb, addr, val);
    else if (addr >= 0xff80 && addr <= 0xfffe)
        hram_write(gb, addr, val);
    else if (addr == 0xffff)
        interrupt_write(gb, addr, val);
}