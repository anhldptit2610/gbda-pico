#define _GNU_SOURCE

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SAMPLE_RATE             44100
#define NUM_CHANNELS            2
#define BUFFER_SIZE             1024

#define COLOR_WHITE          0x9dc2
#define COLOR_LIGHTGRAY      0x8d42
#define COLOR_DARKGRAY       0x3306
#define COLOR_BLACK          0x11c2

#define SYSTEM_CLOCK        4194304
#define LCD_HEIGHT          320
#define LCD_WIDTH           240
#define SCREEN_WIDTH        160
#define SCREEN_HEIGHT       144

#define KiB                 1024
#define MiB                 1048576

#define MSB(n)              (((uint16_t)(n) >> 8) & 0x00ff)
#define LSB(n)              ((uint16_t)(n) & 0x00ff)
#define TO_U16(lsb, msb)    (((uint16_t)(msb) << 8) | (uint16_t)(lsb))
#define BIT(f, n)           (((f) >> (n)) & 0x0001)
#define SET(f, n)           ((f) |= (1U << (n)))
#define RES(f, n)           ((f) &= ~(1U << (n)))
#define IN_RANGE(x, a, b)   ((x) >= (a) && (x) <= (b))

/* registers definition */
#define TIM_REG_DIV                 0xff04
#define TIM_REG_TIMA                0xff05
#define TIM_REG_TMA                 0xff06
#define TIM_REG_TAC                 0xff07

#define INTR_REG_IE                 0xffff
#define INTR_REG_IF                 0xff0f

#define PPU_REG_LCDC                0xff40
#define PPU_REG_STAT                0xff41
#define PPU_REG_SCY                 0xff42
#define PPU_REG_SCX                 0xff43
#define PPU_REG_LY                  0xff44
#define PPU_REG_LYC                 0xff45
#define PPU_REG_BGP                 0xff47
#define PPU_REG_OBP0                0xff48
#define PPU_REG_OBP1                0xff49
#define PPU_REG_WY                  0xff4a
#define PPU_REG_WX                  0xff4b

#define DMA_REG_OAM                 0xff46

#define STAT_INTR_MODE0             (1U << 3)
#define STAT_INTR_MODE1             (1U << 4)
#define STAT_INTR_MODE2             (1U << 5)
#define STAT_INTR_LYC               (1U << 6)
#define INTR_SRC_VBLANK     (1U << 0)
#define INTR_SRC_LCD        (1U << 1)
#define INTR_SRC_TIMER      (1U << 2)
#define INTR_SRC_SERIAL     (1U << 3)
#define INTR_SRC_JOYPAD     (1U << 4)

enum BUS_REGIONS {
    ROM = (1U << 0),
    VRAM = (1U << 1),
    EXTERN_RAM = (1U << 2),
    WRAM = (1U << 3),
    ECHO_RAM = (1U << 4),
    OAM = (1U << 5),
    UNUSED = (1U << 6),
    IO = (1U << 7),
    HRAM = (1U << 8),
};

enum IO_REGIONS {
    INTERRUPT = (1U << 0),
    TIMER = (1U << 1),
    PPU = (1U << 2),
};

typedef enum {
    TETRIS,
    KIRBY_DREAM_LAND,
    LEGENDS_OF_ZELDA,
    DR_MARIO,
    METROID_II,
} games_t;

typedef enum {
    NORMAL,
    HALT,
    HALT_BUG,
    STOP
} gb_mode_t;

typedef enum {
    HBLANK,
    VBLANK,
    OAM_SCAN,
    DRAWING,
} ppu_mode_t;

typedef enum {
    OFF,
    WAITING,
    TRANSFERING,
} dma_mode_t;

typedef enum {
    SQUARE1 = 1,
    SQUARE2 = 2,
    WAVE    = 3,
    NOISE   = 4,
    CTRL = 5,
} apu_channel_t;

typedef enum {
    OBP0,
    OBP1,
    BGP,
} palette_t;

typedef enum {
    BG_WIN,
    SPRITE,
} pixel_type_t;

struct sm83 {
    bool ime;
    union {
        uint16_t val;
        struct {
            union {
                uint8_t f;
                struct {
                    uint8_t unused_0 : 1;
                    uint8_t unused_1 : 1;
                    uint8_t unused_2 : 1;
                    uint8_t unused_3 : 1;
                    uint8_t c : 1;
                    uint8_t h : 1;
                    uint8_t n : 1;
                    uint8_t z : 1;
                } flag;
            };
            uint8_t a;
        };
    } af;

    union {
        uint16_t val;
        struct {
            uint8_t c;
            uint8_t b;
        };
    } bc;

    union {
        uint16_t val;
        struct {
            uint8_t e;
            uint8_t d;
        };
    } de;

    union {
        uint16_t val;
        struct {
            uint8_t l;
            uint8_t h;
        };
    } hl;

    uint16_t sp;
    uint16_t pc;
};

struct cartridge {
    uint8_t *rom;
    uint8_t ram[32 * KiB];
    bool cartridge_loaded;
    struct info {
        char name[17];
        uint8_t type;
        int rom_size;
        int ram_size;
        int bank_size;
        // TODO: fill in other missing infos
    } infos;
};

typedef enum ACCESS_MODE {
    READ,
    WRITE,
    DUMMY,
} access_mode_t;

struct memory_access_record {
    uint16_t addr;
    uint16_t val;
    access_mode_t mode;
};

struct interrupt {
    uint8_t ie;
    uint8_t flag;
    bool interrupt_handled;
};

struct timer {
    uint16_t div : 14;
    uint8_t tima;
    uint8_t tma;
    union {
        uint8_t val;
        struct {
            uint8_t freq : 2;
            uint8_t enable : 1;
            uint8_t unused : 5;
        };
    } tac;
    bool old_edge;
};

struct oam_entry {
    uint8_t y;
    uint8_t x;
    uint8_t tile_index;
    union {
        uint8_t val;
        struct {
            uint8_t cgb_palette : 3;
            uint8_t bank : 1;
            uint8_t dmg_palette : 1;
            uint8_t x_flip : 1;
            uint8_t y_flip : 1;
            uint8_t priority : 1;
        };
    } attributes;
};

struct ppu {
    bool frame_ready;

    struct {
        uint8_t val;
        bool bg_win_enable;
        bool obj_enable;
        bool win_enable;
        bool ppu_enable;
        uint8_t obj_size : 5;
        uint16_t bg_tile_map;
        uint16_t bg_win_tiles;
        uint16_t win_tile_map;
    } lcdc;

    union {
        uint8_t val;
        struct {
            uint8_t ppu_mode : 2;
            uint8_t lyc_equal_ly : 1;
            uint8_t mode0_int_select : 1;
            uint8_t mode1_int_select : 1;
            uint8_t mode2_int_select : 1;
            uint8_t lyc_int_select : 1;
            uint8_t unused : 1;
        };
    } stat;

    uint8_t scy;
    uint8_t scx;
    uint8_t ly;
    uint8_t lyc;
    uint8_t pal[3];
    uint8_t wy;
    uint8_t wx;
    uint16_t ticks;
    ppu_mode_t mode;
    bool scan_line_ready;
    struct oam_entry oam_entry[10];
    uint8_t oam_entry_cnt : 4;
    uint8_t sprite_cnt : 4;
    bool stat_intr_line;
    union {
        uint8_t val;
        struct {
            uint8_t ppu_mode : 2;
            uint8_t lyc_equal_ly : 1;
            uint8_t mode0 : 1;
            uint8_t mode1 : 1;
            uint8_t mode2 : 1;
            uint8_t lyc_int : 1;
            uint8_t unused : 1;
        };
    } stat_intr_src;
    bool window_in_frame;
    int window_line_cnt;
    bool draw_window_this_line;
};

struct dma {
    int tick;
    dma_mode_t mode;
    uint16_t reg;
    uint16_t start_addr;
};

struct joypad {
    union {
        uint8_t val;
        struct {
            uint8_t keys : 4;
            uint8_t select_dpad : 1;
            uint8_t select_button : 1;
            uint8_t unused : 2;
        };
    } joyp;
    bool a;
    bool b;
    bool select;
    bool start;
    bool right;
    bool left;
    bool up;
    bool down;
};

struct mbc1{
    bool ram_enable;
    uint8_t rom_bank : 5;
    uint8_t ram_bank : 2;
    bool banking_mode;
    bool has_battery;
    uint8_t *ram;
};

struct mbc3 {
    bool ram_enable;
    uint8_t rom_bank : 7;
    uint8_t ram_bank : 2;
};

struct mbc {
    struct mbc1 mbc1;
    struct mbc3 mbc3;
};

struct serial {
    uint8_t sb;
    uint8_t sc;
};

struct gb {
    uint16_t frame_buffer[SCREEN_HEIGHT * SCREEN_WIDTH];
    uint16_t line_buffer[SCREEN_WIDTH];
    uint8_t vram[0x2000];
    uint8_t extern_ram[8 * KiB];
    uint8_t wram[0x2000];
    uint8_t echo_ram[0x1e00];
    uint8_t oam[0xa0];
    uint8_t unused[0x60];
    uint8_t hram[0x7f];
    gb_mode_t mode;
    struct sm83 cpu;
    struct cartridge cart;
    struct interrupt interrupt;
    struct timer timer;
    struct ppu ppu;
    struct dma dma;
    struct joypad joypad;
    struct mbc mbc;
    struct serial serial;
    int executed_cycle;
};

const uint16_t palette[4] = {COLOR_WHITE, COLOR_LIGHTGRAY, COLOR_DARKGRAY, COLOR_BLACK};

const uint8_t div_bit_to_freq[] = {
    [0] = 7,
    [1] = 1,
    [2] = 3,
    [3] = 5
};

const uint8_t interrupt_vector[] = {
    [INTR_SRC_VBLANK] = 0x40,
    [INTR_SRC_LCD] = 0x48,
    [INTR_SRC_TIMER] = 0x50,
    [INTR_SRC_SERIAL] = 0x58,
    [INTR_SRC_JOYPAD] = 0x60,
};

const uint8_t instr_cycle[256] = {
//  x0 x1 x2 x3 x4 x5 x6 x7 x8 x9 xA xB xC xD xE xF
    1, 3, 2, 2, 1, 1, 2, 1, 5, 2, 2, 2, 1, 1, 2, 1, // 0x
    1, 3, 2, 2, 1, 1, 2, 1, 3, 2, 2, 2, 1, 1, 2, 1, // 1x
    2, 3, 2, 2, 1, 1, 2, 1, 2, 2, 2, 2, 1, 1, 2, 1, // 2x
    2, 3, 2, 2, 3, 3, 3, 1, 2, 2, 2, 2, 1, 1, 2, 1, // 3x
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, // 4x
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, // 5x
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, // 6x
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, // 7x
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, // 8x
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, // 9x
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, // Ax
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, // Bx
    2, 3, 3, 4, 3, 4, 2, 4, 2, 4, 3, 0, 3, 6, 2, 4, // Cx
    2, 3, 3, 4, 3, 4, 2, 4, 2, 4, 3, 1, 3, 6, 2, 4, // Dx
    3, 3, 2, 0, 0, 4, 2, 4, 4, 1, 4, 0, 0, 0, 2, 4, // Ex
    3, 3, 2, 1, 0, 4, 2, 4, 4, 2, 4, 1, 0, 0, 2, 4, // Fx
};

const uint8_t cb_instr_cycle[256] = {
//  x0 x1 x2 x3 x4 x5 x6 x7 x8 x9 xA xB xC xD xE xF
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // 0x 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // 1x 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // 2x 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // 3x 
    2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2, // 4x 
    2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2, // 5x 
    2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2, // 6x 
    2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2, // 7x 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // 8x 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // 9x 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // Ax 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // Bx 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // Cx 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // Dx 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // Ex 
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2, // Fx 
};

#define INTERRUPT_REQUEST(intr_src)                     \
    gb->interrupt.flag |= intr_src

#define IS_INTERRUPT_PENDING()                          \
    (gb->interrupt.ie & gb->interrupt.flag & 0x1f)

#define GET_MEM_REGION(addr)                    \
    ((IN_RANGE(addr, 0x0000, 0x7fff) << 0)|     \
    (IN_RANGE(addr, 0x8000, 0x9fff) << 1) |     \
    (IN_RANGE(addr, 0xa000, 0xbfff) << 2) |     \
    (IN_RANGE(addr, 0xc000, 0xdfff) << 3) |     \
    (IN_RANGE(addr, 0xe000, 0xfdff) << 4) |     \
    (IN_RANGE(addr, 0xfe00, 0xfe9f) << 5) |     \
    (IN_RANGE(addr, 0xfea0, 0xfeff) << 6) |     \
    (IN_RANGE(addr, 0xff00, 0xff7f) << 7) |     \
    (IN_RANGE(addr, 0xff80, 0xfffe) << 8) |     \
    ((addr == 0xffff) << 7))

#define IS_INTERRUPT_REG(addr)                          \
    (addr == INTR_REG_IE || addr == INTR_REG_IF)

#define IS_TIMER_REG(addr)                              \
    (addr == TIM_REG_DIV || addr == TIM_REG_TAC ||      \
    addr == TIM_REG_TIMA || addr == TIM_REG_TMA)

#define IS_PPU_REG(addr)                                                        \
    (addr == PPU_REG_LCDC || addr == PPU_REG_STAT || addr == PPU_REG_SCY ||     \
    addr == PPU_REG_SCX || addr == PPU_REG_LY || addr == PPU_REG_LYC ||         \
    addr == PPU_REG_BGP || addr == PPU_REG_OBP0 || addr == PPU_REG_OBP1 ||      \
    addr == PPU_REG_WY || addr == PPU_REG_WX)

#define WHICH_IO_REGION(addr)                   \
    ((IS_INTERRUPT_REG(addr) << 0)        |     \
    (IN_RANGE(addr, 0xff04, 0xff07) << 1) |     \
    (IN_RANGE(addr, 0xff40, 0xff4b) << 2))

#define SET_MODE(Mode)              \
    gb->ppu.stat.ppu_mode = Mode;   \
    gb->ppu.mode = Mode

/* bus declarations */
uint8_t dma_get_data(struct gb *gb, uint16_t addr);
uint8_t bus_read(struct gb *gb, uint16_t addr);
void bus_write(struct gb *gb, uint16_t addr, uint8_t val);

void push_word(struct gb *gb, uint16_t val);

/* interrupt declarations */
uint8_t interrupt_read(struct gb *gb, uint16_t addr);
void interrupt_write(struct gb *gb, uint16_t addr, uint8_t val);
int interrupt_handler(struct gb *gb, uint8_t intr_src);
void interrupt_process(struct gb *gb);
void interrupt_request(struct gb *gb, uint8_t intr_src);
bool is_interrupt_pending(struct gb *gb);

/* timer declarations */
uint8_t timer_read(struct gb *gb, uint16_t addr);
void timer_write(struct gb *gb, uint16_t addr, uint8_t val);

/* CPU declarations */
void cpu_step(struct gb *gb);
void cpu_init(struct gb *gb);
void cpu_cycle(struct gb *gb, int cycles);

/* PPU declarations */
int cmpfunc(const void *a, const void *b);
uint8_t ppu_read(struct gb *gb, uint16_t addr);
void ppu_write(struct gb *gb, uint16_t addr, uint8_t val);

/**********************************************************************************************/
/************************************* CPU related parts **************************************/
/**********************************************************************************************/

void sm83_init(struct gb *gb)
{
    gb->cpu.pc = 0;
    gb->cart.cartridge_loaded = false;
}

#define SM83_FETCH_BYTE()                   \
    bus_read(gb, gb->cpu.pc++)

#define SM83_FETCH_WORD(ret)            \
    lsb = SM83_FETCH_BYTE();            \
    msb = SM83_FETCH_BYTE();            \
    ret = TO_U16(lsb, msb)

uint16_t sm83_fetch_word(struct gb *gb)
{
    uint8_t lsb = SM83_FETCH_BYTE();
    uint8_t msb = SM83_FETCH_BYTE();

    return TO_U16(lsb, msb);
}

#define SM83_PUSH_BYTE(val)             \
    bus_write(gb, --gb->cpu.sp, val)

void sm83_push_word(struct gb *gb, uint16_t val)
{
    SM83_PUSH_BYTE(MSB(val));
    SM83_PUSH_BYTE(LSB(val));
}

uint8_t sm83_pop_byte(struct gb *gb)
{
    gb->cpu.sp++;
    return bus_read(gb, gb->cpu.sp - 1);
}

uint16_t sm83_pop_word(struct gb *gb)
{
    uint8_t lsb = sm83_pop_byte(gb);
    uint8_t msb = sm83_pop_byte(gb);

    return TO_U16(lsb, msb);
}

#define TOGGLE_ZNH(res, fn, fh)             \
    gb->cpu.af.flag.z = !(uint8_t)(res);      \
    gb->cpu.af.flag.n = (bool)(fn);           \
    gb->cpu.af.flag.h = (bool)(fh)

void toggle_znh(struct gb *gb, uint8_t res, bool n, bool h)
{
    gb->cpu.af.flag.z = !res;
    gb->cpu.af.flag.n = n;
    gb->cpu.af.flag.h = h;
}

/* CPU Instructions */

#define LD_INDIRECT_HL_N(n)     \
    bus_write(gb, gb->cpu.hl.val, n)

#define LDH_INDIRECT_C_A()      \
    bus_write(gb, 0xff00 + gb->cpu.bc.c, gb->cpu.af.a)

#define LDH_A_INDIRECT_C()      \
    gb->cpu.af.a = bus_read(gb, 0xff00 + gb->cpu.bc.c)

#define LDH_INDIRECT_N_A(n)     \
    bus_write(gb, 0xff00 + n, gb->cpu.af.a)

#define LDH_A_INDIRECT_N(n)     \
    gb->cpu.af.a = bus_read(gb, 0xff00 + n)

#define LD_INDIRECT_NN_SP(nn)   \
    bus_write(gb, nn, LSB(gb->cpu.sp));   \
    bus_write(gb, nn + 1, MSB(gb->cpu.sp))

#define PUSH_RR(rr)     \
    sm83_push_word(gb, rr)

#define LD_HL_SP_PLUS_I8(i8)                                            \
    gb->cpu.hl.val = gb->cpu.sp + (int8_t)i8;                           \
    gb->cpu.af.flag.z = 0;                                              \
    gb->cpu.af.flag.n = 0;                                              \
    gb->cpu.af.flag.h = BIT((gb->cpu.sp + i8) ^ gb->cpu.sp ^ i8, 4);    \
    gb->cpu.af.flag.c = BIT((gb->cpu.sp + i8) ^ gb->cpu.sp ^ i8, 8)

#define ADD(b, d)                                   \
    res = gb->cpu.af.a + b + d;                     \
    carry_per_bit = res ^ gb->cpu.af.a ^ b;         \
    gb->cpu.af.a = res;                             \
    TOGGLE_ZNH(res, 0, BIT(carry_per_bit, 4));  \
    gb->cpu.af.flag.c = BIT(carry_per_bit, 8)       

#define SUB(b, d)                                                                                               \
    res = gb->cpu.af.a + ~b + 1 - d;                \
    carry_per_bit = res ^ gb->cpu.af.a ^ ~b;        \
    gb->cpu.af.a = res;                             \
    TOGGLE_ZNH(res, 1, !BIT(carry_per_bit, 4)); \
    gb->cpu.af.flag.c = !BIT(carry_per_bit, 8);    

#define CP(b)                                                                                                   \
    TOGGLE_ZNH(gb->cpu.af.a + ~b + 1, 1, !BIT(gb->cpu.af.a ^ ~b ^ (gb->cpu.af.a + ~b + 1), 4));             \
    gb->cpu.af.flag.c = !BIT(gb->cpu.af.a ^ ~b ^ (gb->cpu.af.a + ~b + 1), 8)

#define INC_R(r)                                            \
    TOGGLE_ZNH(r + 1, 0, BIT((r + 1) ^ r ^ 1, 4));      \
    r += 1

#define INC_INDIRECT_HL()                                               \
    operand = bus_read(gb, gb->cpu.hl.val);                             \
    bus_write(gb, gb->cpu.hl.val, operand + 1);                         \
    TOGGLE_ZNH(operand + 1, 0, BIT((operand + 1) ^ operand ^ 1U, 4))

#define DEC_R(r)                                               \
    TOGGLE_ZNH(r - 1, 1, !BIT((r - 1) ^ r ^ 0xff, 4));     \
    r--;

#define DEC_INDIRECT_HL()                                               \
    operand = bus_read(gb, gb->cpu.hl.val);                             \
    bus_write(gb, gb->cpu.hl.val, operand - 1);                         \
    TOGGLE_ZNH(operand - 1, 1, !BIT((operand - 1) ^ operand ^ 0xff, 4))

#define AND(b)                          \
    gb->cpu.af.a &= b;                  \
    TOGGLE_ZNH(gb->cpu.af.a, 0, 1); \
    gb->cpu.af.flag.c = 0

#define OR(b)                              \
    gb->cpu.af.a |= b;                     \
    TOGGLE_ZNH(gb->cpu.af.a, 0, 0);    \
    gb->cpu.af.flag.c = 0 

#define XOR(b)                             \
    gb->cpu.af.a ^= b;                     \
    TOGGLE_ZNH(gb->cpu.af.a, 0, 0);    \
    gb->cpu.af.flag.c = 0 

#define CCF()                               \
    gb->cpu.af.flag.n = 0;                  \
    gb->cpu.af.flag.h = 0;                  \
    gb->cpu.af.flag.c = !gb->cpu.af.flag.c

// static inline void ccf(struct gb *gb)
// {
//     gb->cpu.af.flag.n = 0;
//     gb->cpu.af.flag.h = 0;
//     gb->cpu.af.flag.c = !gb->cpu.af.flag.c;
// }
#define SCF()               \
    gb->cpu.af.flag.n = 0;  \
    gb->cpu.af.flag.h = 0;  \
    gb->cpu.af.flag.c = 1
// static inline void scf(struct gb *gb)
// {
//     gb->cpu.af.flag.n = 0;
//     gb->cpu.af.flag.h = 0;
//     gb->cpu.af.flag.c = 1;
// }

#define DAA()                                                   \
    a = gb->cpu.af.a;                                           \
    if (!gb->cpu.af.flag.n) {                                   \
        if (gb->cpu.af.flag.h || (gb->cpu.af.a & 0x0f) > 0x09)  \
            a += 0x06;                                          \
        if (gb->cpu.af.flag.c || gb->cpu.af.a > 0x99) {         \
            a += 0x60;                                          \
            gb->cpu.af.flag.c = 1;                              \
        }                                                       \
    } else {                                                    \
        if (gb->cpu.af.flag.h)                                  \
            a -= 0x06;                                          \
        if (gb->cpu.af.flag.c)                                  \
            a -= 0x60;                                          \
    }                                                           \
    gb->cpu.af.flag.z = !a;                                     \
    gb->cpu.af.flag.h = 0;                                      \
    gb->cpu.af.a = a;                                           \

#define CPL()                       \
    gb->cpu.af.a = ~gb->cpu.af.a;   \
    gb->cpu.af.flag.n = 1;          \
    gb->cpu.af.flag.h = 1

#define INC_RR(rr)              \
    rr++

#define DEC_RR(rr)              \
    rr--

#define ADD_HL_RR(rr)                                                           \
    gb->cpu.af.flag.n = 0;                                                      \
    gb->cpu.af.flag.h = BIT((gb->cpu.hl.val + rr) ^ gb->cpu.hl.val ^ rr, 12);   \
    gb->cpu.af.flag.c = BIT((gb->cpu.hl.val + rr) ^ gb->cpu.hl.val ^ rr, 16);   \
    gb->cpu.hl.val = gb->cpu.hl.val + rr                                    


#define ADD_SP_I8(i8)                                                   \
    gb->cpu.af.flag.z = gb->cpu.af.flag.n = 0;                          \
    gb->cpu.af.flag.h = BIT((gb->cpu.sp + i8) ^ gb->cpu.sp ^ i8, 4);    \
    gb->cpu.af.flag.c = BIT((gb->cpu.sp + i8) ^ gb->cpu.sp ^ i8, 8);    \
    gb->cpu.sp = gb->cpu.sp + (int8_t)i8

#define RLCA()      \
    gb->cpu.af.flag.c = BIT(gb->cpu.af.a, 7);   \
    gb->cpu.af.a = (gb->cpu.af.a << 1) | gb->cpu.af.flag.c; \
    gb->cpu.af.flag.z = gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define RRCA()          \
    gb->cpu.af.flag.c = BIT(gb->cpu.af.a, 0);   \
    gb->cpu.af.a = (gb->cpu.af.a >> 1) | (gb->cpu.af.flag.c << 7);  \
    gb->cpu.af.flag.z = gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0 

#define RLA()           \
    operand = BIT(gb->cpu.af.a, 7); \
    gb->cpu.af.a = (gb->cpu.af.a << 1) | gb->cpu.af.flag.c; \
    gb->cpu.af.flag.c = operand;    \
    gb->cpu.af.flag.z = gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define RRA()               \
    operand = BIT(gb->cpu.af.a, 0);                                 \
    gb->cpu.af.a = (gb->cpu.af.a >> 1) | (gb->cpu.af.flag.c << 7);  \
    gb->cpu.af.flag.c = operand;           \
    gb->cpu.af.flag.z = gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0 

#define RLC_R(r)                        \
    gb->cpu.af.flag.c = BIT(r, 7);      \
    r = (r << 1) | gb->cpu.af.flag.c;   \
    gb->cpu.af.flag.z = !(r);           \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define RLC_INDIRECT_HL()                           \
    operand = bus_read(gb, gb->cpu.hl.val);         \
    gb->cpu.af.flag.c = BIT(operand, 7);            \
    operand = (operand << 1) | gb->cpu.af.flag.c;   \
    bus_write(gb, gb->cpu.hl.val, operand);         \
    gb->cpu.af.flag.z = !operand;                   \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define RRC_R(r)                                \
    gb->cpu.af.flag.c = BIT(r, 0);              \
    r = (r >> 1) | (gb->cpu.af.flag.c << 7);    \
    gb->cpu.af.flag.z = !(r);                   \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define RRC_INDIRECT_HL()                       \
    operand = bus_read(gb, gb->cpu.hl.val);     \
    gb->cpu.af.flag.c = BIT(operand, 0);        \
    operand = (operand >> 1) | (gb->cpu.af.flag.c << 7);    \
    bus_write(gb, gb->cpu.hl.val, operand);     \
    gb->cpu.af.flag.z = !operand;               \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0  

#define RL_R(r)                             \
    operand = BIT(r, 7);                    \
    r = (r << 1) | gb->cpu.af.flag.c;       \
    gb->cpu.af.flag.z = !(r);               \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0;  \
    gb->cpu.af.flag.c = operand

#define RL_INDIRECT_HL()                        \
    val = bus_read(gb, gb->cpu.hl.val);         \
    operand = BIT(val, 7);                      \
    val = (val << 1) | gb->cpu.af.flag.c;       \
    bus_write(gb, gb->cpu.hl.val, val);         \
    gb->cpu.af.flag.z = !val;                   \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0;  \
    gb->cpu.af.flag.c = operand

#define RR_R(r)                                 \
    operand = BIT(r, 0);                        \
    r = (r >> 1) | (gb->cpu.af.flag.c << 7);    \
    gb->cpu.af.flag.z = !r;                     \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0;  \
    gb->cpu.af.flag.c = operand

#define RR_INDIRECT_HL()                    \
    val = bus_read(gb, gb->cpu.hl.val);     \
    operand = BIT(val, 0);                  \
    val = (val >> 1) | (gb->cpu.af.flag.c << 7);    \
    bus_write(gb, gb->cpu.hl.val, val);     \
    gb->cpu.af.flag.z = !val;               \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0;      \
    gb->cpu.af.flag.c = operand

#define SLA_R(r)                                \
    gb->cpu.af.flag.c = BIT(r, 7);              \
    r <<= 1;                                    \
    gb->cpu.af.flag.z = !r;                     \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define SLA_INDIRECT_HL()                       \
    val = bus_read(gb, gb->cpu.hl.val);         \
    gb->cpu.af.flag.c = BIT(val, 7);            \
    val <<= 1;                                  \
    bus_write(gb, gb->cpu.hl.val, val);         \
    gb->cpu.af.flag.z = !val;                   \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define SRA_R(r)                                \
    gb->cpu.af.flag.c = BIT(r, 0);              \
    r = (r & 0x80) | (r >> 1);                  \
    gb->cpu.af.flag.z = !r;                     \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define SRA_INDIRECT_HL()                       \
    val = bus_read(gb, gb->cpu.hl.val);         \
    gb->cpu.af.flag.c = BIT(val, 0);            \
    val = (val & 0x80) | (val >> 1);            \
    bus_write(gb, gb->cpu.hl.val, val);         \
    gb->cpu.af.flag.z = !val;                   \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define SWAP_R(r)                               \
    r = (r >> 4) | (r << 4);                    \
    TOGGLE_ZNH(r, 0, 0);                    \
    gb->cpu.af.flag.c = 0

#define SWAP_INDIRECT_HL()                      \
    val = bus_read(gb, gb->cpu.hl.val);         \
    val = (val >> 4) | (val << 4);              \
    bus_write(gb, gb->cpu.hl.val, val);         \
    TOGGLE_ZNH(val, 0, 0);                  \
    gb->cpu.af.flag.c = 0

#define SRL_R(r)                    \
    gb->cpu.af.flag.c = BIT(r, 0);  \
    r >>= 1;                        \
    gb->cpu.af.flag.z = !r;         \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define SRL_INDIRECT_HL()                       \
    val = bus_read(gb, gb->cpu.hl.val);         \
    gb->cpu.af.flag.c = BIT(val, 0);            \
    val >>= 1;                                  \
    bus_write(gb, gb->cpu.hl.val, val);         \
    gb->cpu.af.flag.z = !val;                   \
    gb->cpu.af.flag.n = gb->cpu.af.flag.h = 0

#define BIT_N_R(n, r)                   \
    TOGGLE_ZNH(BIT(r, n), 0, 1)

#define BIT_N_INDIRECT_HL(n)                    \
    val = bus_read(gb, gb->cpu.hl.val);         \
    TOGGLE_ZNH(BIT(val, n), 0, 1)

#define RES_N_R(n, r)                           \
    RES(r, n)

#define RES_N_INDIRECT_HL(n)                    \
    val = bus_read(gb, gb->cpu.hl.val);         \
    RES(val, n);                                \
    bus_write(gb, gb->cpu.hl.val, val)

#define SET_N_R(n, r)                           \
    SET(r, n)

#define SET_N_INDIRECT_HL(n)                    \
    val = bus_read(gb, gb->cpu.hl.val);         \
    SET(val, n);                                \
    bus_write(gb, gb->cpu.hl.val, val)

#define JP(nn, offset, cond)                    \
    if (cond) {                                 \
        gb->executed_cycle += 1;                \
        gb->cpu.pc = nn + (int8_t)offset;       \
    }                                           

#define CALL(nn, cond)                          \
    if (cond) {                                 \
        sm83_push_word(gb, gb->cpu.pc);         \
        gb->cpu.pc = nn;                        \
        gb->executed_cycle += 3;                \
    }

#define RET(opcode, cond)                       \
    if (cond) {                                 \
        uint16_t pc = sm83_pop_word(gb);        \
        gb->cpu.pc = pc;                        \
        gb->executed_cycle += 3;                \
    }

#define RETI()                      \
    RET(0xc9, 1);                   \
    gb->cpu.ime = true

#define RST_N(n)                    \
    sm83_push_word(gb, gb->cpu.pc); \
    gb->cpu.pc = n

#define HALT()                      \
    gb->mode = HALT;                \

#define DI()                    \
    gb->cpu.ime = false

#define EI()                    \
    gb->cpu.ime = true

int execute_cb_instructions(struct gb *gb, uint8_t opcode)
{
    uint8_t operand, val;

    switch (opcode) {
    case 0x00: RLC_R(gb->cpu.bc.b);                              break;
    case 0x01: RLC_R(gb->cpu.bc.c);                              break;
    case 0x02: RLC_R(gb->cpu.de.d);                              break;
    case 0x03: RLC_R(gb->cpu.de.e);                              break;
    case 0x04: RLC_R(gb->cpu.hl.h);                              break;
    case 0x05: RLC_R(gb->cpu.hl.l);                              break;
    case 0x06: RLC_INDIRECT_HL();                                break;
    case 0x07: RLC_R(gb->cpu.af.a);                              break;
    case 0x08: RRC_R(gb->cpu.bc.b);                              break;
    case 0x09: RRC_R(gb->cpu.bc.c);                              break;
    case 0x0a: RRC_R(gb->cpu.de.d);                              break;
    case 0x0b: RRC_R(gb->cpu.de.e);                              break;
    case 0x0c: RRC_R(gb->cpu.hl.h);                              break;
    case 0x0d: RRC_R(gb->cpu.hl.l);                              break;
    case 0x0e: RRC_INDIRECT_HL();                                break;
    case 0x0f: RRC_R(gb->cpu.af.a);                              break;
    case 0x10: RL_R(gb->cpu.bc.b);                               break;
    case 0x11: RL_R(gb->cpu.bc.c);                               break;
    case 0x12: RL_R(gb->cpu.de.d);                               break;
    case 0x13: RL_R(gb->cpu.de.e);                               break;
    case 0x14: RL_R(gb->cpu.hl.h);                               break;
    case 0x15: RL_R(gb->cpu.hl.l);                               break;
    case 0x16: RL_INDIRECT_HL();                                 break;
    case 0x17: RL_R(gb->cpu.af.a);                               break;
    case 0x18: RR_R(gb->cpu.bc.b);                               break;
    case 0x19: RR_R(gb->cpu.bc.c);                               break;
    case 0x1a: RR_R(gb->cpu.de.d);                               break;
    case 0x1b: RR_R(gb->cpu.de.e);                               break;
    case 0x1c: RR_R(gb->cpu.hl.h);                               break;
    case 0x1d: RR_R(gb->cpu.hl.l);                               break;
    case 0x1e: RR_INDIRECT_HL();                                 break;
    case 0x1f: RR_R(gb->cpu.af.a);                               break;
    case 0x20: SLA_R(gb->cpu.bc.b);                              break;
    case 0x21: SLA_R(gb->cpu.bc.c);                              break;
    case 0x22: SLA_R(gb->cpu.de.d);                              break;
    case 0x23: SLA_R(gb->cpu.de.e);                              break;
    case 0x24: SLA_R(gb->cpu.hl.h);                              break;
    case 0x25: SLA_R(gb->cpu.hl.l);                              break;
    case 0x26: SLA_INDIRECT_HL();                                break;
    case 0x27: SLA_R(gb->cpu.af.a);                              break;
    case 0x28: SRA_R(gb->cpu.bc.b);                              break;
    case 0x29: SRA_R(gb->cpu.bc.c);                              break;
    case 0x2a: SRA_R(gb->cpu.de.d);                              break;
    case 0x2b: SRA_R(gb->cpu.de.e);                              break;
    case 0x2c: SRA_R(gb->cpu.hl.h);                              break;
    case 0x2d: SRA_R(gb->cpu.hl.l);                              break;
    case 0x2e: SRA_INDIRECT_HL();                                break;
    case 0x2f: SRA_R(gb->cpu.af.a);                              break;
    case 0x30: SWAP_R(gb->cpu.bc.b);                             break;
    case 0x31: SWAP_R(gb->cpu.bc.c);                             break;
    case 0x32: SWAP_R(gb->cpu.de.d);                             break;
    case 0x33: SWAP_R(gb->cpu.de.e);                             break;
    case 0x34: SWAP_R(gb->cpu.hl.h);                             break;
    case 0x35: SWAP_R(gb->cpu.hl.l);                             break;
    case 0x36: SWAP_INDIRECT_HL();                               break;
    case 0x37: SWAP_R(gb->cpu.af.a);                             break;
    case 0x38: SRL_R(gb->cpu.bc.b);                              break;
    case 0x39: SRL_R(gb->cpu.bc.c);                              break;
    case 0x3a: SRL_R(gb->cpu.de.d);                              break;
    case 0x3b: SRL_R(gb->cpu.de.e);                              break;
    case 0x3c: SRL_R(gb->cpu.hl.h);                              break;
    case 0x3d: SRL_R(gb->cpu.hl.l);                              break;
    case 0x3e: SRL_INDIRECT_HL();                                break;
    case 0x3f: SRL_R(gb->cpu.af.a);                              break;
    case 0x40: BIT_N_R(0, gb->cpu.bc.b);                         break;
    case 0x41: BIT_N_R(0, gb->cpu.bc.c);                         break;
    case 0x42: BIT_N_R(0, gb->cpu.de.d);                         break;
    case 0x43: BIT_N_R(0, gb->cpu.de.e);                         break;
    case 0x44: BIT_N_R(0, gb->cpu.hl.h);                         break;
    case 0x45: BIT_N_R(0, gb->cpu.hl.l);                         break;
    case 0x46: BIT_N_INDIRECT_HL(0);                             break;
    case 0x47: BIT_N_R(0, gb->cpu.af.a);                         break;
    case 0x48: BIT_N_R(1, gb->cpu.bc.b);                         break;
    case 0x49: BIT_N_R(1, gb->cpu.bc.c);                         break;
    case 0x4a: BIT_N_R(1, gb->cpu.de.d);                         break;
    case 0x4b: BIT_N_R(1, gb->cpu.de.e);                         break;
    case 0x4c: BIT_N_R(1, gb->cpu.hl.h);                         break;
    case 0x4d: BIT_N_R(1, gb->cpu.hl.l);                         break;
    case 0x4e: BIT_N_INDIRECT_HL(1);                             break;
    case 0x4f: BIT_N_R(1, gb->cpu.af.a);                         break;
    case 0x50: BIT_N_R(2, gb->cpu.bc.b);                         break;
    case 0x51: BIT_N_R(2, gb->cpu.bc.c);                         break;
    case 0x52: BIT_N_R(2, gb->cpu.de.d);                         break;
    case 0x53: BIT_N_R(2, gb->cpu.de.e);                         break;
    case 0x54: BIT_N_R(2, gb->cpu.hl.h);                         break;
    case 0x55: BIT_N_R(2, gb->cpu.hl.l);                         break;
    case 0x56: BIT_N_INDIRECT_HL(2);                             break;
    case 0x57: BIT_N_R(2, gb->cpu.af.a);                         break;
    case 0x58: BIT_N_R(3, gb->cpu.bc.b);                         break;
    case 0x59: BIT_N_R(3, gb->cpu.bc.c);                         break;
    case 0x5a: BIT_N_R(3, gb->cpu.de.d);                         break;
    case 0x5b: BIT_N_R(3, gb->cpu.de.e);                         break;
    case 0x5c: BIT_N_R(3, gb->cpu.hl.h);                         break;
    case 0x5d: BIT_N_R(3, gb->cpu.hl.l);                         break;
    case 0x5e: BIT_N_INDIRECT_HL(3);                             break;
    case 0x5f: BIT_N_R(3, gb->cpu.af.a);                         break;
    case 0x60: BIT_N_R(4, gb->cpu.bc.b);                         break;
    case 0x61: BIT_N_R(4, gb->cpu.bc.c);                         break;
    case 0x62: BIT_N_R(4, gb->cpu.de.d);                         break;
    case 0x63: BIT_N_R(4, gb->cpu.de.e);                         break;
    case 0x64: BIT_N_R(4, gb->cpu.hl.h);                         break;
    case 0x65: BIT_N_R(4, gb->cpu.hl.l);                         break;
    case 0x66: BIT_N_INDIRECT_HL(4);                             break;
    case 0x67: BIT_N_R(4, gb->cpu.af.a);                         break;
    case 0x68: BIT_N_R(5, gb->cpu.bc.b);                         break;
    case 0x69: BIT_N_R(5, gb->cpu.bc.c);                         break;
    case 0x6a: BIT_N_R(5, gb->cpu.de.d);                         break;
    case 0x6b: BIT_N_R(5, gb->cpu.de.e);                         break;
    case 0x6c: BIT_N_R(5, gb->cpu.hl.h);                         break;
    case 0x6d: BIT_N_R(5, gb->cpu.hl.l);                         break;
    case 0x6e: BIT_N_INDIRECT_HL(5);                             break;
    case 0x6f: BIT_N_R(5, gb->cpu.af.a);                         break;
    case 0x70: BIT_N_R(6, gb->cpu.bc.b);                         break;
    case 0x71: BIT_N_R(6, gb->cpu.bc.c);                         break;
    case 0x72: BIT_N_R(6, gb->cpu.de.d);                         break;
    case 0x73: BIT_N_R(6, gb->cpu.de.e);                         break;
    case 0x74: BIT_N_R(6, gb->cpu.hl.h);                         break;
    case 0x75: BIT_N_R(6, gb->cpu.hl.l);                         break;
    case 0x76: BIT_N_INDIRECT_HL(6);                             break;
    case 0x77: BIT_N_R(6, gb->cpu.af.a);                         break;
    case 0x78: BIT_N_R(7, gb->cpu.bc.b);                         break;
    case 0x79: BIT_N_R(7, gb->cpu.bc.c);                         break;
    case 0x7a: BIT_N_R(7, gb->cpu.de.d);                         break;
    case 0x7b: BIT_N_R(7, gb->cpu.de.e);                         break;
    case 0x7c: BIT_N_R(7, gb->cpu.hl.h);                         break;
    case 0x7d: BIT_N_R(7, gb->cpu.hl.l);                         break;
    case 0x7e: BIT_N_INDIRECT_HL(7);                             break;
    case 0x7f: BIT_N_R(7, gb->cpu.af.a);                         break;
    case 0x80: RES_N_R(0, gb->cpu.bc.b);                         break;
    case 0x81: RES_N_R(0, gb->cpu.bc.c);                         break;
    case 0x82: RES_N_R(0, gb->cpu.de.d);                         break;
    case 0x83: RES_N_R(0, gb->cpu.de.e);                         break;
    case 0x84: RES_N_R(0, gb->cpu.hl.h);                         break;
    case 0x85: RES_N_R(0, gb->cpu.hl.l);                         break;
    case 0x86: RES_N_INDIRECT_HL(0);                             break;
    case 0x87: RES_N_R(0, gb->cpu.af.a);                         break;
    case 0x88: RES_N_R(1, gb->cpu.bc.b);                         break;
    case 0x89: RES_N_R(1, gb->cpu.bc.c);                         break;
    case 0x8a: RES_N_R(1, gb->cpu.de.d);                         break;
    case 0x8b: RES_N_R(1, gb->cpu.de.e);                         break;
    case 0x8c: RES_N_R(1, gb->cpu.hl.h);                         break;
    case 0x8d: RES_N_R(1, gb->cpu.hl.l);                         break;
    case 0x8e: RES_N_INDIRECT_HL(1);                             break;
    case 0x8f: RES_N_R(1, gb->cpu.af.a);                         break;
    case 0x90: RES_N_R(2, gb->cpu.bc.b);                         break;
    case 0x91: RES_N_R(2, gb->cpu.bc.c);                         break;
    case 0x92: RES_N_R(2, gb->cpu.de.d);                         break;
    case 0x93: RES_N_R(2, gb->cpu.de.e);                         break;
    case 0x94: RES_N_R(2, gb->cpu.hl.h);                         break;
    case 0x95: RES_N_R(2, gb->cpu.hl.l);                         break;
    case 0x96: RES_N_INDIRECT_HL(2);                             break;
    case 0x97: RES_N_R(2, gb->cpu.af.a);                         break;
    case 0x98: RES_N_R(3, gb->cpu.bc.b);                         break;
    case 0x99: RES_N_R(3, gb->cpu.bc.c);                         break;
    case 0x9a: RES_N_R(3, gb->cpu.de.d);                         break;
    case 0x9b: RES_N_R(3, gb->cpu.de.e);                         break;
    case 0x9c: RES_N_R(3, gb->cpu.hl.h);                         break;
    case 0x9d: RES_N_R(3, gb->cpu.hl.l);                         break;
    case 0x9e: RES_N_INDIRECT_HL(3);                             break;
    case 0x9f: RES_N_R(3, gb->cpu.af.a);                         break;
    case 0xa0: RES_N_R(4, gb->cpu.bc.b);                         break;
    case 0xa1: RES_N_R(4, gb->cpu.bc.c);                         break;
    case 0xa2: RES_N_R(4, gb->cpu.de.d);                         break;
    case 0xa3: RES_N_R(4, gb->cpu.de.e);                         break;
    case 0xa4: RES_N_R(4, gb->cpu.hl.h);                         break;
    case 0xa5: RES_N_R(4, gb->cpu.hl.l);                         break;
    case 0xa6: RES_N_INDIRECT_HL(4);                             break;
    case 0xa7: RES_N_R(4, gb->cpu.af.a);                         break;
    case 0xa8: RES_N_R(5, gb->cpu.bc.b);                         break;
    case 0xa9: RES_N_R(5, gb->cpu.bc.c);                         break;
    case 0xaa: RES_N_R(5, gb->cpu.de.d);                         break;
    case 0xab: RES_N_R(5, gb->cpu.de.e);                         break;
    case 0xac: RES_N_R(5, gb->cpu.hl.h);                         break;
    case 0xad: RES_N_R(5, gb->cpu.hl.l);                         break;
    case 0xae: RES_N_INDIRECT_HL(5);                             break;
    case 0xaf: RES_N_R(5, gb->cpu.af.a);                         break;
    case 0xb0: RES_N_R(6, gb->cpu.bc.b);                         break;
    case 0xb1: RES_N_R(6, gb->cpu.bc.c);                         break;
    case 0xb2: RES_N_R(6, gb->cpu.de.d);                         break;
    case 0xb3: RES_N_R(6, gb->cpu.de.e);                         break;
    case 0xb4: RES_N_R(6, gb->cpu.hl.h);                         break;
    case 0xb5: RES_N_R(6, gb->cpu.hl.l);                         break;
    case 0xb6: RES_N_INDIRECT_HL(6);                             break;
    case 0xb7: RES_N_R(6, gb->cpu.af.a);                         break;
    case 0xb8: RES_N_R(7, gb->cpu.bc.b);                         break;
    case 0xb9: RES_N_R(7, gb->cpu.bc.c);                         break;
    case 0xba: RES_N_R(7, gb->cpu.de.d);                         break;
    case 0xbb: RES_N_R(7, gb->cpu.de.e);                         break;
    case 0xbc: RES_N_R(7, gb->cpu.hl.h);                         break;
    case 0xbd: RES_N_R(7, gb->cpu.hl.l);                         break;
    case 0xbe: RES_N_INDIRECT_HL(7);                             break;
    case 0xbf: RES_N_R(7, gb->cpu.af.a);                         break;
    case 0xc0: SET_N_R(0, gb->cpu.bc.b);                         break;
    case 0xc1: SET_N_R(0, gb->cpu.bc.c);                         break;
    case 0xc2: SET_N_R(0, gb->cpu.de.d);                         break;
    case 0xc3: SET_N_R(0, gb->cpu.de.e);                         break;
    case 0xc4: SET_N_R(0, gb->cpu.hl.h);                         break;
    case 0xc5: SET_N_R(0, gb->cpu.hl.l);                         break;
    case 0xc6: SET_N_INDIRECT_HL(0);                             break;
    case 0xc7: SET_N_R(0, gb->cpu.af.a);                         break;
    case 0xc8: SET_N_R(1, gb->cpu.bc.b);                         break;
    case 0xc9: SET_N_R(1, gb->cpu.bc.c);                         break;
    case 0xca: SET_N_R(1, gb->cpu.de.d);                         break;
    case 0xcb: SET_N_R(1, gb->cpu.de.e);                         break;
    case 0xcc: SET_N_R(1, gb->cpu.hl.h);                         break;
    case 0xcd: SET_N_R(1, gb->cpu.hl.l);                         break;
    case 0xce: SET_N_INDIRECT_HL(1);                             break;
    case 0xcf: SET_N_R(1, gb->cpu.af.a);                         break;
    case 0xd0: SET_N_R(2, gb->cpu.bc.b);                         break;
    case 0xd1: SET_N_R(2, gb->cpu.bc.c);                         break;
    case 0xd2: SET_N_R(2, gb->cpu.de.d);                         break;
    case 0xd3: SET_N_R(2, gb->cpu.de.e);                         break;
    case 0xd4: SET_N_R(2, gb->cpu.hl.h);                         break;
    case 0xd5: SET_N_R(2, gb->cpu.hl.l);                         break;
    case 0xd6: SET_N_INDIRECT_HL(2);                             break;
    case 0xd7: SET_N_R(2, gb->cpu.af.a);                         break;
    case 0xd8: SET_N_R(3, gb->cpu.bc.b);                         break;
    case 0xd9: SET_N_R(3, gb->cpu.bc.c);                         break;
    case 0xda: SET_N_R(3, gb->cpu.de.d);                         break;
    case 0xdb: SET_N_R(3, gb->cpu.de.e);                         break;
    case 0xdc: SET_N_R(3, gb->cpu.hl.h);                         break;
    case 0xdd: SET_N_R(3, gb->cpu.hl.l);                         break;
    case 0xde: SET_N_INDIRECT_HL(3);                             break;
    case 0xdf: SET_N_R(3, gb->cpu.af.a);                         break;
    case 0xe0: SET_N_R(4, gb->cpu.bc.b);                         break;
    case 0xe1: SET_N_R(4, gb->cpu.bc.c);                         break;
    case 0xe2: SET_N_R(4, gb->cpu.de.d);                         break;
    case 0xe3: SET_N_R(4, gb->cpu.de.e);                         break;
    case 0xe4: SET_N_R(4, gb->cpu.hl.h);                         break;
    case 0xe5: SET_N_R(4, gb->cpu.hl.l);                         break;
    case 0xe6: SET_N_INDIRECT_HL(4);                             break;
    case 0xe7: SET_N_R(4, gb->cpu.af.a);                         break;
    case 0xe8: SET_N_R(5, gb->cpu.bc.b);                         break;
    case 0xe9: SET_N_R(5, gb->cpu.bc.c);                         break;
    case 0xea: SET_N_R(5, gb->cpu.de.d);                         break;
    case 0xeb: SET_N_R(5, gb->cpu.de.e);                         break;
    case 0xec: SET_N_R(5, gb->cpu.hl.h);                         break;
    case 0xed: SET_N_R(5, gb->cpu.hl.l);                         break;
    case 0xee: SET_N_INDIRECT_HL(5);                             break;
    case 0xef: SET_N_R(5, gb->cpu.af.a);                         break;
    case 0xf0: SET_N_R(6, gb->cpu.bc.b);                         break;
    case 0xf1: SET_N_R(6, gb->cpu.bc.c);                         break;
    case 0xf2: SET_N_R(6, gb->cpu.de.d);                         break;
    case 0xf3: SET_N_R(6, gb->cpu.de.e);                         break;
    case 0xf4: SET_N_R(6, gb->cpu.hl.h);                         break;
    case 0xf5: SET_N_R(6, gb->cpu.hl.l);                         break;
    case 0xf6: SET_N_INDIRECT_HL(6);                             break;
    case 0xf7: SET_N_R(6, gb->cpu.af.a);                         break;
    case 0xf8: SET_N_R(7, gb->cpu.bc.b);                         break;
    case 0xf9: SET_N_R(7, gb->cpu.bc.c);                         break;
    case 0xfa: SET_N_R(7, gb->cpu.de.d);                         break;
    case 0xfb: SET_N_R(7, gb->cpu.de.e);                         break;
    case 0xfc: SET_N_R(7, gb->cpu.hl.h);                         break;
    case 0xfd: SET_N_R(7, gb->cpu.hl.l);                         break;
    case 0xfe: SET_N_INDIRECT_HL(7);                             break;
    case 0xff: SET_N_R(7, gb->cpu.af.a);                         break;
    default:
        break;
    }
    return cb_instr_cycle[opcode];
}

void sm83_step(struct gb *gb)
{
    uint8_t opcode, a, msb, lsb;
    uint16_t operand, res, carry_per_bit;
    bool old_edge, new_edge, is_interrupt, stat_intr_line;
    static uint8_t dma_addr = 0;

    gb->executed_cycle = 0;
    opcode = (gb->mode == HALT) ? 0x00 : SM83_FETCH_BYTE();
    gb->executed_cycle = gb->executed_cycle + instr_cycle[opcode] + ((gb->interrupt.interrupt_handled) ? 5 : 0);
    switch (opcode) {
    case 0x00:                                                          break;
    case 0x01: gb->cpu.bc.val = sm83_fetch_word(gb);                    break;
    case 0x02: bus_write(gb, gb->cpu.bc.val, gb->cpu.af.a);             break;
    case 0x03: INC_RR(gb->cpu.bc.val);                                  break;
    case 0x04: INC_R(gb->cpu.bc.b);                                     break;
    case 0x05: DEC_R(gb->cpu.bc.b);                                     break;
    case 0x06: gb->cpu.bc.b = SM83_FETCH_BYTE();                        break;
    case 0x07: RLCA();                                                  break;
    case 0x08: 
        operand = sm83_fetch_word(gb);
        LD_INDIRECT_NN_SP(operand);
        break;
    case 0x09: ADD_HL_RR(gb->cpu.bc.val);                               break;
    case 0x0a: gb->cpu.af.a = bus_read(gb, gb->cpu.bc.val);             break;
    case 0x0b: DEC_RR(gb->cpu.bc.val);                                  break;
    case 0x0c: INC_R(gb->cpu.bc.c);                                     break;
    case 0x0d: DEC_R(gb->cpu.bc.c);                                     break;
    case 0x0e: gb->cpu.bc.c = SM83_FETCH_BYTE();                        break;
    case 0x0f: RRCA();                                                  break;
    case 0x10:                                                          break;
    case 0x11: gb->cpu.de.val = sm83_fetch_word(gb);                    break;
    case 0x12: bus_write(gb, gb->cpu.de.val, gb->cpu.af.a);             break;
    case 0x13: INC_RR(gb->cpu.de.val);                                  break;
    case 0x14: INC_R(gb->cpu.de.d);                                     break;
    case 0x15: DEC_R(gb->cpu.de.d);                                     break;
    case 0x16: gb->cpu.de.d = SM83_FETCH_BYTE();                        break;
    case 0x17: RLA();                                                   break;
    case 0x18: 
        operand = SM83_FETCH_BYTE();
        JP(gb->cpu.pc, operand, 1);
        break;
    case 0x19: ADD_HL_RR(gb->cpu.de.val);                               break;
    case 0x1a: gb->cpu.af.a = bus_read(gb, gb->cpu.de.val);             break;
    case 0x1b: DEC_RR(gb->cpu.de.val);                                  break;
    case 0x1c: INC_R(gb->cpu.de.e);                                     break;
    case 0x1d: DEC_R(gb->cpu.de.e);                                     break;
    case 0x1e: gb->cpu.de.e = SM83_FETCH_BYTE();                        break;
    case 0x1f: RRA();                                                   break;
    case 0x20: 
        operand = SM83_FETCH_BYTE();
        JP(gb->cpu.pc, operand, !gb->cpu.af.flag.z);
        break;
    case 0x21: gb->cpu.hl.val = sm83_fetch_word(gb);                    break;
    case 0x22: bus_write(gb, gb->cpu.hl.val++, gb->cpu.af.a);           break;
    case 0x23: INC_RR(gb->cpu.hl.val);                                  break;
    case 0x24: INC_R(gb->cpu.hl.h);                                     break;
    case 0x25: DEC_R(gb->cpu.hl.h);                                     break;
    case 0x26: gb->cpu.hl.h = SM83_FETCH_BYTE();                        break;
    case 0x27: DAA();                                                   break;
    case 0x28: 
        operand = SM83_FETCH_BYTE();
        JP(gb->cpu.pc, operand, gb->cpu.af.flag.z);
        break;
    case 0x29: ADD_HL_RR(gb->cpu.hl.val);                               break;
    case 0x2a: gb->cpu.af.a = bus_read(gb, gb->cpu.hl.val++);           break;
    case 0x2b: DEC_RR(gb->cpu.hl.val);                                  break;
    case 0x2c: INC_R(gb->cpu.hl.l);                                     break;
    case 0x2d: DEC_R(gb->cpu.hl.l);                                     break;
    case 0x2e: gb->cpu.hl.l = SM83_FETCH_BYTE();                        break;
    case 0x2f: CPL();                                                   break;
    case 0x30:
        operand = SM83_FETCH_BYTE();
        JP(gb->cpu.pc, operand, !gb->cpu.af.flag.c);
        break;
    case 0x31: gb->cpu.sp = sm83_fetch_word(gb);                        break;
    case 0x32: bus_write(gb, gb->cpu.hl.val--, gb->cpu.af.a);           break;
    case 0x33: INC_RR(gb->cpu.sp);                                      break;
    case 0x34: INC_INDIRECT_HL();                                       break;
    case 0x35: DEC_INDIRECT_HL();                                       break;
    case 0x36: 
        operand = SM83_FETCH_BYTE();
        LD_INDIRECT_HL_N(operand);
        break;
    case 0x37: SCF();                                                   break;
    case 0x38:
        operand = SM83_FETCH_BYTE();
        JP(gb->cpu.pc, operand, gb->cpu.af.flag.c);
        break;
    case 0x39: ADD_HL_RR(gb->cpu.sp);                                   break;
    case 0x3a: gb->cpu.af.a = bus_read(gb, gb->cpu.hl.val--);           break;
    case 0x3b: DEC_RR(gb->cpu.sp);                                      break;
    case 0x3c: INC_R(gb->cpu.af.a);                                     break;
    case 0x3d: DEC_R(gb->cpu.af.a);                                     break;
    case 0x3e: gb->cpu.af.a = SM83_FETCH_BYTE();                        break;
    case 0x3f: CCF();                                                   break;
    case 0x40: gb->cpu.bc.b = gb->cpu.bc.b;                             break;
    case 0x41: gb->cpu.bc.b = gb->cpu.bc.c;                             break;
    case 0x42: gb->cpu.bc.b = gb->cpu.de.d;                             break;
    case 0x43: gb->cpu.bc.b = gb->cpu.de.e;                             break;
    case 0x44: gb->cpu.bc.b = gb->cpu.hl.h;                             break;
    case 0x45: gb->cpu.bc.b = gb->cpu.hl.l;                             break;
    case 0x46: gb->cpu.bc.b = bus_read(gb, gb->cpu.hl.val);             break;
    case 0x47: gb->cpu.bc.b = gb->cpu.af.a;                             break;
    case 0x48: gb->cpu.bc.c = gb->cpu.bc.b;                             break;
    case 0x49: gb->cpu.bc.c = gb->cpu.bc.c;                             break;
    case 0x4a: gb->cpu.bc.c = gb->cpu.de.d;                             break;
    case 0x4b: gb->cpu.bc.c = gb->cpu.de.e;                             break;
    case 0x4c: gb->cpu.bc.c = gb->cpu.hl.h;                             break;
    case 0x4d: gb->cpu.bc.c = gb->cpu.hl.l;                             break;
    case 0x4e: gb->cpu.bc.c = bus_read(gb, gb->cpu.hl.val);             break;
    case 0x4f: gb->cpu.bc.c = gb->cpu.af.a;                             break;
    case 0x50: gb->cpu.de.d = gb->cpu.bc.b;                             break;
    case 0x51: gb->cpu.de.d = gb->cpu.bc.c;                             break;
    case 0x52: gb->cpu.de.d = gb->cpu.de.d;                             break;
    case 0x53: gb->cpu.de.d = gb->cpu.de.e;                             break;
    case 0x54: gb->cpu.de.d = gb->cpu.hl.h;                             break;
    case 0x55: gb->cpu.de.d = gb->cpu.hl.l;                             break;
    case 0x56: gb->cpu.de.d = bus_read(gb, gb->cpu.hl.val);             break;
    case 0x57: gb->cpu.de.d = gb->cpu.af.a;                             break;
    case 0x58: gb->cpu.de.e = gb->cpu.bc.b;                             break;
    case 0x59: gb->cpu.de.e = gb->cpu.bc.c;                             break;
    case 0x5a: gb->cpu.de.e = gb->cpu.de.d;                             break;
    case 0x5b: gb->cpu.de.e = gb->cpu.de.e;                             break;
    case 0x5c: gb->cpu.de.e = gb->cpu.hl.h;                             break;
    case 0x5d: gb->cpu.de.e = gb->cpu.hl.l;                             break;
    case 0x5e: gb->cpu.de.e = bus_read(gb, gb->cpu.hl.val);             break;
    case 0x5f: gb->cpu.de.e = gb->cpu.af.a;                             break;
    case 0x60: gb->cpu.hl.h = gb->cpu.bc.b;                             break;
    case 0x61: gb->cpu.hl.h = gb->cpu.bc.c;                             break;
    case 0x62: gb->cpu.hl.h = gb->cpu.de.d;                             break;
    case 0x63: gb->cpu.hl.h = gb->cpu.de.e;                             break;
    case 0x64: gb->cpu.hl.h = gb->cpu.hl.h;                             break;
    case 0x65: gb->cpu.hl.h = gb->cpu.hl.l;                             break;
    case 0x66: gb->cpu.hl.h = bus_read(gb, gb->cpu.hl.val);             break;
    case 0x67: gb->cpu.hl.h = gb->cpu.af.a;                             break;
    case 0x68: gb->cpu.hl.l = gb->cpu.bc.b;                             break;
    case 0x69: gb->cpu.hl.l = gb->cpu.bc.c;                             break;
    case 0x6a: gb->cpu.hl.l = gb->cpu.de.d;                             break;
    case 0x6b: gb->cpu.hl.l = gb->cpu.de.e;                             break;
    case 0x6c: gb->cpu.hl.l = gb->cpu.hl.h;                             break;
    case 0x6d: gb->cpu.hl.l = gb->cpu.hl.l;                             break;
    case 0x6e: gb->cpu.hl.l = bus_read(gb, gb->cpu.hl.val);             break;
    case 0x6f: gb->cpu.hl.l = gb->cpu.af.a;                             break;
    case 0x70: bus_write(gb, gb->cpu.hl.val, gb->cpu.bc.b);             break;
    case 0x71: bus_write(gb, gb->cpu.hl.val, gb->cpu.bc.c);             break;
    case 0x72: bus_write(gb, gb->cpu.hl.val, gb->cpu.de.d);             break;
    case 0x73: bus_write(gb, gb->cpu.hl.val, gb->cpu.de.e);             break;
    case 0x74: bus_write(gb, gb->cpu.hl.val, gb->cpu.hl.h);             break;
    case 0x75: bus_write(gb, gb->cpu.hl.val, gb->cpu.hl.l);             break;
    case 0x76: HALT();                                                  break;
    case 0x77: bus_write(gb, gb->cpu.hl.val, gb->cpu.af.a);             break;
    case 0x78: gb->cpu.af.a = gb->cpu.bc.b;                             break;
    case 0x79: gb->cpu.af.a = gb->cpu.bc.c;                             break;
    case 0x7a: gb->cpu.af.a = gb->cpu.de.d;                             break;
    case 0x7b: gb->cpu.af.a = gb->cpu.de.e;                             break;
    case 0x7c: gb->cpu.af.a = gb->cpu.hl.h;                             break;
    case 0x7d: gb->cpu.af.a = gb->cpu.hl.l;                             break;
    case 0x7e: gb->cpu.af.a = bus_read(gb, gb->cpu.hl.val);             break;
    case 0x7f: gb->cpu.af.a = gb->cpu.af.a;                             break;
    case 0x80: ADD(gb->cpu.bc.b, 0);                                    break;
    case 0x81: ADD(gb->cpu.bc.c, 0);                                    break;
    case 0x82: ADD(gb->cpu.de.d, 0);                                    break;
    case 0x83: ADD(gb->cpu.de.e, 0);                                    break;
    case 0x84: ADD(gb->cpu.hl.h, 0);                                    break;
    case 0x85: ADD(gb->cpu.hl.l, 0);                                    break;
    case 0x86: 
        operand = bus_read(gb, gb->cpu.hl.val);
        ADD(operand, 0);
        break;
    case 0x87: ADD(gb->cpu.af.a, 0);                                    break;
    case 0x88: ADD(gb->cpu.bc.b, gb->cpu.af.flag.c);                    break;
    case 0x89: ADD(gb->cpu.bc.c, gb->cpu.af.flag.c);                    break;
    case 0x8a: ADD(gb->cpu.de.d, gb->cpu.af.flag.c);                    break;
    case 0x8b: ADD(gb->cpu.de.e, gb->cpu.af.flag.c);                    break;
    case 0x8c: ADD(gb->cpu.hl.h, gb->cpu.af.flag.c);                    break;
    case 0x8d: ADD(gb->cpu.hl.l, gb->cpu.af.flag.c);                    break;
    case 0x8e: 
        operand = bus_read(gb, gb->cpu.hl.val);
        ADD(operand, gb->cpu.af.flag.c);
        break;
    case 0x8f: ADD(gb->cpu.af.a, gb->cpu.af.flag.c);                    break;
    case 0x90: SUB(gb->cpu.bc.b, 0);                                    break;
    case 0x91: SUB(gb->cpu.bc.c, 0);                                    break;
    case 0x92: SUB(gb->cpu.de.d, 0);                                    break;
    case 0x93: SUB(gb->cpu.de.e, 0);                                    break;
    case 0x94: SUB(gb->cpu.hl.h, 0);                                    break;
    case 0x95: SUB(gb->cpu.hl.l, 0);                                    break;
    case 0x96:
        operand = bus_read(gb, gb->cpu.hl.val);
        SUB(operand, 0);      
        break;
    case 0x97: SUB(gb->cpu.af.a, 0);                                    break;
    case 0x98: SUB(gb->cpu.bc.b, gb->cpu.af.flag.c);                    break;
    case 0x99: SUB(gb->cpu.bc.c, gb->cpu.af.flag.c);                    break;
    case 0x9a: SUB(gb->cpu.de.d, gb->cpu.af.flag.c);                    break;
    case 0x9b: SUB(gb->cpu.de.e, gb->cpu.af.flag.c);                    break;
    case 0x9c: SUB(gb->cpu.hl.h, gb->cpu.af.flag.c);                    break;
    case 0x9d: SUB(gb->cpu.hl.l, gb->cpu.af.flag.c);                    break;
    case 0x9e:
        operand = bus_read(gb, gb->cpu.hl.val);
        SUB(operand, gb->cpu.af.flag.c);
        break;
    case 0x9f: SUB(gb->cpu.af.a, gb->cpu.af.flag.c);                    break;
    case 0xa0: AND(gb->cpu.bc.b);                                       break;
    case 0xa1: AND(gb->cpu.bc.c);                                       break;
    case 0xa2: AND(gb->cpu.de.d);                                       break;
    case 0xa3: AND(gb->cpu.de.e);                                       break;
    case 0xa4: AND(gb->cpu.hl.h);                                       break;
    case 0xa5: AND(gb->cpu.hl.l);                                       break;
    case 0xa6: 
        operand = bus_read(gb, gb->cpu.hl.val);
        AND(operand);
        break;
    case 0xa7: AND(gb->cpu.af.a);                                       break;
    case 0xa8: XOR(gb->cpu.bc.b);                                       break;
    case 0xa9: XOR(gb->cpu.bc.c);                                       break;
    case 0xaa: XOR(gb->cpu.de.d);                                       break;
    case 0xab: XOR(gb->cpu.de.e);                                       break;
    case 0xac: XOR(gb->cpu.hl.h);                                       break;
    case 0xad: XOR(gb->cpu.hl.l);                                       break;
    case 0xae: 
        operand = bus_read(gb, gb->cpu.hl.val);
        XOR(operand);
        break;
    case 0xaf: XOR(gb->cpu.af.a);                                       break;
    case 0xb0: OR(gb->cpu.bc.b);                                        break;
    case 0xb1: OR(gb->cpu.bc.c);                                        break;
    case 0xb2: OR(gb->cpu.de.d);                                        break;
    case 0xb3: OR(gb->cpu.de.e);                                        break;
    case 0xb4: OR(gb->cpu.hl.h);                                        break;
    case 0xb5: OR(gb->cpu.hl.l);                                        break;
    case 0xb6: 
        operand = bus_read(gb, gb->cpu.hl.val);
        OR(operand);
        break;
    case 0xb7: OR(gb->cpu.af.a);                                        break;
    case 0xb8: CP(gb->cpu.bc.b);                                        break;
    case 0xb9: CP(gb->cpu.bc.c);                                        break;
    case 0xba: CP(gb->cpu.de.d);                                        break;
    case 0xbb: CP(gb->cpu.de.e);                                        break;
    case 0xbc: CP(gb->cpu.hl.h);                                        break;
    case 0xbd: CP(gb->cpu.hl.l);                                        break;
    case 0xbe:
        operand = bus_read(gb, gb->cpu.hl.val);
        CP(operand);
        break;
    case 0xbf: CP(gb->cpu.af.a);                                        break;
    case 0xc0: RET(opcode, !gb->cpu.af.flag.z);                         break;
    case 0xc1: gb->cpu.bc.val = sm83_pop_word(gb);                      break;
    case 0xc2: 
        operand = sm83_fetch_word(gb);
        JP(operand, 0, !gb->cpu.af.flag.z);
        break;
    case 0xc3: 
        operand = sm83_fetch_word(gb);
        JP(operand, 0, 1);
        break; 
    case 0xc4:
        operand = sm83_fetch_word(gb);
        CALL(operand, !gb->cpu.af.flag.z);
        break;
    case 0xc5: PUSH_RR(gb->cpu.bc.val);                                 break;
    case 0xc6: 
        operand = SM83_FETCH_BYTE();
        ADD(operand, 0);
        break;
    case 0xc7: RST_N(0x00);                                             break;
    case 0xc8: RET(opcode, gb->cpu.af.flag.z);                          break;
    case 0xc9: RET(opcode, 1);                                          break;
    case 0xca: 
        operand = sm83_fetch_word(gb);
        JP(operand, 0, gb->cpu.af.flag.z);
        break;
    case 0xcb:
        operand = SM83_FETCH_BYTE();
        gb->executed_cycle += execute_cb_instructions(gb, operand);
        break;
    case 0xcc:
        operand = sm83_fetch_word(gb);
        CALL(operand, gb->cpu.af.flag.z);
        break;
    case 0xcd: 
        operand = sm83_fetch_word(gb);
        CALL(operand, 1);
        break;
    case 0xce: 
        operand = SM83_FETCH_BYTE();
        ADD(operand, gb->cpu.af.flag.c);
        break;
    case 0xcf: RST_N(0x08);                                             break;
    case 0xd0: RET(opcode, !gb->cpu.af.flag.c);                         break;
    case 0xd1: gb->cpu.de.val = sm83_pop_word(gb);                      break;
    case 0xd2: 
        operand = sm83_fetch_word(gb);
        JP(operand, 0, !gb->cpu.af.flag.c);
        break;
    case 0xd4: 
        operand = sm83_fetch_word(gb);
        CALL(operand, !gb->cpu.af.flag.c);
        break;
    case 0xd5: PUSH_RR(gb->cpu.de.val);                                 break;
    case 0xd6:
        operand = SM83_FETCH_BYTE();
        SUB(operand, 0);
        break;
    case 0xd7: RST_N(0x10);                                             break;
    case 0xd8: RET(opcode, gb->cpu.af.flag.c);                          break;
    case 0xd9: RETI();                                                  break;
    case 0xda:
        operand = sm83_fetch_word(gb);
        JP(operand, 0, gb->cpu.af.flag.c);
        break;
    case 0xdc: 
        operand = sm83_fetch_word(gb);
        CALL(operand, gb->cpu.af.flag.c);
        break;
    case 0xde:
        operand = SM83_FETCH_BYTE();
        SUB(operand, gb->cpu.af.flag.c);
        break;
    case 0xdf: RST_N(0x18);                                             break;
    case 0xe0: 
        operand = SM83_FETCH_BYTE();
        LDH_INDIRECT_N_A(operand);
        break;
    case 0xe1: gb->cpu.hl.val = sm83_pop_word(gb);                      break;
    case 0xe2: LDH_INDIRECT_C_A();                                      break;
    case 0xe5: PUSH_RR(gb->cpu.hl.val);                                 break;
    case 0xe6: 
        operand = SM83_FETCH_BYTE();
        AND(operand);
        break;
    case 0xe7: RST_N(0x20);                                             break;
    case 0xe8: 
        operand = SM83_FETCH_BYTE();
        ADD_SP_I8(operand);
        break;
    case 0xe9: gb->cpu.pc = gb->cpu.hl.val;                             break;
    case 0xea:
        operand = sm83_fetch_word(gb);
        bus_write(gb, operand, gb->cpu.af.a);
        break;
    case 0xee: 
        operand = SM83_FETCH_BYTE();
        XOR(operand);
        break;
    case 0xef: RST_N(0x28);                                             break;
    case 0xf0: 
        operand = SM83_FETCH_BYTE();
        LDH_A_INDIRECT_N(operand);
        break;
    case 0xf1: 
        operand = sm83_pop_word(gb);
        gb->cpu.af.val = (operand & 0xfff0) & ~0x000f;
        break;
    case 0xf2: LDH_A_INDIRECT_C();                                      break;
    case 0xf3: DI();                                                    break;
    case 0xf5: PUSH_RR(gb->cpu.af.val);                                 break;
    case 0xf6:
        operand = SM83_FETCH_BYTE();
        OR(operand);
        break;
    case 0xf7: RST_N(0x30);                                             break;
    case 0xf8: 
        operand = SM83_FETCH_BYTE();
        LD_HL_SP_PLUS_I8(operand);
        break;
    case 0xf9: gb->cpu.sp = gb->cpu.hl.val;                             break;
    case 0xfa: 
        operand = sm83_fetch_word(gb);
        gb->cpu.af.a = bus_read(gb, operand);
        break;
    case 0xfb: EI();                                                    break;
    case 0xfe: 
        operand = SM83_FETCH_BYTE();
        CP(operand);
        break;
    case 0xff: RST_N(0x38);                                             break;
    default:
        printf("Unknown opcode 0x%02x\n", opcode);
        break;
    }
    
    /* timer handling */
    gb->timer.div += gb->executed_cycle * 4;
    if ((gb->timer.div >= div_bit_to_freq[gb->timer.tac.freq]) && (gb->timer.tac.enable)) {
        if (gb->timer.tima == 0xff)
            INTERRUPT_REQUEST(INTR_SRC_TIMER);
        gb->timer.tima = (gb->timer.tima == 0xff) ? gb->timer.tma : gb->timer.tima + 1;
    }

    /* oam dma handling */
    if (gb->dma.mode != OFF) {
        gb->dma.tick += gb->executed_cycle;
        if (gb->dma.tick >= 1 && gb->dma.mode == WAITING) {
            gb->dma.tick -= 1;
            gb->dma.mode = TRANSFERING;
        } else if (gb->dma.tick >= 160 && gb->dma.mode == TRANSFERING) {
            for (int i = 0; i < 160; i++) {
                uint8_t transfer_val = bus_read(gb, gb->dma.start_addr + i);
                bus_write(gb, 0xfe00 + i, transfer_val);
            }
            gb->dma.mode = OFF;
        }
    }

    /* ppu handling */
    if (!gb->ppu.lcdc.ppu_enable)
        return;
    gb->ppu.ticks += gb->executed_cycle * 4;
    if (gb->ppu.ly <= 143) {
        if (gb->ppu.ticks <= 80) {
            SET_MODE(OAM_SCAN);
        } else if (gb->ppu.ticks <= 252) {
            SET_MODE(DRAWING);
        } else if (gb->ppu.ticks <= 456) {
            SET_MODE(HBLANK);
        }
    }
    if (gb->ppu.ticks > 456) {
        gb->ppu.scan_line_ready = true;
        gb->ppu.ticks -= 456;
        if (gb->ppu.ly <= 143) {
            // ppu_oam_scan
            for (int i = 0; i < 40; i++) {
                if (gb->oam[i * 4 + 1] > 0 && gb->ppu.oam_entry_cnt < 10 &&
                    IN_RANGE(gb->ppu.ly, gb->oam[i * 4] - 16, gb->oam[i * 4] - 17 + gb->ppu.lcdc.obj_size)) {
                    gb->ppu.oam_entry[gb->ppu.oam_entry_cnt].y = gb->oam[i * 4];
                    gb->ppu.oam_entry[gb->ppu.oam_entry_cnt].x = gb->oam[i * 4 + 1];
                    gb->ppu.oam_entry[gb->ppu.oam_entry_cnt].tile_index = gb->oam[i * 4 + 2];
                    gb->ppu.oam_entry[gb->ppu.oam_entry_cnt].attributes.val = gb->oam[i * 4 + 3];
                    gb->ppu.oam_entry_cnt++;
                }
                if (gb->ppu.oam_entry_cnt == 10)
                    break;
            }
            qsort(gb->ppu.oam_entry, gb->ppu.oam_entry_cnt, sizeof(struct oam_entry), cmpfunc);

            // draw the scanline
            ppu_draw_scanline(gb);
            SET_MODE(HBLANK);
        }

        // hblank & vblank handler
        if (gb->ppu.mode == HBLANK) {
            gb->ppu.ly++;
            gb->ppu.stat_intr_src.val = 0;
            gb->ppu.stat.lyc_equal_ly = gb->ppu.ly == gb->ppu.lyc;
            if (gb->ppu.ly == 144) {
                SET_MODE(VBLANK);
                if (gb->ppu.lcdc.ppu_enable)
                    INTERRUPT_REQUEST(INTR_SRC_VBLANK);
                gb->ppu.frame_ready = true;
                gb->ppu.window_line_cnt = 0;
                gb->ppu.draw_window_this_line = false;
                gb->ppu.window_in_frame = false;
            } else {
                if (gb->ppu.wy == gb->ppu.ly && !gb->ppu.window_in_frame)
                    gb->ppu.window_in_frame = true;
                SET_MODE(OAM_SCAN);
                if (gb->ppu.draw_window_this_line) {
                    gb->ppu.window_line_cnt++;
                    gb->ppu.draw_window_this_line = false;
                }
            }
            gb->ppu.ticks = 0;
            gb->ppu.oam_entry_cnt = 0;
        } else if (gb->ppu.mode == VBLANK) {
            if (gb->ppu.ly == 153) {
                gb->ppu.ly = 0;
                if (gb->ppu.wy == gb->ppu.ly && !gb->ppu.window_in_frame)
                    gb->ppu.window_in_frame = true;
                gb->ppu.oam_entry_cnt = 0;
                SET_MODE(OAM_SCAN);
                gb->ppu.stat_intr_src.val = 0;
            } else {
                gb->ppu.ly++;
            }
            gb->ppu.stat.lyc_equal_ly = gb->ppu.ly == gb->ppu.lyc;
            gb->ppu.ticks = 0;
        }
    }
    if (gb->interrupt.ie & INTR_SRC_LCD) {
        stat_intr_line = ((gb->ppu.stat.mode0_int_select && gb->ppu.stat.ppu_mode == HBLANK) ||
        (gb->ppu.stat.mode1_int_select && gb->ppu.stat.ppu_mode == VBLANK) ||
        (gb->ppu.stat.mode2_int_select && gb->ppu.stat.ppu_mode == OAM_SCAN) || 
        (gb->ppu.stat.lyc_int_select && gb->ppu.stat.lyc_equal_ly));
        if (!gb->ppu.stat_intr_line && stat_intr_line)
            INTERRUPT_REQUEST(INTR_SRC_LCD);
        gb->ppu.stat_intr_line = stat_intr_line;
    }
//        ppu_check_stat_intr(gb);

    /* deal with interrupt */
    is_interrupt = IS_INTERRUPT_PENDING();
    if (is_interrupt)
        gb->mode = (!gb->cpu.ime) ? HALT_BUG : NORMAL;
    gb->interrupt.interrupt_handled = gb->cpu.ime && is_interrupt;
    if (gb->interrupt.interrupt_handled) {
        if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_VBLANK) == INTR_SRC_VBLANK)
            interrupt_handler(gb, INTR_SRC_VBLANK);
        else if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_LCD) == INTR_SRC_LCD)
            interrupt_handler(gb, INTR_SRC_LCD);
        else if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_TIMER) == INTR_SRC_TIMER)
            interrupt_handler(gb, INTR_SRC_TIMER);
        else if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_SERIAL) == INTR_SRC_SERIAL)
            interrupt_handler(gb, INTR_SRC_SERIAL);
        else if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_JOYPAD) == INTR_SRC_JOYPAD)
            interrupt_handler(gb, INTR_SRC_JOYPAD);
    }
}

/**********************************************************************************************/
/************************************* PPU related parts **************************************/
/**********************************************************************************************/

#define GET_COLOR(which_pal, color_id)        \
    palette[(gb->ppu.pal[(which_pal)] >> ((color_id) * 2)) & 0x03]
// uint16_t get_color_from_palette(struct gb *gb, palette_t pal, uint8_t color_id)
// {
//     uint32_t ret;

//     switch (pal) {
//     case BGP:
//         ret =  palette[(gb->ppu.bgp >> (color_id * 2)) & 0x03];
//         break;
//     case OBP0:
//         ret =  palette[(gb->ppu.obp0 >> (color_id * 2)) & 0x03];
//         break;
//     case OBP1:
//         ret =  palette[(gb->ppu.obp1 >> (color_id * 2)) & 0x03];
//         break;
//     default:
//         break;
//     }
//     return ret;
// }

#define READ_VRAM(addr)         \
    gb->vram[addr - 0x8000]

int cmpfunc(const void *a, const void *b)
{
    struct oam_entry *sa = (struct oam_entry *)a;
    struct oam_entry *sb = (struct oam_entry *)b;

    return (sa->x - sb->x);
}

void ppu_draw_scanline(struct gb *gb)
{
    uint8_t tile_index, color_id_low, color_id_high, color_id,
            offset_x, offset_y, x_pos, y_pos, sprite_color_id;
    bool is_window_pixel;
    uint16_t tile_map_addr, tile_addr, color;
    static int window_offset = 0;
    pixel_type_t ptype;

    for (int i = 0; i < SCREEN_WIDTH; i++) {
        // deal with bg and window
        if (!gb->ppu.lcdc.bg_win_enable)
            goto set_frame_buffer;
        is_window_pixel = gb->ppu.draw_window_this_line = gb->ppu.lcdc.win_enable && gb->ppu.window_in_frame && (i + 7 >= gb->ppu.wx);
        tile_map_addr = (is_window_pixel) ? gb->ppu.lcdc.win_tile_map : gb->ppu.lcdc.bg_tile_map;
        offset_x = (!is_window_pixel) ? (i + gb->ppu.scx) & 0xff : window_offset;
        offset_y = (!is_window_pixel) ? (gb->ppu.ly + gb->ppu.scy) & 0xff : gb->ppu.window_line_cnt;
        ptype = BG_WIN;
//        if (!(i % 8)) {
            tile_index = READ_VRAM(tile_map_addr + ((offset_x / 8 + 32 * (offset_y / 8)) & 0x3ff));
            tile_addr = (gb->ppu.lcdc.bg_win_tiles == 0x8000)
                        ? 0x8000 + 16 * (uint8_t)tile_index : 0x9000 + 16 * (int8_t)tile_index;
//        }
        if (is_window_pixel)
            offset_x = window_offset++;
        color_id_low = (READ_VRAM(tile_addr + (offset_y % 8) * 2) >> (7 - (offset_x % 8))) & 0x01;
        color_id_high = (READ_VRAM(tile_addr + (offset_y % 8) * 2 + 1) >> (7 - (offset_x % 8))) & 0x01;
        color_id = color_id_low | (color_id_high << 1);
set_frame_buffer:
        gb->frame_buffer[i + gb->ppu.ly * SCREEN_WIDTH] = (gb->ppu.lcdc.bg_win_enable) 
                                                                    ? GET_COLOR(BGP, color_id_low | (color_id_high << 1)) : palette[0];
        // gb->line_buffer[i] = (gb->ppu.lcdc.bg_win_enable) ? get_color_from_palette(gb, BGP, color_id_low | (color_id_high << 1)) : palette[0];
        // gb->frame_buffer[i * 2 + gb->ppu.ly * 2 * SCREEN_WIDTH] = MSB(color);
        // gb->frame_buffer[i * 2 + 1 + gb->ppu.ly * 2 * SCREEN_WIDTH] = LSB(color);
         
        // deal with sprite
        if (!gb->ppu.lcdc.obj_enable)
            continue;
        for (int j = gb->ppu.oam_entry_cnt - 1; j >= 0; j--) {
            if (!IN_RANGE(i, gb->ppu.oam_entry[j].x - 8, gb->ppu.oam_entry[j].x))
                continue;
            tile_index = gb->ppu.oam_entry[j].tile_index;
            x_pos = i - (gb->ppu.oam_entry[j].x - 8);
            y_pos = (gb->ppu.ly - (gb->ppu.oam_entry[j].y - 16)) % 16;
            offset_x = (gb->ppu.oam_entry[j].attributes.x_flip) ? x_pos : 7 - x_pos;
            offset_y = (!gb->ppu.oam_entry[j].attributes.y_flip) ? y_pos : gb->ppu.lcdc.obj_size - 1 - (y_pos);
            if (gb->ppu.lcdc.obj_size == 16 && y_pos >= 8)  // bottom
                tile_index = (gb->ppu.oam_entry[j].attributes.y_flip) ?  tile_index & 0xfe : tile_index | 0x01;
            else if (gb->ppu.lcdc.obj_size == 16 && y_pos <= 7) // top
                tile_index = (gb->ppu.oam_entry[j].attributes.y_flip) ?  tile_index | 0x01 : tile_index & 0xfe;
            tile_addr = 0x8000 + 16 * (uint8_t)tile_index;
            uint8_t test = (!gb->ppu.oam_entry[j].attributes.y_flip) ? (y_pos % 8) : 7 - (y_pos % 8);
            // color_id_low = (READ_VRAM(tile_addr + (offset_y) * 2) >> (offset_x)) & 0x01;
            // color_id_high = (READ_VRAM(tile_addr + (offset_y) * 2 + 1) >> (offset_x)) & 0x01;
            color_id_low = (READ_VRAM(tile_addr + (test) * 2) >> (offset_x)) & 0x01;
            color_id_high = (READ_VRAM(tile_addr + (test) * 2 + 1) >> (offset_x)) & 0x01;
            sprite_color_id = color_id_low | (color_id_high << 1);
            if (((ptype == BG_WIN) && (!sprite_color_id || (sprite_color_id > 0 && gb->ppu.oam_entry[j].attributes.priority && color_id > 0))) ||
                ((ptype == SPRITE) && (color_id > 0 && !sprite_color_id)))
                continue;
            gb->frame_buffer[i + gb->ppu.ly * SCREEN_WIDTH] = 
                            GET_COLOR(gb->ppu.oam_entry[j].attributes.dmg_palette, sprite_color_id);
            color_id = sprite_color_id;
            ptype = SPRITE;
        }
    }
    window_offset = 0;
}

void ppu_check_stat_intr(struct gb *gb)
{
    bool stat_intr_line = 0;

    stat_intr_line = ((gb->ppu.stat.mode0_int_select && gb->ppu.stat.ppu_mode == HBLANK) ||
        (gb->ppu.stat.mode1_int_select && gb->ppu.stat.ppu_mode == VBLANK) ||
        (gb->ppu.stat.mode2_int_select && gb->ppu.stat.ppu_mode == OAM_SCAN) || 
        (gb->ppu.stat.lyc_int_select && gb->ppu.stat.lyc_equal_ly));
    if (!gb->ppu.stat_intr_line && stat_intr_line)
        INTERRUPT_REQUEST(INTR_SRC_LCD);
    gb->ppu.stat_intr_line = stat_intr_line;
}

int interrupt_handler(struct gb *gb, uint8_t intr_src)
{
    gb->cpu.ime = false;
    gb->interrupt.flag &= ~intr_src;
    sm83_push_word(gb, gb->cpu.pc);
    gb->cpu.pc = interrupt_vector[intr_src];
    return 5;
}

void interrupt_process(struct gb *gb)
{
    bool is_interrupt = IS_INTERRUPT_PENDING();

    if (is_interrupt)
        gb->mode = (!gb->cpu.ime) ? HALT_BUG : NORMAL;
    gb->interrupt.interrupt_handled = gb->cpu.ime && is_interrupt;
    if (gb->interrupt.interrupt_handled) {
        if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_VBLANK) == INTR_SRC_VBLANK)
            interrupt_handler(gb, INTR_SRC_VBLANK);
        else if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_LCD) == INTR_SRC_LCD)
            interrupt_handler(gb, INTR_SRC_LCD);
        else if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_TIMER) == INTR_SRC_TIMER)
            interrupt_handler(gb, INTR_SRC_TIMER);
        else if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_SERIAL) == INTR_SRC_SERIAL)
            interrupt_handler(gb, INTR_SRC_SERIAL);
        else if ((gb->interrupt.ie & gb->interrupt.flag & INTR_SRC_JOYPAD) == INTR_SRC_JOYPAD)
            interrupt_handler(gb, INTR_SRC_JOYPAD);
    }
}


/**********************************************************************************************/
/************************************* timer related parts ************************************/
/**********************************************************************************************/

/**********************************************************************************************/
/********************************** cartridge related parts ***********************************/
/**********************************************************************************************/

const char *cart_types[] = {
	[0x00] = "ROM ONLY",
	[0x01] = "MBC1",
	[0x02] = "MBC1+RAM",
	[0x03] = "MBC1+RAM+BATTERY",
	[0x05] = "MBC2",
	[0x06] = "MBC2+BATTERY",
	[0x08] = "ROM+RAM",
	[0x09] = "ROM+RAM+BATTERY",
	[0x0B] = "MMM01",
	[0x0C] = "MMM01+RAM",
	[0x0D] = "MMM01+RAM+BATTERY",
	[0x0F] = "MBC3+TIMER+BATTERY",
	[0x10] = "MBC3+TIMER+RAM+BATTERY",
	[0x11] = "MBC3",
	[0x12] = "MBC3+RAM",
	[0x13] = "MBC3+RAM+BATTERY",
	[0x15] = "MBC4",
	[0x16] = "MBC4+RAM",
	[0x17] = "MBC4+RAM+BATTERY",
	[0x19] = "MBC5",
	[0x1A] = "MBC5+RAM",
	[0x1B] = "MBC5+RAM+BATTERY",
	[0x1C] = "MBC5+RUMBLE",
	[0x1D] = "MBC5+RUMBLE+RAM",
	[0x1E] = "MBC5+RUMBLE+RAM+BATTERY",
	[0xFC] = "POCKET CAMERA",
	[0xFD] = "BANDAI TAMA5",
	[0xFE] = "HuC3",
	[0xFF] = "HuC1+RAM+BATTERY",
};

const char *rom_size[12] = {
    "32 KiB",
    "64 KiB",
    "128 KiB",
    "256 KiB",
    "512 KiB",
    "1 MiB",
    "2 MiB",
    "4 MiB",
    "8 MiB",
    "1.1 MiB",
    "1.2 MiB",
    "1.5 MiB",
};

const char *sram_size[6] = {
    "0",
    "0",
    "8 KiB",
    "32 KiB",
    "128 KiB",
    "64 KiB",
};

static int sram_size_num[] = {
    0, 0, 8 * KiB, 32 * KiB, 128 * KiB, 64 * KiB
};

void cartridge_get_infos(struct gb *gb)
{
    if (!gb->cart.cartridge_loaded) {
        printf("Error: No cartridge found.\n");
        return;
    }
    for (uint16_t i = 0x0134, j = 0; i <= 0x0143; i++) {
        if (gb->cart.rom[i] != 0x00)
            gb->cart.infos.name[j++] = gb->cart.rom[i];
        else {
            gb->cart.infos.name[j] = '\0';
            break;
        }
    }
    gb->cart.infos.type = gb->cart.rom[0x0147];
    // if (gb->cart.infos.type == MBC1_RAM_BATTERY)
    //     gb->mbc.mbc1.has_battery = true;
    gb->cart.infos.rom_size = 32 * KiB * (1 << gb->cart.rom[0x0148]);
    gb->cart.infos.ram_size = sram_size_num[gb->cart.rom[0x0149]];
    gb->cart.infos.bank_size = gb->cart.infos.rom_size / (16 * KiB);
}    

void cartridge_print_info(struct gb *gb)
{
    // print cartridge infos
    printf("name: %s\n", gb->cart.infos.name);
    printf("mbc type: %s\n", cart_types[gb->cart.infos.type]);
    printf("ROM size: %s\n", rom_size[gb->cart.rom[0x0148]]);
    printf("RAM size: %s\n", sram_size[gb->cart.rom[0x0149]]);
    printf("bank size: %d\n", gb->cart.infos.bank_size);
}

void cartridge_load(struct gb *gb, const uint8_t *rom, unsigned int rom_size)
{
    gb->cart.rom = rom;
    gb->cart.cartridge_loaded = true;
    cartridge_get_infos(gb);
    cartridge_print_info(gb);
}

void load_state_after_booting(struct gb *gb)
{
    // make code shorter, easier to read
    struct sm83 *cpu = &gb->cpu;
    struct ppu *ppu = &gb->ppu;
    struct interrupt *interrupt = &gb->interrupt;
    struct timer *timer = &gb->timer;
    struct dma *dma = &gb->dma;
    struct joypad *joypad = &gb->joypad;
    struct mbc *mbc = &gb->mbc;
    struct serial *serial = &gb->serial;

    gb->mode = NORMAL;

    for (int i = 0; i < 160 * 144; i++)
        gb->frame_buffer[i] = COLOR_DARKGRAY;
    for (int i = 0; i < 160; i++)
        gb->line_buffer[i] = COLOR_DARKGRAY;

    // cpu
    cpu->pc = 0x100;
    cpu->af.a = 0x01;
    cpu->af.f = 0x80;
    cpu->bc.val = 0x0013;
    cpu->de.val = 0x00d8;
    cpu->hl.val = 0x014d;
    cpu->sp = 0xfffe; 

    // interrupt
    interrupt->flag = 0xe1;
    interrupt->ie = 0x00;

    // timer
    timer->div = 0xab;
    timer->tima = 0x00;
    timer->tma = 0x00;
    timer->tac.val = 0xf8;
    timer->old_edge = 0;

    // serial
    serial->sb = 0x00;
    serial->sc = 0x7e;

    // ppu
    ppu->lcdc.val = 0x91;
    uint8_t val = ppu->lcdc.val;
    ppu->lcdc.ppu_enable = BIT(val, 7);
    ppu->lcdc.win_tile_map = 0x9800 | (BIT(val, 6) << 10);
    ppu->lcdc.win_enable = BIT(val, 5);
    ppu->lcdc.bg_win_tiles = 0x8800 - (0x800 * BIT(val, 4));
    ppu->lcdc.bg_tile_map = 0x9800 | (BIT(val, 3) << 10);
    ppu->lcdc.obj_size = 0x08 << BIT(val, 2);
    ppu->lcdc.obj_enable = BIT(val, 1);
    ppu->lcdc.bg_win_enable = BIT(val, 0);
    ppu->stat.val = 0x85;
    ppu->scy = 0x00;
    ppu->scx = 0x00;
    ppu->ly = 0x00;
    ppu->lyc = 0x00;
    ppu->pal[BGP] = 0xfc;
    ppu->wy = 0x00;
    ppu->wx = 0x00;
    ppu->ticks = 0;
    ppu->mode = OAM_SCAN;
    ppu->frame_ready = false;
    ppu->scan_line_ready = false;
    ppu->oam_entry_cnt = 0;
    ppu->sprite_cnt = 0;
    ppu->stat_intr_line = false;
    ppu->stat_intr_src.val = 0;
    ppu->window_in_frame = false;
    ppu->window_line_cnt = 0;
    ppu->draw_window_this_line = false;

    // dma
    dma->mode = OFF;
    dma->reg = 0xff;

    // // joypad
    // joypad->a = 1;
    // joypad->b = 1;
    // joypad->select = 1;
    // joypad->start = 1;
    // joypad->up = 1;
    // joypad->down = 1;
    // joypad->left = 1;
    // joypad->right = 1;
    // joypad->joyp.val = 0xcf;

    // // mbc1
    // mbc->mbc1.ram_enable = 0;
    // mbc->mbc1.banking_mode = 0;
    // mbc->mbc1.rom_bank = 0;
    // mbc->mbc1.ram_bank = 0;
    // mbc->mbc1.has_battery = false;

    // // mbc3
    // mbc->mbc3.ram_enable = false;
    // mbc->mbc3.rom_bank = 0;
    // mbc->mbc3.ram_bank = 0;



    // // TODO: update each channel after booting status 
    // //       when implementing them. 

    // apu->tick = 0;
    // apu->frame_sequencer = 0;

    // // apu square1
    // apu->sqr1.name = SQUARE1;
    // apu->sqr1.is_active = true;
    // apu->sqr1.regs.nrx0 = 0x80;
    // apu->sqr1.regs.nrx1 = 0xbf;
    // apu->sqr1.regs.nrx2 = 0xf3;
    // apu->sqr1.regs.nrx3 = 0xff;
    // apu->sqr1.regs.nrx4 = 0xbf;
    // apu->sqr1.is_dac_on = is_dac_on(&gb->apu.sqr1);
    // apu->sqr1.length.counter = get_length_load(&gb->apu.sqr1);
    // apu->sqr1.timer = get_frequency(&gb->apu.sqr1);
    // apu->sqr1.left_output = true;
    // apu->sqr1.right_output = true;
    // apu->sqr1.volume_envelope.add_mode = get_envelope_add_mode(&gb->apu.sqr1);
    // apu->sqr1.volume = 15;
    // apu->sqr1.pos = 0;
    // apu->sqr1.volume_envelope.period = get_envelope_period(&gb->apu.sqr1);
    // apu->sqr1.frequency_sweep.period = get_sweep_period(&gb->apu.sqr1);
    // apu->sqr1.frequency_sweep.negate = BIT(gb->apu.sqr1.regs.nrx0, 3);
    // apu->sqr1.frequency_sweep.shift = get_sweep_shift(&gb->apu.sqr1);
    // apu->sqr1.frequency_sweep.is_active = true;
    // apu->sqr1.frequency_sweep.timer = gb->apu.sqr1.frequency_sweep.period;
    // apu->sqr1.frequency_sweep.shadow_frequency = get_frequency(&gb->apu.sqr1);

    // // apu square2
    // apu->sqr2.name = SQUARE2;
    // apu->sqr2.is_active = false;
    // apu->sqr2.regs.nrx1 = 0x3f;
    // apu->sqr2.regs.nrx2 = 0x00;
    // apu->sqr2.regs.nrx3 = 0xff;
    // apu->sqr2.regs.nrx4 = 0xbf;
    // apu->sqr2.is_dac_on = is_dac_on(&gb->apu.sqr2);
    // apu->sqr2.length.counter = 0;
    // apu->sqr2.timer = 0;
    // apu->sqr2.left_output = true;
    // apu->sqr2.right_output = true;
    // apu->sqr2.volume_envelope.add_mode = get_envelope_add_mode(&gb->apu.sqr2);
    // apu->sqr2.volume = 0;
    // apu->sqr2.pos = 0;
    // apu->sqr2.volume_envelope.period = get_envelope_period(&gb->apu.sqr2);

    // // apu wave
    // apu->wave.name = WAVE;
    // apu->wave.is_active = false;
    // apu->wave.regs.nrx0 = 0x7f;
    // apu->wave.regs.nrx1 = 0xff;
    // apu->wave.regs.nrx2 = 0x9f;
    // apu->wave.regs.nrx3 = 0xff;
    // apu->wave.regs.nrx4 = 0xbf;
    // apu->wave.is_dac_on = BIT(gb->apu.wave.regs.nrx0, 7);
    // apu->wave.left_output = true;
    // apu->wave.right_output = false;
    // apu->wave.pos = 0;
    // apu->wave.is_dac_on = is_dac_on(&apu->wave);

    // // apu noise
    // apu->noise.name = NOISE;
    // apu->noise.is_active = false;
    // apu->noise.regs.nrx1 = 0xff;
    // apu->noise.regs.nrx2 = 0x00;
    // apu->noise.regs.nrx3 = 0x00;
    // apu->noise.regs.nrx4 = 0xbf;
    // apu->noise.left_output = true;
    // apu->noise.right_output = false;
    // apu->noise.is_dac_on = is_dac_on(&gb->apu.noise);
    // apu->noise.lfsr.clock_shift = get_noise_clock_shift(&gb->apu.noise);
    // apu->noise.lfsr.divisor = get_noise_divisor(&gb->apu.noise);
    // apu->noise.lfsr.width_mode = get_noise_width_mode(&gb->apu.noise);
    // apu->noise.timer = apu->noise.lfsr.divisor << apu->noise.lfsr.clock_shift;
    // apu->noise.volume = get_envelope_volume(&gb->apu.noise);
    // apu->noise.volume_envelope.add_mode = get_envelope_add_mode(&gb->apu.noise);
    // apu->noise.volume_envelope.period = get_envelope_period(&gb->apu.noise);
    // apu->noise.lfsr.reg = 0x7fff;

    // // apu global control
    // apu->ctrl.name = CTRL;
    // apu->ctrl.regs.nrx0 = 0x77;
    // apu->ctrl.regs.nrx1 = 0xf3;
    // apu->ctrl.regs.nrx2 = 0xf1;

    // // sound panning
    // apu->noise.left_output = true;
    // apu->wave.left_output = true;
    // apu->sqr2.left_output = true;
    // apu->sqr1.left_output = true;
    // apu->noise.right_output = false;
    // apu->wave.right_output = false;
    // apu->sqr2.right_output = true;
    // apu->sqr1.right_output = true;

    // apu->sample_buffer.ptr = 0;
    // memset(apu->sample_buffer.buf, 0, BUFFER_SIZE * sizeof(int16_t));
    // apu->sample_buffer.is_full = false;
}

/**********************************************************************************************/
/************************************* bus related parts **************************************/
/**********************************************************************************************/

uint8_t bus_read(struct gb *gb, uint16_t addr)
{
    uint8_t ret = 0xff;

    switch (GET_MEM_REGION(addr)) {
    case ROM:
        ret = gb->cart.rom[addr];
        break;
    case VRAM:
        ret = gb->vram[addr - 0x8000];
        break;
    case EXTERN_RAM:
        ret = gb->extern_ram[addr - 0xa000];
        break;
    case WRAM:
        ret = gb->wram[addr - 0xc000];
        break;
    case ECHO_RAM:
        ret = gb->wram[(addr & 0xddff) - 0xc000];
        break;
    case OAM:
        ret = gb->oam[addr - 0xfe00];
        break;
    case UNUSED:
        ret = gb->unused[addr - 0xfea0];
        break;
    case IO:
        switch (WHICH_IO_REGION(addr)) {
        case TIMER:
            switch (addr) {
            case TIM_REG_DIV:
                ret = (gb->timer.div >> 6) & 0x00ff;
                break;
            case TIM_REG_TAC:
                ret = gb->timer.tac.val;
                break;
            case TIM_REG_TIMA:
                ret = gb->timer.tima;
                break;
            case TIM_REG_TMA:
                ret = gb->timer.tma;
                break;
            default:
                break;
            }
        case INTERRUPT:
            switch (addr) {
            case INTR_REG_IE:
                ret = gb->interrupt.ie;
                break;
            case INTR_REG_IF:
                ret = gb->interrupt.flag;
                break;
            default:
                break;
            }
        case PPU:
            switch (addr) {
            case PPU_REG_LCDC:
                ret = gb->ppu.lcdc.val;
                break;
            case PPU_REG_STAT:
                ret = gb->ppu.stat.val;
                break;
            case PPU_REG_SCY:
                ret = gb->ppu.scy;
                break; 
            case PPU_REG_SCX:
                ret = gb->ppu.scx;
                break; 
            case PPU_REG_LY:
                ret = gb->ppu.ly;
                break; 
            case PPU_REG_LYC:
                ret = gb->ppu.lyc;
                break; 
            case PPU_REG_BGP:
                ret = gb->ppu.pal[BGP];
                break;
            case PPU_REG_OBP0:
                ret = gb->ppu.pal[OBP0];
                break;
            case PPU_REG_OBP1:
                ret = gb->ppu.pal[OBP1];
                break;
            case PPU_REG_WY:
                ret = gb->ppu.wy;
                break;
            case PPU_REG_WX:
                ret = gb->ppu.wx;
                break;
            case DMA_REG_OAM:
                ret = gb->dma.reg;
                break;
            default:
                break;
            }
        default:
            break;
        }
        break;
    case HRAM:
        ret = gb->hram[addr - 0xff80];
        break;
    default:
        break;
    }
    return ret;
}

void bus_write(struct gb *gb, uint16_t addr, uint8_t val)
{
    switch (GET_MEM_REGION(addr)) {
    case ROM:
        // TODO: support more MBCs
        break;
    case VRAM:
        gb->vram[addr - 0x8000] = val;
        break;
    case EXTERN_RAM:
        gb->extern_ram[addr - 0xa000] = val;
        break;
    case WRAM:
        gb->wram[addr - 0xc000] = val;
        break;
    case ECHO_RAM:
        gb->wram[(addr & 0xddff) - 0xc000] = val;
        break;
    case OAM:
        gb->oam[addr - 0xfe00] = val;
        break;
    case UNUSED:
        gb->unused[addr - 0xfea0] = val;
        break;
    case IO:
        switch (WHICH_IO_REGION(addr)) {
        case INTERRUPT:
            switch (addr){
            case INTR_REG_IE:
                gb->interrupt.ie = val;
                break;
            case INTR_REG_IF:
                gb->interrupt.flag = val;
                break;
            }
            break;
        case TIMER:
            switch (addr) {
            case TIM_REG_DIV:
                gb->timer.div = 0;
                break;
            case TIM_REG_TAC:
                gb->timer.tac.val = val;
                break;
            case TIM_REG_TIMA:
                gb->timer.tima = val;
                break;
            case TIM_REG_TMA:
                gb->timer.tma = val;
                break;
            default:
                break;
            }
            break;
        case PPU:
            switch (addr) {
                case PPU_REG_LCDC:
                    gb->ppu.lcdc.val = val;
                    // extract informations from LCDC
                    gb->ppu.lcdc.ppu_enable = BIT(val, 7);
                    gb->ppu.lcdc.win_tile_map = 0x9800 | (BIT(val, 6) << 10);
                    gb->ppu.lcdc.win_enable = BIT(val, 5);
                    gb->ppu.lcdc.bg_win_tiles = 0x8800 - (0x800 * BIT(val, 4));
                    gb->ppu.lcdc.bg_tile_map = 0x9800 | (BIT(val, 3) << 10);
                    gb->ppu.lcdc.obj_size = 0x08 << BIT(val, 2);
                    gb->ppu.lcdc.obj_enable = BIT(val, 1);
                    gb->ppu.lcdc.bg_win_enable = BIT(val, 0);
                    if (!gb->ppu.lcdc.ppu_enable) {
                        SET_MODE(HBLANK);
                        gb->ppu.ly = 0;
                    }
                    break;
                case PPU_REG_STAT:
                    gb->ppu.stat.val = (val & 0xf8) | (gb->ppu.stat.val & 0x87);
                    break;
                case PPU_REG_SCY:
                    gb->ppu.scy = val;
                    break; 
                case PPU_REG_SCX:
                    gb->ppu.scx = val;
                    break; 
                case PPU_REG_LYC:
                    gb->ppu.lyc = val;
                    gb->ppu.stat.lyc_equal_ly = gb->ppu.lyc == gb->ppu.ly;
                    break; 
                case PPU_REG_BGP:
                    gb->ppu.pal[BGP] = val;
                    break;
                case PPU_REG_OBP0:
                    gb->ppu.pal[OBP0] = val;
                    break;
                case PPU_REG_OBP1:
                    gb->ppu.pal[OBP1] = val;
                    break;
                case PPU_REG_WY:
                    gb->ppu.wy = val;
                    gb->ppu.window_in_frame = gb->ppu.wy == gb->ppu.ly;
                    break;
                case PPU_REG_WX:
                    gb->ppu.wx = val;
                    break;
                case DMA_REG_OAM:
                    gb->dma.reg = val;
                    gb->dma.mode = WAITING;
                    gb->dma.start_addr = TO_U16(0x00, val);
                    gb->dma.tick = 0;
                    break;
                default:
                    break;
                }
            break;
        default:
            break;
        }
        break;
    case HRAM:
        gb->hram[addr - 0xff80] = val;
        break;
    default:
        break;
    }
}

