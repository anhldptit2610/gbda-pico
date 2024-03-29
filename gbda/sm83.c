#include "sm83.h"

int instr_cycle[] = {
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

int cb_instr_cycle[] = {
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


void sm83_tick(struct gb *gb)
{
    for (int i = 0; i < 4; i++) {
        timer_tick(gb);
        ppu_tick(gb);
        // apu_tick(gb);
    }
}

void sm83_cycle_when_interrupt(struct gb *gb)
{
    // interrupt handler costs 5 M-cycles, each M-cycle = 4 T-cycle
    for (int i = 0; i < 20; i++) {
        timer_tick(gb);
    }
}

bool sm83_cycle(struct gb *gb, int cycles)
{
    bool is_interrupt;
    static int i = 0;

    for (int j = 0; j < cycles; j++) {
        sm83_tick(gb);

        // deal with DMA
        if (gb->dma.mode == WAITING) {
            gb->dma.mode = TRANSFERING;
        } else if (gb->dma.mode == TRANSFERING) {
            gb->oam[i] = dma_get_data(gb, gb->dma.start_addr + i);
            if (i == 0x9f) {
                gb->dma.mode = OFF;
                i = 0;
            } else {
                i++;
            }
        }
    }
    is_interrupt = interrupt_process(gb);
    return is_interrupt;
}

void sm83_init(struct gb *gb)
{
    gb->cpu.pc = 0;
    gb->cart.cartridge_loaded = false;
    for (int i = 0; i < 320; i++) {
        for (int j = 0; j < 480; j += 2) {
            gb->frame_buffer[j + i * 480] = MSB(COLOR_LIGHTGRAY);
            gb->frame_buffer[j + 1 + i * 480] = LSB(COLOR_LIGHTGRAY);
        }
    }
}

uint8_t sm83_fetch_byte(struct gb *gb)
{
    uint8_t ret = 0xff;

    if (gb->mode == HALT) {
        ret = 0x76;
    } else {
        ret = bus_read(gb, gb->cpu.pc++);
    }
    return ret;
}

uint16_t sm83_fetch_word(struct gb *gb)
{
    uint8_t lsb = sm83_fetch_byte(gb);
    uint8_t msb = sm83_fetch_byte(gb);

    return TO_U16(lsb, msb);
}

void sm83_push_byte(struct gb *gb, uint8_t val)
{
    bus_write(gb, --gb->cpu.sp, val);
}

void sm83_push_word(struct gb *gb, uint16_t val)
{
    sm83_push_byte(gb, MSB(val));
    sm83_push_byte(gb, LSB(val));
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

static inline void toggle_znh(struct gb *gb, uint8_t res, bool n, bool h)
{
    gb->cpu.af.flag.z = !res;
    gb->cpu.af.flag.n = n;
    gb->cpu.af.flag.h = h;
}

/* Instructions */

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
    toggle_znh(gb, res, 0, BIT(carry_per_bit, 4));  \
    gb->cpu.af.flag.c = BIT(carry_per_bit, 8)       

#define SUB(b, d)                                                                                               \
    res = gb->cpu.af.a + ~b + 1 - d;                \
    carry_per_bit = res ^ gb->cpu.af.a ^ ~b;        \
    gb->cpu.af.a = res;                             \
    toggle_znh(gb, res, 1, !BIT(carry_per_bit, 4)); \
    gb->cpu.af.flag.c = !BIT(carry_per_bit, 8);    

#define CP(b)                                                                                                   \
    toggle_znh(gb, gb->cpu.af.a + ~b + 1, 1, !BIT(gb->cpu.af.a ^ ~b ^ (gb->cpu.af.a + ~b + 1), 4));             \
    gb->cpu.af.flag.c = !BIT(gb->cpu.af.a ^ ~b ^ (gb->cpu.af.a + ~b + 1), 8)

#define INC_R(r)                                            \
    toggle_znh(gb, r + 1, 0, BIT((r + 1) ^ r ^ 1, 4));      \
    r += 1

#define INC_INDIRECT_HL()                                               \
    operand = bus_read(gb, gb->cpu.hl.val);                             \
    bus_write(gb, gb->cpu.hl.val, operand + 1);                         \
    toggle_znh(gb, operand + 1, 0, BIT((operand + 1) ^ operand ^ 1U, 4))

#define DEC_R(r)                                               \
    toggle_znh(gb, r - 1, 1, !BIT((r - 1) ^ r ^ 0xff, 4));     \
    r--;

#define DEC_INDIRECT_HL()                                               \
    operand = bus_read(gb, gb->cpu.hl.val);                             \
    bus_write(gb, gb->cpu.hl.val, operand - 1);                         \
    toggle_znh(gb, operand - 1, 1, !BIT((operand - 1) ^ operand ^ 0xff, 4))

#define AND(b)                          \
    gb->cpu.af.a &= b;                  \
    toggle_znh(gb, gb->cpu.af.a, 0, 1); \
    gb->cpu.af.flag.c = 0

#define OR(b)                              \
    gb->cpu.af.a |= b;                     \
    toggle_znh(gb, gb->cpu.af.a, 0, 0);    \
    gb->cpu.af.flag.c = 0 

#define XOR(b)                             \
    gb->cpu.af.a ^= b;                     \
    toggle_znh(gb, gb->cpu.af.a, 0, 0);    \
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
    toggle_znh(gb, r, 0, 0);                    \
    gb->cpu.af.flag.c = 0

#define SWAP_INDIRECT_HL()                      \
    val = bus_read(gb, gb->cpu.hl.val);         \
    val = (val >> 4) | (val << 4);              \
    bus_write(gb, gb->cpu.hl.val, val);         \
    toggle_znh(gb, val, 0, 0);                  \
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
    toggle_znh(gb, BIT(r, n), 0, 1)

#define BIT_N_INDIRECT_HL(n)                    \
    val = bus_read(gb, gb->cpu.hl.val);         \
    toggle_znh(gb, BIT(val, n), 0, 1)

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
    if (is_interrupt_pending(gb)) { \
        gb->mode = (!gb->cpu.ime) ? HALT_BUG : NORMAL;  \
    }

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
    case 0x06: RLC_INDIRECT_HL();                                     break;
    case 0x07: RLC_R(gb->cpu.af.a);                              break;
    case 0x08: RRC_R(gb->cpu.bc.b);                              break;
    case 0x09: RRC_R(gb->cpu.bc.c);                              break;
    case 0x0a: RRC_R(gb->cpu.de.d);                              break;
    case 0x0b: RRC_R(gb->cpu.de.e);                              break;
    case 0x0c: RRC_R(gb->cpu.hl.h);                              break;
    case 0x0d: RRC_R(gb->cpu.hl.l);                              break;
    case 0x0e: RRC_INDIRECT_HL();                                     break;
    case 0x0f: RRC_R(gb->cpu.af.a);                              break;
    case 0x10: RL_R(gb->cpu.bc.b);                               break;
    case 0x11: RL_R(gb->cpu.bc.c);                               break;
    case 0x12: RL_R(gb->cpu.de.d);                               break;
    case 0x13: RL_R(gb->cpu.de.e);                               break;
    case 0x14: RL_R(gb->cpu.hl.h);                               break;
    case 0x15: RL_R(gb->cpu.hl.l);                               break;
    case 0x16: RL_INDIRECT_HL();                                      break;
    case 0x17: RL_R(gb->cpu.af.a);                               break;
    case 0x18: RR_R(gb->cpu.bc.b);                               break;
    case 0x19: RR_R(gb->cpu.bc.c);                               break;
    case 0x1a: RR_R(gb->cpu.de.d);                               break;
    case 0x1b: RR_R(gb->cpu.de.e);                               break;
    case 0x1c: RR_R(gb->cpu.hl.h);                               break;
    case 0x1d: RR_R(gb->cpu.hl.l);                               break;
    case 0x1e: RR_INDIRECT_HL();                                      break;
    case 0x1f: RR_R(gb->cpu.af.a);                               break;
    case 0x20: SLA_R(gb->cpu.bc.b);                              break;
    case 0x21: SLA_R(gb->cpu.bc.c);                              break;
    case 0x22: SLA_R(gb->cpu.de.d);                              break;
    case 0x23: SLA_R(gb->cpu.de.e);                              break;
    case 0x24: SLA_R(gb->cpu.hl.h);                              break;
    case 0x25: SLA_R(gb->cpu.hl.l);                              break;
    case 0x26: SLA_INDIRECT_HL();                                     break;
    case 0x27: SLA_R(gb->cpu.af.a);                              break;
    case 0x28: SRA_R(gb->cpu.bc.b);                              break;
    case 0x29: SRA_R(gb->cpu.bc.c);                              break;
    case 0x2a: SRA_R(gb->cpu.de.d);                              break;
    case 0x2b: SRA_R(gb->cpu.de.e);                              break;
    case 0x2c: SRA_R(gb->cpu.hl.h);                              break;
    case 0x2d: SRA_R(gb->cpu.hl.l);                              break;
    case 0x2e: SRA_INDIRECT_HL();                                     break;
    case 0x2f: SRA_R(gb->cpu.af.a);                              break;
    case 0x30: SWAP_R(gb->cpu.bc.b);                             break;
    case 0x31: SWAP_R(gb->cpu.bc.c);                             break;
    case 0x32: SWAP_R(gb->cpu.de.d);                             break;
    case 0x33: SWAP_R(gb->cpu.de.e);                             break;
    case 0x34: SWAP_R(gb->cpu.hl.h);                             break;
    case 0x35: SWAP_R(gb->cpu.hl.l);                             break;
    case 0x36: SWAP_INDIRECT_HL();                                    break;
    case 0x37: SWAP_R(gb->cpu.af.a);                             break;
    case 0x38: SRL_R(gb->cpu.bc.b);                              break;
    case 0x39: SRL_R(gb->cpu.bc.c);                              break;
    case 0x3a: SRL_R(gb->cpu.de.d);                              break;
    case 0x3b: SRL_R(gb->cpu.de.e);                              break;
    case 0x3c: SRL_R(gb->cpu.hl.h);                              break;
    case 0x3d: SRL_R(gb->cpu.hl.l);                              break;
    case 0x3e: SRL_INDIRECT_HL();                                     break;
    case 0x3f: SRL_R(gb->cpu.af.a);                              break;
    case 0x40: BIT_N_R(0, gb->cpu.bc.b);                         break;
    case 0x41: BIT_N_R(0, gb->cpu.bc.c);                         break;
    case 0x42: BIT_N_R(0, gb->cpu.de.d);                         break;
    case 0x43: BIT_N_R(0, gb->cpu.de.e);                         break;
    case 0x44: BIT_N_R(0, gb->cpu.hl.h);                         break;
    case 0x45: BIT_N_R(0, gb->cpu.hl.l);                         break;
    case 0x46: BIT_N_INDIRECT_HL(0);                                break;
    case 0x47: BIT_N_R(0, gb->cpu.af.a);                         break;
    case 0x48: BIT_N_R(1, gb->cpu.bc.b);                         break;
    case 0x49: BIT_N_R(1, gb->cpu.bc.c);                         break;
    case 0x4a: BIT_N_R(1, gb->cpu.de.d);                         break;
    case 0x4b: BIT_N_R(1, gb->cpu.de.e);                         break;
    case 0x4c: BIT_N_R(1, gb->cpu.hl.h);                         break;
    case 0x4d: BIT_N_R(1, gb->cpu.hl.l);                         break;
    case 0x4e: BIT_N_INDIRECT_HL(1);                                break;
    case 0x4f: BIT_N_R(1, gb->cpu.af.a);                         break;
    case 0x50: BIT_N_R(2, gb->cpu.bc.b);                         break;
    case 0x51: BIT_N_R(2, gb->cpu.bc.c);                         break;
    case 0x52: BIT_N_R(2, gb->cpu.de.d);                         break;
    case 0x53: BIT_N_R(2, gb->cpu.de.e);                         break;
    case 0x54: BIT_N_R(2, gb->cpu.hl.h);                         break;
    case 0x55: BIT_N_R(2, gb->cpu.hl.l);                         break;
    case 0x56: BIT_N_INDIRECT_HL(2);                                break;
    case 0x57: BIT_N_R(2, gb->cpu.af.a);                         break;
    case 0x58: BIT_N_R(3, gb->cpu.bc.b);                         break;
    case 0x59: BIT_N_R(3, gb->cpu.bc.c);                         break;
    case 0x5a: BIT_N_R(3, gb->cpu.de.d);                         break;
    case 0x5b: BIT_N_R(3, gb->cpu.de.e);                         break;
    case 0x5c: BIT_N_R(3, gb->cpu.hl.h);                         break;
    case 0x5d: BIT_N_R(3, gb->cpu.hl.l);                         break;
    case 0x5e: BIT_N_INDIRECT_HL(3);                                break;
    case 0x5f: BIT_N_R(3, gb->cpu.af.a);                         break;
    case 0x60: BIT_N_R(4, gb->cpu.bc.b);                         break;
    case 0x61: BIT_N_R(4, gb->cpu.bc.c);                         break;
    case 0x62: BIT_N_R(4, gb->cpu.de.d);                         break;
    case 0x63: BIT_N_R(4, gb->cpu.de.e);                         break;
    case 0x64: BIT_N_R(4, gb->cpu.hl.h);                         break;
    case 0x65: BIT_N_R(4, gb->cpu.hl.l);                         break;
    case 0x66: BIT_N_INDIRECT_HL(4);                                break;
    case 0x67: BIT_N_R(4, gb->cpu.af.a);                         break;
    case 0x68: BIT_N_R(5, gb->cpu.bc.b);                         break;
    case 0x69: BIT_N_R(5, gb->cpu.bc.c);                         break;
    case 0x6a: BIT_N_R(5, gb->cpu.de.d);                         break;
    case 0x6b: BIT_N_R(5, gb->cpu.de.e);                         break;
    case 0x6c: BIT_N_R(5, gb->cpu.hl.h);                         break;
    case 0x6d: BIT_N_R(5, gb->cpu.hl.l);                         break;
    case 0x6e: BIT_N_INDIRECT_HL(5);                                break;
    case 0x6f: BIT_N_R(5, gb->cpu.af.a);                         break;
    case 0x70: BIT_N_R(6, gb->cpu.bc.b);                         break;
    case 0x71: BIT_N_R(6, gb->cpu.bc.c);                         break;
    case 0x72: BIT_N_R(6, gb->cpu.de.d);                         break;
    case 0x73: BIT_N_R(6, gb->cpu.de.e);                         break;
    case 0x74: BIT_N_R(6, gb->cpu.hl.h);                         break;
    case 0x75: BIT_N_R(6, gb->cpu.hl.l);                         break;
    case 0x76: BIT_N_INDIRECT_HL(6);                                break;
    case 0x77: BIT_N_R(6, gb->cpu.af.a);                         break;
    case 0x78: BIT_N_R(7, gb->cpu.bc.b);                         break;
    case 0x79: BIT_N_R(7, gb->cpu.bc.c);                         break;
    case 0x7a: BIT_N_R(7, gb->cpu.de.d);                         break;
    case 0x7b: BIT_N_R(7, gb->cpu.de.e);                         break;
    case 0x7c: BIT_N_R(7, gb->cpu.hl.h);                         break;
    case 0x7d: BIT_N_R(7, gb->cpu.hl.l);                         break;
    case 0x7e: BIT_N_INDIRECT_HL(7);                                break;
    case 0x7f: BIT_N_R(7, gb->cpu.af.a);                         break;
    case 0x80: RES_N_R(0, gb->cpu.bc.b);                         break;
    case 0x81: RES_N_R(0, gb->cpu.bc.c);                         break;
    case 0x82: RES_N_R(0, gb->cpu.de.d);                         break;
    case 0x83: RES_N_R(0, gb->cpu.de.e);                         break;
    case 0x84: RES_N_R(0, gb->cpu.hl.h);                         break;
    case 0x85: RES_N_R(0, gb->cpu.hl.l);                         break;
    case 0x86: RES_N_INDIRECT_HL(0);                                break;
    case 0x87: RES_N_R(0, gb->cpu.af.a);                         break;
    case 0x88: RES_N_R(1, gb->cpu.bc.b);                         break;
    case 0x89: RES_N_R(1, gb->cpu.bc.c);                         break;
    case 0x8a: RES_N_R(1, gb->cpu.de.d);                         break;
    case 0x8b: RES_N_R(1, gb->cpu.de.e);                         break;
    case 0x8c: RES_N_R(1, gb->cpu.hl.h);                         break;
    case 0x8d: RES_N_R(1, gb->cpu.hl.l);                         break;
    case 0x8e: RES_N_INDIRECT_HL(1);                                break;
    case 0x8f: RES_N_R(1, gb->cpu.af.a);                         break;
    case 0x90: RES_N_R(2, gb->cpu.bc.b);                         break;
    case 0x91: RES_N_R(2, gb->cpu.bc.c);                         break;
    case 0x92: RES_N_R(2, gb->cpu.de.d);                         break;
    case 0x93: RES_N_R(2, gb->cpu.de.e);                         break;
    case 0x94: RES_N_R(2, gb->cpu.hl.h);                         break;
    case 0x95: RES_N_R(2, gb->cpu.hl.l);                         break;
    case 0x96: RES_N_INDIRECT_HL(2);                                break;
    case 0x97: RES_N_R(2, gb->cpu.af.a);                         break;
    case 0x98: RES_N_R(3, gb->cpu.bc.b);                         break;
    case 0x99: RES_N_R(3, gb->cpu.bc.c);                         break;
    case 0x9a: RES_N_R(3, gb->cpu.de.d);                         break;
    case 0x9b: RES_N_R(3, gb->cpu.de.e);                         break;
    case 0x9c: RES_N_R(3, gb->cpu.hl.h);                         break;
    case 0x9d: RES_N_R(3, gb->cpu.hl.l);                         break;
    case 0x9e: RES_N_INDIRECT_HL(3);                                break;
    case 0x9f: RES_N_R(3, gb->cpu.af.a);                         break;
    case 0xa0: RES_N_R(4, gb->cpu.bc.b);                         break;
    case 0xa1: RES_N_R(4, gb->cpu.bc.c);                         break;
    case 0xa2: RES_N_R(4, gb->cpu.de.d);                         break;
    case 0xa3: RES_N_R(4, gb->cpu.de.e);                         break;
    case 0xa4: RES_N_R(4, gb->cpu.hl.h);                         break;
    case 0xa5: RES_N_R(4, gb->cpu.hl.l);                         break;
    case 0xa6: RES_N_INDIRECT_HL(4);                                break;
    case 0xa7: RES_N_R(4, gb->cpu.af.a);                         break;
    case 0xa8: RES_N_R(5, gb->cpu.bc.b);                         break;
    case 0xa9: RES_N_R(5, gb->cpu.bc.c);                         break;
    case 0xaa: RES_N_R(5, gb->cpu.de.d);                         break;
    case 0xab: RES_N_R(5, gb->cpu.de.e);                         break;
    case 0xac: RES_N_R(5, gb->cpu.hl.h);                         break;
    case 0xad: RES_N_R(5, gb->cpu.hl.l);                         break;
    case 0xae: RES_N_INDIRECT_HL(5);                                break;
    case 0xaf: RES_N_R(5, gb->cpu.af.a);                         break;
    case 0xb0: RES_N_R(6, gb->cpu.bc.b);                         break;
    case 0xb1: RES_N_R(6, gb->cpu.bc.c);                         break;
    case 0xb2: RES_N_R(6, gb->cpu.de.d);                         break;
    case 0xb3: RES_N_R(6, gb->cpu.de.e);                         break;
    case 0xb4: RES_N_R(6, gb->cpu.hl.h);                         break;
    case 0xb5: RES_N_R(6, gb->cpu.hl.l);                         break;
    case 0xb6: RES_N_INDIRECT_HL(6);                                break;
    case 0xb7: RES_N_R(6, gb->cpu.af.a);                         break;
    case 0xb8: RES_N_R(7, gb->cpu.bc.b);                         break;
    case 0xb9: RES_N_R(7, gb->cpu.bc.c);                         break;
    case 0xba: RES_N_R(7, gb->cpu.de.d);                         break;
    case 0xbb: RES_N_R(7, gb->cpu.de.e);                         break;
    case 0xbc: RES_N_R(7, gb->cpu.hl.h);                         break;
    case 0xbd: RES_N_R(7, gb->cpu.hl.l);                         break;
    case 0xbe: RES_N_INDIRECT_HL(7);                                break;
    case 0xbf: RES_N_R(7, gb->cpu.af.a);                         break;
    case 0xc0: SET_N_R(0, gb->cpu.bc.b);                         break;
    case 0xc1: SET_N_R(0, gb->cpu.bc.c);                         break;
    case 0xc2: SET_N_R(0, gb->cpu.de.d);                         break;
    case 0xc3: SET_N_R(0, gb->cpu.de.e);                         break;
    case 0xc4: SET_N_R(0, gb->cpu.hl.h);                         break;
    case 0xc5: SET_N_R(0, gb->cpu.hl.l);                         break;
    case 0xc6: SET_N_INDIRECT_HL(0);                                break;
    case 0xc7: SET_N_R(0, gb->cpu.af.a);                         break;
    case 0xc8: SET_N_R(1, gb->cpu.bc.b);                         break;
    case 0xc9: SET_N_R(1, gb->cpu.bc.c);                         break;
    case 0xca: SET_N_R(1, gb->cpu.de.d);                         break;
    case 0xcb: SET_N_R(1, gb->cpu.de.e);                         break;
    case 0xcc: SET_N_R(1, gb->cpu.hl.h);                         break;
    case 0xcd: SET_N_R(1, gb->cpu.hl.l);                         break;
    case 0xce: SET_N_INDIRECT_HL(1);                                break;
    case 0xcf: SET_N_R(1, gb->cpu.af.a);                         break;
    case 0xd0: SET_N_R(2, gb->cpu.bc.b);                         break;
    case 0xd1: SET_N_R(2, gb->cpu.bc.c);                         break;
    case 0xd2: SET_N_R(2, gb->cpu.de.d);                         break;
    case 0xd3: SET_N_R(2, gb->cpu.de.e);                         break;
    case 0xd4: SET_N_R(2, gb->cpu.hl.h);                         break;
    case 0xd5: SET_N_R(2, gb->cpu.hl.l);                         break;
    case 0xd6: SET_N_INDIRECT_HL(2);                                break;
    case 0xd7: SET_N_R(2, gb->cpu.af.a);                         break;
    case 0xd8: SET_N_R(3, gb->cpu.bc.b);                         break;
    case 0xd9: SET_N_R(3, gb->cpu.bc.c);                         break;
    case 0xda: SET_N_R(3, gb->cpu.de.d);                         break;
    case 0xdb: SET_N_R(3, gb->cpu.de.e);                         break;
    case 0xdc: SET_N_R(3, gb->cpu.hl.h);                         break;
    case 0xdd: SET_N_R(3, gb->cpu.hl.l);                         break;
    case 0xde: SET_N_INDIRECT_HL(3);                                break;
    case 0xdf: SET_N_R(3, gb->cpu.af.a);                         break;
    case 0xe0: SET_N_R(4, gb->cpu.bc.b);                         break;
    case 0xe1: SET_N_R(4, gb->cpu.bc.c);                         break;
    case 0xe2: SET_N_R(4, gb->cpu.de.d);                         break;
    case 0xe3: SET_N_R(4, gb->cpu.de.e);                         break;
    case 0xe4: SET_N_R(4, gb->cpu.hl.h);                         break;
    case 0xe5: SET_N_R(4, gb->cpu.hl.l);                         break;
    case 0xe6: SET_N_INDIRECT_HL(4);                                break;
    case 0xe7: SET_N_R(4, gb->cpu.af.a);                         break;
    case 0xe8: SET_N_R(5, gb->cpu.bc.b);                         break;
    case 0xe9: SET_N_R(5, gb->cpu.bc.c);                         break;
    case 0xea: SET_N_R(5, gb->cpu.de.d);                         break;
    case 0xeb: SET_N_R(5, gb->cpu.de.e);                         break;
    case 0xec: SET_N_R(5, gb->cpu.hl.h);                         break;
    case 0xed: SET_N_R(5, gb->cpu.hl.l);                         break;
    case 0xee: SET_N_INDIRECT_HL(5);                                break;
    case 0xef: SET_N_R(5, gb->cpu.af.a);                         break;
    case 0xf0: SET_N_R(6, gb->cpu.bc.b);                         break;
    case 0xf1: SET_N_R(6, gb->cpu.bc.c);                         break;
    case 0xf2: SET_N_R(6, gb->cpu.de.d);                         break;
    case 0xf3: SET_N_R(6, gb->cpu.de.e);                         break;
    case 0xf4: SET_N_R(6, gb->cpu.hl.h);                         break;
    case 0xf5: SET_N_R(6, gb->cpu.hl.l);                         break;
    case 0xf6: SET_N_INDIRECT_HL(6);                                break;
    case 0xf7: SET_N_R(6, gb->cpu.af.a);                         break;
    case 0xf8: SET_N_R(7, gb->cpu.bc.b);                         break;
    case 0xf9: SET_N_R(7, gb->cpu.bc.c);                         break;
    case 0xfa: SET_N_R(7, gb->cpu.de.d);                         break;
    case 0xfb: SET_N_R(7, gb->cpu.de.e);                         break;
    case 0xfc: SET_N_R(7, gb->cpu.hl.h);                         break;
    case 0xfd: SET_N_R(7, gb->cpu.hl.l);                         break;
    case 0xfe: SET_N_INDIRECT_HL(7);                                break;
    case 0xff: SET_N_R(7, gb->cpu.af.a);                         break;
    default:
        break;
    }
    return cb_instr_cycle[opcode];
}

int sm83_step(struct gb *gb)
{
    uint8_t opcode = sm83_fetch_byte(gb), a;
    uint16_t operand, res, carry_per_bit;

    gb->executed_cycle = instr_cycle[opcode];

    // printf("opcode: 0x%02x a: 0x%02x f: 0x%02x b: 0x%02x c: 0x%02x d: 0x%02x e: 0x%02x h: 0x%02x l: 0x%02x pc: 0x%04x\n", 
    //     opcode, gb->cpu.af.a, gb->cpu.af.f, gb->cpu.bc.b, gb->cpu.bc.c, gb->cpu.de.d, gb->cpu.de.e, gb->cpu.hl.h, gb->cpu.hl.l, gb->cpu.pc);
    switch (opcode) {
    case 0x00:                                                          break;
    case 0x01: gb->cpu.bc.val = sm83_fetch_word(gb);                   break;
    case 0x02: bus_write(gb, gb->cpu.bc.val, gb->cpu.af.a);          break;
    case 0x03: INC_RR(gb->cpu.bc.val);                            break;
    case 0x04: INC_R(gb->cpu.bc.b);                              break;
    case 0x05: DEC_R(gb->cpu.bc.b);                              break;
    case 0x06: gb->cpu.bc.b = sm83_fetch_byte(gb);                    break;
    case 0x07: RLCA();                                                break;
    case 0x08: 
        operand = sm83_fetch_word(gb);
        LD_INDIRECT_NN_SP(operand);
        break;
    case 0x09: ADD_HL_RR(gb->cpu.bc.val);                          break;
    case 0x0a: gb->cpu.af.a = bus_read(gb, gb->cpu.bc.val);          break;
    case 0x0b: DEC_RR(gb->cpu.bc.val);                            break;
    case 0x0c: INC_R(gb->cpu.bc.c);                              break;
    case 0x0d: DEC_R(gb->cpu.bc.c);                              break;
    case 0x0e: gb->cpu.bc.c = sm83_fetch_byte(gb);                    break;
    case 0x0f: RRCA();                                                break;
    case 0x10:                                                 break;
    case 0x11: gb->cpu.de.val = sm83_fetch_word(gb);                   break;
    case 0x12: bus_write(gb, gb->cpu.de.val, gb->cpu.af.a);          break;
    case 0x13: INC_RR(gb->cpu.de.val);                            break;
    case 0x14: INC_R(gb->cpu.de.d);                              break;
    case 0x15: DEC_R(gb->cpu.de.d);                              break;
    case 0x16: gb->cpu.de.d = sm83_fetch_byte(gb);                    break;
    case 0x17: RLA();                                                 break;
    case 0x18: 
        operand = sm83_fetch_byte(gb);
        JP(gb->cpu.pc, operand, 1);
        break;
    case 0x19: ADD_HL_RR(gb->cpu.de.val);                          break;
    case 0x1a: gb->cpu.af.a = bus_read(gb, gb->cpu.de.val);          break;
    case 0x1b: DEC_RR(gb->cpu.de.val);                            break;
    case 0x1c: INC_R(gb->cpu.de.e);                              break;
    case 0x1d: DEC_R(gb->cpu.de.e);                              break;
    case 0x1e: gb->cpu.de.e = sm83_fetch_byte(gb);                    break;
    case 0x1f: RRA();                                                 break;
    case 0x20: 
        operand = sm83_fetch_byte(gb);
        JP(gb->cpu.pc, operand, !gb->cpu.af.flag.z);
        break;
    case 0x21: gb->cpu.hl.val = sm83_fetch_word(gb);                   break;
    case 0x22: bus_write(gb, gb->cpu.hl.val++, gb->cpu.af.a);        break;
    case 0x23: INC_RR(gb->cpu.hl.val);                            break;
    case 0x24: INC_R(gb->cpu.hl.h);                              break;
    case 0x25: DEC_R(gb->cpu.hl.h);                              break;
    case 0x26: gb->cpu.hl.h = sm83_fetch_byte(gb);                    break;
    case 0x27: DAA();                                                 break;
    case 0x28: 
        operand = sm83_fetch_byte(gb);
        JP(gb->cpu.pc, operand, gb->cpu.af.flag.z);
        break;
    case 0x29: ADD_HL_RR(gb->cpu.hl.val);                          break;
    case 0x2a: gb->cpu.af.a = bus_read(gb, gb->cpu.hl.val++);        break;
    case 0x2b: DEC_RR(gb->cpu.hl.val);                            break;
    case 0x2c: INC_R(gb->cpu.hl.l);                              break;
    case 0x2d: DEC_R(gb->cpu.hl.l);                              break;
    case 0x2e: gb->cpu.hl.l = sm83_fetch_byte(gb);                    break;
    case 0x2f: CPL();                                                 break;
    case 0x30:
        operand = sm83_fetch_byte(gb);
        JP(gb->cpu.pc, operand, !gb->cpu.af.flag.c);
        break;
    case 0x31: gb->cpu.sp = sm83_fetch_word(gb);                   break;
    case 0x32: bus_write(gb, gb->cpu.hl.val--, gb->cpu.af.a);        break;
    case 0x33: INC_RR(gb->cpu.sp);                            break;
    case 0x34: INC_INDIRECT_HL();                                     break;
    case 0x35: DEC_INDIRECT_HL();                                     break;
    case 0x36: 
        operand = sm83_fetch_byte(gb);
        LD_INDIRECT_HL_N(operand);
        break;
    case 0x37: SCF();                                                 break;
    case 0x38:
        operand = sm83_fetch_byte(gb);
        JP(gb->cpu.pc, operand, gb->cpu.af.flag.c);
        break;
    case 0x39: ADD_HL_RR(gb->cpu.sp);                          break;
    case 0x3a: gb->cpu.af.a = bus_read(gb, gb->cpu.hl.val--);        break;
    case 0x3b: DEC_RR(gb->cpu.sp);                            break;
    case 0x3c: INC_R(gb->cpu.af.a);                              break;
    case 0x3d: DEC_R(gb->cpu.af.a);                              break;
    case 0x3e: gb->cpu.af.a = sm83_fetch_byte(gb);                    break;
    case 0x3f: CCF();                                                 break;
    case 0x40: gb->cpu.bc.b = gb->cpu.bc.b;                         break;
    case 0x41: gb->cpu.bc.b = gb->cpu.bc.c;                         break;
    case 0x42: gb->cpu.bc.b = gb->cpu.de.d;                         break;
    case 0x43: gb->cpu.bc.b = gb->cpu.de.e;                         break;
    case 0x44: gb->cpu.bc.b = gb->cpu.hl.h;                         break;
    case 0x45: gb->cpu.bc.b = gb->cpu.hl.l;                         break;
    case 0x46: gb->cpu.bc.b = bus_read(gb, gb->cpu.hl.val);          break;
    case 0x47: gb->cpu.bc.b = gb->cpu.af.a;                         break;
    case 0x48: gb->cpu.bc.c = gb->cpu.bc.b;                         break;
    case 0x49: gb->cpu.bc.c = gb->cpu.bc.c;                         break;
    case 0x4a: gb->cpu.bc.c = gb->cpu.de.d;                         break;
    case 0x4b: gb->cpu.bc.c = gb->cpu.de.e;                         break;
    case 0x4c: gb->cpu.bc.c = gb->cpu.hl.h;                         break;
    case 0x4d: gb->cpu.bc.c = gb->cpu.hl.l;                         break;
    case 0x4e: gb->cpu.bc.c = bus_read(gb, gb->cpu.hl.val);          break;
    case 0x4f: gb->cpu.bc.c = gb->cpu.af.a;                         break;
    case 0x50: gb->cpu.de.d = gb->cpu.bc.b;                         break;
    case 0x51: gb->cpu.de.d = gb->cpu.bc.c;                         break;
    case 0x52: gb->cpu.de.d = gb->cpu.de.d;                         break;
    case 0x53: gb->cpu.de.d = gb->cpu.de.e;                         break;
    case 0x54: gb->cpu.de.d = gb->cpu.hl.h;                         break;
    case 0x55: gb->cpu.de.d = gb->cpu.hl.l;                         break;
    case 0x56: gb->cpu.de.d = bus_read(gb, gb->cpu.hl.val);          break;
    case 0x57: gb->cpu.de.d = gb->cpu.af.a;                         break;
    case 0x58: gb->cpu.de.e = gb->cpu.bc.b;                         break;
    case 0x59: gb->cpu.de.e = gb->cpu.bc.c;                         break;
    case 0x5a: gb->cpu.de.e = gb->cpu.de.d;                         break;
    case 0x5b: gb->cpu.de.e = gb->cpu.de.e;                         break;
    case 0x5c: gb->cpu.de.e = gb->cpu.hl.h;                         break;
    case 0x5d: gb->cpu.de.e = gb->cpu.hl.l;                         break;
    case 0x5e: gb->cpu.de.e = bus_read(gb, gb->cpu.hl.val);          break;
    case 0x5f: gb->cpu.de.e = gb->cpu.af.a;                         break;
    case 0x60: gb->cpu.hl.h = gb->cpu.bc.b;                         break;
    case 0x61: gb->cpu.hl.h = gb->cpu.bc.c;                         break;
    case 0x62: gb->cpu.hl.h = gb->cpu.de.d;                         break;
    case 0x63: gb->cpu.hl.h = gb->cpu.de.e;                         break;
    case 0x64: gb->cpu.hl.h = gb->cpu.hl.h;                         break;
    case 0x65: gb->cpu.hl.h = gb->cpu.hl.l;                         break;
    case 0x66: gb->cpu.hl.h = bus_read(gb, gb->cpu.hl.val);          break;
    case 0x67: gb->cpu.hl.h = gb->cpu.af.a;                         break;
    case 0x68: gb->cpu.hl.l = gb->cpu.bc.b;                         break;
    case 0x69: gb->cpu.hl.l = gb->cpu.bc.c;                         break;
    case 0x6a: gb->cpu.hl.l = gb->cpu.de.d;                         break;
    case 0x6b: gb->cpu.hl.l = gb->cpu.de.e;                         break;
    case 0x6c: gb->cpu.hl.l = gb->cpu.hl.h;                         break;
    case 0x6d: gb->cpu.hl.l = gb->cpu.hl.l;                         break;
    case 0x6e: gb->cpu.hl.l = bus_read(gb, gb->cpu.hl.val);          break;
    case 0x6f: gb->cpu.hl.l = gb->cpu.af.a;                         break;
    case 0x70: bus_write(gb, gb->cpu.hl.val, gb->cpu.bc.b);          break;
    case 0x71: bus_write(gb, gb->cpu.hl.val, gb->cpu.bc.c);          break;
    case 0x72: bus_write(gb, gb->cpu.hl.val, gb->cpu.de.d);          break;
    case 0x73: bus_write(gb, gb->cpu.hl.val, gb->cpu.de.e);          break;
    case 0x74: bus_write(gb, gb->cpu.hl.val, gb->cpu.hl.h);          break;
    case 0x75: bus_write(gb, gb->cpu.hl.val, gb->cpu.hl.l);          break;
    case 0x76: HALT();                                                break;
    case 0x77: bus_write(gb, gb->cpu.hl.val, gb->cpu.af.a);          break;
    case 0x78: gb->cpu.af.a = gb->cpu.bc.b;                         break;
    case 0x79: gb->cpu.af.a = gb->cpu.bc.c;                         break;
    case 0x7a: gb->cpu.af.a = gb->cpu.de.d;                         break;
    case 0x7b: gb->cpu.af.a = gb->cpu.de.e;                         break;
    case 0x7c: gb->cpu.af.a = gb->cpu.hl.h;                         break;
    case 0x7d: gb->cpu.af.a = gb->cpu.hl.l;                         break;
    case 0x7e: gb->cpu.af.a = bus_read(gb, gb->cpu.hl.val);          break;
    case 0x7f: gb->cpu.af.a = gb->cpu.af.a;                         break;
    case 0x80: ADD(gb->cpu.bc.b, 0);                              break;
    case 0x81: ADD(gb->cpu.bc.c, 0);                              break;
    case 0x82: ADD(gb->cpu.de.d, 0);                              break;
    case 0x83: ADD(gb->cpu.de.e, 0);                              break;
    case 0x84: ADD(gb->cpu.hl.h, 0);                              break;
    case 0x85: ADD(gb->cpu.hl.l, 0);                              break;
    case 0x86: 
        operand = bus_read(gb, gb->cpu.hl.val);
        ADD(operand, 0);
        break;
    case 0x87: ADD(gb->cpu.af.a, 0);                              break;
    case 0x88: ADD(gb->cpu.bc.b, gb->cpu.af.flag.c);            break;
    case 0x89: ADD(gb->cpu.bc.c, gb->cpu.af.flag.c);            break;
    case 0x8a: ADD(gb->cpu.de.d, gb->cpu.af.flag.c);            break;
    case 0x8b: ADD(gb->cpu.de.e, gb->cpu.af.flag.c);            break;
    case 0x8c: ADD(gb->cpu.hl.h, gb->cpu.af.flag.c);            break;
    case 0x8d: ADD(gb->cpu.hl.l, gb->cpu.af.flag.c);            break;
    case 0x8e: 
        operand = bus_read(gb, gb->cpu.hl.val);
        ADD(operand, gb->cpu.af.flag.c);
        break;
    case 0x8f: ADD(gb->cpu.af.a, gb->cpu.af.flag.c);            break;
    case 0x90: SUB(gb->cpu.bc.b, 0);                              break;
    case 0x91: SUB(gb->cpu.bc.c, 0);                              break;
    case 0x92: SUB(gb->cpu.de.d, 0);                              break;
    case 0x93: SUB(gb->cpu.de.e, 0);                              break;
    case 0x94: SUB(gb->cpu.hl.h, 0);                              break;
    case 0x95: SUB(gb->cpu.hl.l, 0);                              break;
    case 0x96:
        operand = bus_read(gb, gb->cpu.hl.val);
        SUB(operand, 0);      
        break;
    case 0x97: SUB(gb->cpu.af.a, 0);                              break;
    case 0x98: SUB(gb->cpu.bc.b, gb->cpu.af.flag.c);            break;
    case 0x99: SUB(gb->cpu.bc.c, gb->cpu.af.flag.c);            break;
    case 0x9a: SUB(gb->cpu.de.d, gb->cpu.af.flag.c);            break;
    case 0x9b: SUB(gb->cpu.de.e, gb->cpu.af.flag.c);            break;
    case 0x9c: SUB(gb->cpu.hl.h, gb->cpu.af.flag.c);            break;
    case 0x9d: SUB(gb->cpu.hl.l, gb->cpu.af.flag.c);            break;
    case 0x9e:
        operand = bus_read(gb, gb->cpu.hl.val);
        SUB(operand, gb->cpu.af.flag.c);
        break;
    case 0x9f: SUB(gb->cpu.af.a, gb->cpu.af.flag.c);            break;
    case 0xa0: AND(gb->cpu.bc.b);                                 break;
    case 0xa1: AND(gb->cpu.bc.c);                                 break;
    case 0xa2: AND(gb->cpu.de.d);                                 break;
    case 0xa3: AND(gb->cpu.de.e);                                 break;
    case 0xa4: AND(gb->cpu.hl.h);                                 break;
    case 0xa5: AND(gb->cpu.hl.l);                                 break;
    case 0xa6: 
        operand = bus_read(gb, gb->cpu.hl.val);
        AND(operand);
        break;
    case 0xa7: AND(gb->cpu.af.a);                                 break;
    case 0xa8: XOR(gb->cpu.bc.b);                                 break;
    case 0xa9: XOR(gb->cpu.bc.c);                                 break;
    case 0xaa: XOR(gb->cpu.de.d);                                 break;
    case 0xab: XOR(gb->cpu.de.e);                                 break;
    case 0xac: XOR(gb->cpu.hl.h);                                 break;
    case 0xad: XOR(gb->cpu.hl.l);                                 break;
    case 0xae: 
        operand = bus_read(gb, gb->cpu.hl.val);
        XOR(operand);
        break;
    case 0xaf: XOR(gb->cpu.af.a);                                 break;
    case 0xb0: OR(gb->cpu.bc.b);                                  break;
    case 0xb1: OR(gb->cpu.bc.c);                                  break;
    case 0xb2: OR(gb->cpu.de.d);                                  break;
    case 0xb3: OR(gb->cpu.de.e);                                  break;
    case 0xb4: OR(gb->cpu.hl.h);                                  break;
    case 0xb5: OR(gb->cpu.hl.l);                                  break;
    case 0xb6: 
        operand = bus_read(gb, gb->cpu.hl.val);
        OR(operand);
        break;
    case 0xb7: OR(gb->cpu.af.a);                                  break;
    case 0xb8: CP(gb->cpu.bc.b);                                  break;
    case 0xb9: CP(gb->cpu.bc.c);                                  break;
    case 0xba: CP(gb->cpu.de.d);                                  break;
    case 0xbb: CP(gb->cpu.de.e);                                  break;
    case 0xbc: CP(gb->cpu.hl.h);                                  break;
    case 0xbd: CP(gb->cpu.hl.l);                                  break;
    case 0xbe:
        operand = bus_read(gb, gb->cpu.hl.val);
        CP(operand);
        break;
    case 0xbf: CP(gb->cpu.af.a);                                  break;
    case 0xc0: RET(opcode, !gb->cpu.af.flag.z);                   break;
    case 0xc1: gb->cpu.bc.val = sm83_pop_word(gb);                     break;
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
    case 0xc5: PUSH_RR(gb->cpu.bc.val);                            break;
    case 0xc6: 
        operand = sm83_fetch_byte(gb);
        ADD(operand, 0);
        break;
    case 0xc7: RST_N(0x00);                                         break;
    case 0xc8: RET(opcode, gb->cpu.af.flag.z);                    break;
    case 0xc9: RET(opcode, 1);                                      break;
    case 0xca: 
        operand = sm83_fetch_word(gb);
        JP(operand, 0, gb->cpu.af.flag.z);
        break;
    case 0xcb:
        operand = sm83_fetch_byte(gb);
        gb->executed_cycle = execute_cb_instructions(gb, operand);
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
        operand = sm83_fetch_byte(gb);
        ADD(operand, gb->cpu.af.flag.c);
        break;
    case 0xcf: RST_N(0x08);                                         break;
    case 0xd0: RET(opcode, !gb->cpu.af.flag.c);                   break;
    case 0xd1: gb->cpu.de.val = sm83_pop_word(gb);                     break;
    case 0xd2: 
        operand = sm83_fetch_word(gb);
        JP(operand, 0, !gb->cpu.af.flag.c);
        break;
    case 0xd4: 
        operand = sm83_fetch_word(gb);
        CALL(operand, !gb->cpu.af.flag.c);
        break;
    case 0xd5: PUSH_RR(gb->cpu.de.val);                            break;
    case 0xd6:
        operand = sm83_fetch_byte(gb);
        SUB(operand, 0);
        break;
    case 0xd7: RST_N(0x10);                                         break;
    case 0xd8: RET(opcode, gb->cpu.af.flag.c);                    break;
    case 0xd9: RETI();                                                break;
    case 0xda:
        operand = sm83_fetch_word(gb);
        JP(operand, 0, gb->cpu.af.flag.c);
        break;
    case 0xdc: 
        operand = sm83_fetch_word(gb);
        CALL(operand, gb->cpu.af.flag.c);
        break;
    case 0xde:
        operand = sm83_fetch_byte(gb);
        SUB(operand, gb->cpu.af.flag.c);
        break;
    case 0xdf: RST_N(0x18);                                         break;
    case 0xe0: 
        operand = sm83_fetch_byte(gb);
        LDH_INDIRECT_N_A(operand);
        break;
    case 0xe1: gb->cpu.hl.val = sm83_pop_word(gb);                     break;
    case 0xe2: LDH_INDIRECT_C_A();                                    break;
    case 0xe5: PUSH_RR(gb->cpu.hl.val);                            break;
    case 0xe6: 
        operand = sm83_fetch_byte(gb);
        AND(operand);
        break;
    case 0xe7: RST_N(0x20);                                         break;
    case 0xe8: 
        operand = sm83_fetch_byte(gb);
        ADD_SP_I8(operand);
        break;
    case 0xe9: gb->cpu.pc = gb->cpu.hl.val;                       break;
    case 0xea:
        operand = sm83_fetch_word(gb);
        bus_write(gb, operand, gb->cpu.af.a);
        break;
    case 0xee: 
        operand = sm83_fetch_byte(gb);
        XOR(operand);
        break;
    case 0xef: RST_N(0x28);                                         break;
    case 0xf0: 
        operand = sm83_fetch_byte(gb);
        LDH_A_INDIRECT_N(operand);
        break;
    case 0xf1: 
        operand = sm83_pop_word(gb);
        gb->cpu.af.val = (operand & 0xfff0) & ~0x000f;
        break;
    case 0xf2: LDH_A_INDIRECT_C();                                    break;
    case 0xf3: DI();                                                  break;
    case 0xf5: PUSH_RR(gb->cpu.af.val);                            break;
    case 0xf6:
        operand = sm83_fetch_byte(gb);
        OR(operand);
        break;
    case 0xf7: RST_N(0x30);                                         break;
    case 0xf8: 
        operand = sm83_fetch_byte(gb);
        LD_HL_SP_PLUS_I8(operand);
        break;
    case 0xf9: gb->cpu.sp = gb->cpu.hl.val;         break;
    case 0xfa: 
        operand = sm83_fetch_word(gb);
        gb->cpu.af.a = bus_read(gb, operand);
        break;
    case 0xfb: EI();                                                  break;
    case 0xfe: 
        operand = sm83_fetch_byte(gb);
        CP(operand);
        break;
    case 0xff: RST_N(0x38);                                         break;
    default:
        printf("Unknown opcode 0x%02x\n", opcode);
        break;
    }
    return gb->executed_cycle;
}