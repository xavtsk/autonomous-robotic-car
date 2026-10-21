/*
 * File: wheel_encoder.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Wheel encoder acquisition (quadrature, decoded in PIO0).
 *
 * Each encoder is decoded by its own PIO0 state machine (SM0 left, SM1
 * right) running the quadrature program from pico-examples: a jump table on
 * (last A/B state, new A/B state) adds +1 / -1 to the Y register and the
 * count is pushed to the RX FIFO on every loop. The CPU takes no interrupt
 * per edge; it drains the FIFO for the latest count when it needs one. This
 * leaves the shared IO_BANK0 GPIO interrupt free for other subsystems.
 *
 * Edge rate (counts = quadrature edges, 4 per encoder line):
 *   edges/s per wheel = wheel speed / (pi x WHEEL_DIAMETER_M)
 *                       x ENCODER_COUNTS_PER_REV
 *   Worst case is the motor at no load (calibration, wheels lifted):
 *   117 RPM + 10 % = 129 RPM = 2.15 turns/s -> 2.15 x 2000 = 4.3 k edges/s
 *   per wheel, 8.6 k edges/s for both (one edge every ~116 us on average).
 *   The PIO loop takes at most 10 cycles at 125 MHz: 12.5 M edges/s per
 *   wheel, about 2900 times the worst case, and it cannot miss an edge to
 *   interrupt latency. It still counts if ENCODER_COUNTS_PER_REV turns out
 *   to be measured at the motor shaft (x 90 gearbox).
 *
 * PIO1 belongs to the kernel's CYW43 WiFi driver (the Pico SDK claims PIO1
 * first). This module owns PIO0: instruction memory 0..23, SM0 and SM1.
 */

#include "wheel_encoder.h"
#include "../app.h"

/* Pins (Week 6 Design Review, section 2). B must be A + 1: the PIO reads
 * both lines with one IN instruction from IN_BASE = A. */
#define ENC_L_A                 (16U)   /* left wheel encoder, Grove port 4  */
#define ENC_R_A                 (4U)    /* right wheel encoder, Grove port 3 */

/* PIO0 registers (RP2040 datasheet, section 3.7) */
#define PIO0_BASE               (0x50200000U)
#define PIO0_CTRL               (PIO0_BASE + 0x000U)
#define PIO0_FSTAT              (PIO0_BASE + 0x004U)
#define PIO0_FLEVEL             (PIO0_BASE + 0x00CU)
#define PIO0_RXF0               (PIO0_BASE + 0x020U)
#define PIO0_INSTR_MEM0         (PIO0_BASE + 0x048U)
#define PIO0_SM0_CLKDIV         (PIO0_BASE + 0x0C8U)
#define PIO0_SM0_EXECCTRL       (PIO0_BASE + 0x0CCU)
#define PIO0_SM0_SHIFTCTRL      (PIO0_BASE + 0x0D0U)
#define PIO0_SM0_INSTR          (PIO0_BASE + 0x0D8U)
#define PIO0_SM0_PINCTRL        (PIO0_BASE + 0x0DCU)
#define PIO_SM_STRIDE           (0x18U)     /* SM1 registers follow SM0 */

#define PIO_CTRL_SM_ENABLE_LSB      (0U)
#define PIO_CTRL_SM_RESTART_LSB     (4U)
#define PIO_CTRL_CLKDIV_RESTART_LSB (8U)
#define PIO_FSTAT_RXEMPTY_LSB       (8U)
#define PIO_CLKDIV_ONE              (1U << 16)  /* full speed, 125 MHz    */
#define PIO_EXECCTRL_JMP_PIN_LSB    (24U)
#define PIO_EXECCTRL_WRAP_TOP_LSB   (12U)
#define PIO_EXECCTRL_WRAP_BOT_LSB   (7U)
#define PIO_SHIFTCTRL_OUT_RIGHT     (1U << 19)  /* IN shifts left         */
#define PIO_SHIFTCTRL_FJOIN_RX      (1UL << 31)
#define PIO_PINCTRL_IN_BASE_LSB     (15U)

/* Quadrature program, assembled by pioasm from pico-examples
 * pio/quadrature_encoder/quadrature_encoder.pio (.origin 0). Y holds the
 * count; invalid transitions (both lines changed) leave it unchanged. */
#define QE_WRAP_BOTTOM          (15U)       /* "update"                     */
#define QE_WRAP_TOP             (23U)

static const uint16_t g_qe_program[] =
{
    0x000FU,    /*  0: jmp 15            00 -> 00                          */
    0x000EU,    /*  1: jmp 14            00 -> 01  decrement               */
    0x0015U,    /*  2: jmp 21            00 -> 10  increment               */
    0x000FU,    /*  3: jmp 15            00 -> 11                          */
    0x0015U,    /*  4: jmp 21            01 -> 00  increment               */
    0x000FU,    /*  5: jmp 15            01 -> 01                          */
    0x000FU,    /*  6: jmp 15            01 -> 10                          */
    0x000EU,    /*  7: jmp 14            01 -> 11  decrement               */
    0x000EU,    /*  8: jmp 14            10 -> 00  decrement               */
    0x000FU,    /*  9: jmp 15            10 -> 01                          */
    0x000FU,    /* 10: jmp 15            10 -> 10                          */
    0x0015U,    /* 11: jmp 21            10 -> 11  increment               */
    0x000FU,    /* 12: jmp 15            11 -> 00                          */
    0x0015U,    /* 13: jmp 21            11 -> 01  increment               */
    0x008FU,    /* 14: jmp y--, 15       11 -> 10  decrement               */
    0xA0C2U,    /* 15: mov isr, y        11 -> 11; update: count to ISR    */
    0x8000U,    /* 16: push noblock                                        */
    0x60C2U,    /* 17: out isr, 2        last state                        */
    0x4002U,    /* 18: in pins, 2        new state                         */
    0xA0E6U,    /* 19: mov osr, isr                                        */
    0xA0A6U,    /* 20: mov pc, isr       jump table                        */
    0xA04AU,    /* 21: mov y, ~y         increment: y = ~(~y - 1)          */
    0x0097U,    /* 22: jmp y--, 23                                         */
    0xA04AU     /* 23: mov y, ~y                                           */
};

/* Instructions run once through SMx_INSTR before the SM starts */
#define QE_SET_Y_0              (0xE040U)   /* set y, 0                     */
#define QE_IN_PINS_2            (0x4002U)   /* in pins, 2                   */
#define QE_MOV_OSR_ISR          (0xA0E6U)   /* mov osr, isr                 */
#define QE_JMP_UPDATE           (0x000FU)   /* jmp 15                       */

#define RX_WAIT_LIMIT           (1000U)     /* new count is due in < 0.1 us */
#define PULLUP_SETTLE_MS        (1)

#define COUNT_SIGN_BIT          (1UL << (WHEEL_COUNT_BITS - 1U))

/* Minimum-bit fields. The last count is in a word of its own: any task may
 * update it (wheel_encoder_count() is safe from any task), so it must not
 * share a read-modify-write with b_inverted, which calibration changes. */
typedef struct
{
    uint32_t pin_a      : 5;            /* GPIO 0 ... 29                   */
    uint32_t sm         : 2;            /* PIO0 state machine 0 ... 3      */
    uint32_t b_inverted : 1;
    uint32_t            : 0;
    uint32_t last       : WHEEL_COUNT_BITS; /* last count from the FIFO    */
} encoder_t;

static volatile encoder_t g_encoder[WHEEL_COUNT] =
{
    { ENC_L_A, 0U, 0U, 0U },
    { ENC_R_A, 1U, 0U, 0U }
};

static bool g_b_running = false;

static inline uint32_t sm_reg(uint32_t sm0_reg, uint32_t sm)
{
    return sm0_reg + (sm * PIO_SM_STRIDE);
}

static void encoder_pins_init(uint32_t pin_a)
{
    uint32_t pin;

    for (pin = pin_a; pin <= (pin_a + 1U); pin++)
    {
        (void)gpio_set_pin(pin, GPIO_MODE_IN);
        clr_w(GPIO(pin), GPIO_PDE);
        set_w(GPIO(pin), GPIO_PUE | GPIO_IE);
        out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_PIO0);
    }
}

static void encoder_sm_start(const volatile encoder_t *p_enc)
{
    uint32_t sm = p_enc->sm;

    clr_w(PIO0_CTRL, 1UL << (PIO_CTRL_SM_ENABLE_LSB + sm));
    out_w(sm_reg(PIO0_SM0_CLKDIV, sm), PIO_CLKDIV_ONE);
    out_w(sm_reg(PIO0_SM0_EXECCTRL, sm),
          (p_enc->pin_a << PIO_EXECCTRL_JMP_PIN_LSB)
          | (QE_WRAP_TOP << PIO_EXECCTRL_WRAP_TOP_LSB)
          | (QE_WRAP_BOTTOM << PIO_EXECCTRL_WRAP_BOT_LSB));
    /* Changing FJOIN_RX flushes the FIFOs (stale counts from a restart) */
    out_w(sm_reg(PIO0_SM0_SHIFTCTRL, sm),
          PIO_SHIFTCTRL_OUT_RIGHT | PIO_SHIFTCTRL_FJOIN_RX);
    out_w(sm_reg(PIO0_SM0_SHIFTCTRL, sm), PIO_SHIFTCTRL_OUT_RIGHT);
    out_w(sm_reg(PIO0_SM0_PINCTRL, sm),
          p_enc->pin_a << PIO_PINCTRL_IN_BASE_LSB);
    set_w(PIO0_CTRL, (1UL << (PIO_CTRL_SM_RESTART_LSB + sm))
                     | (1UL << (PIO_CTRL_CLKDIV_RESTART_LSB + sm)));

    /* Count from 0, take the current pin state as the last state (no
     * spurious step at start), then enter the loop at "update" */
    out_w(sm_reg(PIO0_SM0_INSTR, sm), QE_SET_Y_0);
    out_w(sm_reg(PIO0_SM0_INSTR, sm), QE_IN_PINS_2);
    out_w(sm_reg(PIO0_SM0_INSTR, sm), QE_MOV_OSR_ISR);
    out_w(sm_reg(PIO0_SM0_INSTR, sm), QE_JMP_UPDATE);
    set_w(PIO0_CTRL, 1UL << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

/* 20-bit two's complement field -> int32_t */
static int32_t count_from_bits(uint32_t bits)
{
    bits &= WHEEL_COUNT_MASK;

    return ((bits & COUNT_SIGN_BIT) != 0U) ? (int32_t)(bits | ~WHEEL_COUNT_MASK)
                                           : (int32_t)bits;
}

/* Latest PIO count as a 20-bit field: drain the stale entries, then take one
 * fresh one. The SM pushes every <= 10 cycles; if none arrives, keep the
 * last count. */
static uint32_t encoder_read(volatile encoder_t *p_enc)
{
    uint32_t sm = p_enc->sm;
    uint32_t n = ((in_w(PIO0_FLEVEL) >> ((8U * sm) + 4U)) & 0xFU) + 1U;
    uint32_t spin;

    while (n > 0U)
    {
        spin = 0U;
        while ((in_w(PIO0_FSTAT) & (1UL << (PIO_FSTAT_RXEMPTY_LSB + sm)))
               != 0U)
        {
            spin++;
            if (spin >= RX_WAIT_LIMIT)
            {
                return p_enc->last;
            }
        }
        p_enc->last = in_w(PIO0_RXF0 + (4U * sm)) & WHEEL_COUNT_MASK;
        n--;
    }

    return p_enc->last;
}

int32_t wheel_encoder_init(bool b_left_inverted, bool b_right_inverted)
{
    uint32_t i;

    g_encoder[WHEEL_LEFT].b_inverted  = b_left_inverted ? 1U : 0U;
    g_encoder[WHEEL_RIGHT].b_inverted = b_right_inverted ? 1U : 0U;

    /* PIO0 is ours alone: refuse it if a state machine is already running */
    if ((in_w(RESETS_RESET) & RESETS_RESET_PIO0) == 0U)
    {
        if ((in_w(PIO0_CTRL) & (0xFUL << PIO_CTRL_SM_ENABLE_LSB)) != 0U)
        {
            return (int32_t)E_OBJ;
        }
    }
    clr_w(RESETS_RESET, RESETS_RESET_PIO0);         /* take out of reset */
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PIO0) == 0U)
    {
        /* PIO0 reset release takes a few cycles */
    }

    for (i = 0U; i < (sizeof(g_qe_program) / sizeof(g_qe_program[0])); i++)
    {
        out_w(PIO0_INSTR_MEM0 + (4U * i), (uint32_t)g_qe_program[i]);
    }

    encoder_pins_init(ENC_L_A);
    encoder_pins_init(ENC_R_A);
    (void)tk_dly_tsk(PULLUP_SETTLE_MS);

    encoder_sm_start(&g_encoder[WHEEL_LEFT]);
    encoder_sm_start(&g_encoder[WHEEL_RIGHT]);
    g_b_running = true;

    return (int32_t)E_OK;
}

uint32_t wheel_encoder_bits(wheel_id_t wheel)
{
    volatile encoder_t *p_enc = &g_encoder[wheel];
    uint32_t            bits;

    if (!g_b_running)
    {
        return 0U;
    }

    /* The PIO table counts B-leading as +1; A leading is forward here */
    bits = 0U - encoder_read(p_enc);

    return ((p_enc->b_inverted != 0U) ? (0U - bits) : bits) & WHEEL_COUNT_MASK;
}

int32_t wheel_encoder_count(wheel_id_t wheel)
{
    return count_from_bits(wheel_encoder_bits(wheel));
}

int32_t wheel_encoder_delta(uint32_t now_bits, uint32_t then_bits)
{
    return count_from_bits(now_bits - then_bits);
}

bool wheel_encoder_is_running(void)
{
    return g_b_running;
}

uint32_t wheel_encoder_glitches(wheel_id_t wheel)
{
    (void)wheel;

    return 0U;      /* the PIO program ignores invalid transitions */
}

void wheel_encoder_set_inverted(wheel_id_t wheel, bool b_inverted)
{
    g_encoder[wheel].b_inverted = b_inverted ? 1U : 0U;
}

bool wheel_encoder_is_inverted(wheel_id_t wheel)
{
    return (g_encoder[wheel].b_inverted != 0U);
}
