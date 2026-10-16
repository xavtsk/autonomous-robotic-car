/*
 * File: wheel_encoder.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Wheel encoder acquisition (quadrature, interrupt driven).
 *
 * Both A and B lines of each encoder interrupt on both edges (4x decoding).
 * The interrupt handler only touches this module's four pins in IO_BANK0.
 */

#include "wheel_encoder.h"
#include "../app.h"

/* Pins (Week 6 Design Review, section 2) */
#define ENC_L_A                 (16U)   /* left wheel encoder, Grove port 4  */
#define ENC_L_B                 (17U)
#define ENC_R_A                 (4U)    /* right wheel encoder, Grove port 3 */
#define ENC_R_B                 (5U)

/* IO_BANK0 GPIO interrupt (RP2040 IRQ 13) and its priority (1 = highest) */
#define ENCODER_INTNO           (13U)
#define ENCODER_INTPRI          (1)

/* IO_BANK0 interrupt registers (RP2040 datasheet, IO_BANK0 register list) */
#define IO_BANK0_INTR0          (IO_BANK0_BASE + 0x0F0U)
#define IO_BANK0_PROC0_INTE0    (IO_BANK0_BASE + 0x100U)
#define IO_BANK0_PROC0_INTS0    (IO_BANK0_BASE + 0x120U)
#define IO_BANK0_PROC_STRIDE    (0x30U)     /* PROC1 registers follow PROC0 */
#define GPIO_EDGE_LOW_HIGH      (0xCU)      /* EDGE_LOW | EDGE_HIGH bits    */
#define SIO_CPUID               (SIO_BASE + 0x000U)

#define PULLUP_SETTLE_MS        (1)

typedef struct
{
    uint32_t           pin_a;
    uint32_t           pin_b;
    volatile bool      b_inverted;
    volatile uint8_t   state;           /* last (A << 1) | B               */
    volatile int32_t   count;           /* + = forward                     */
    volatile uint32_t  glitches;        /* both lines changed at once      */
} encoder_t;

static encoder_t g_encoder[WHEEL_COUNT] =
{
    { ENC_L_A, ENC_L_B, false, 0U, 0, 0U },
    { ENC_R_A, ENC_R_B, false, 0U, 0, 0U }
};

static uint32_t g_core = 0U;            /* core that services the interrupt */

/* Index = (previous state << 2) | new state -> +1, -1 or 0 (none / invalid) */
static const int8_t g_quad_table[16] =
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static inline uint32_t reg_offset(uint32_t pin)
{
    return (pin / 8U) * 4U;
}

static inline uint32_t edge_bits(uint32_t pin)
{
    return GPIO_EDGE_LOW_HIGH << ((pin % 8U) * 4U);
}

static uint8_t encoder_read_state(const encoder_t *p_enc)
{
    uint32_t pins = in_w(GPIO_IN);

    return (uint8_t)((((pins >> p_enc->pin_a) & 1U) << 1)
                     | ((pins >> p_enc->pin_b) & 1U));
}

static void encoder_update(encoder_t *p_enc)
{
    uint8_t new_state = encoder_read_state(p_enc);
    int8_t  step = g_quad_table[(p_enc->state << 2) | new_state];

    if ((step == 0) && (new_state != p_enc->state))
    {
        p_enc->glitches++;
    }
    p_enc->state = new_state;
    p_enc->count += p_enc->b_inverted ? -step : step;
}

/* Acknowledge this pin's pending edges; true if it had any */
static bool encoder_take_irq(uint32_t pin)
{
    uint32_t ints = IO_BANK0_PROC0_INTS0 + (g_core * IO_BANK0_PROC_STRIDE)
                    + reg_offset(pin);
    uint32_t pending = in_w(ints) & edge_bits(pin);

    if (pending != 0U)
    {
        out_w(IO_BANK0_INTR0 + reg_offset(pin), pending); /* write 1: clear */
    }

    return (pending != 0U);
}

static void wheel_encoder_isr(UINT intno)
{
    bool b_left;
    bool b_right;

    (void)intno;
    b_left  = encoder_take_irq(ENC_L_A) | encoder_take_irq(ENC_L_B);
    b_right = encoder_take_irq(ENC_R_A) | encoder_take_irq(ENC_R_B);
    if (b_left)
    {
        encoder_update(&g_encoder[WHEEL_LEFT]);
    }
    if (b_right)
    {
        encoder_update(&g_encoder[WHEEL_RIGHT]);
    }
}

int32_t wheel_encoder_init(bool b_left_inverted, bool b_right_inverted)
{
    const uint32_t pins[4] = { ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B };
    T_DINT         dint;
    ER             er;
    uint32_t       i;

    g_core = in_w(SIO_CPUID) & 1U;
    g_encoder[WHEEL_LEFT].b_inverted  = b_left_inverted;
    g_encoder[WHEEL_RIGHT].b_inverted = b_right_inverted;

    for (i = 0U; i < 4U; i++)
    {
        (void)gpio_set_pin(pins[i], GPIO_MODE_IN);
        clr_w(GPIO(pins[i]), GPIO_PDE);
        set_w(GPIO(pins[i]), GPIO_PUE);
    }
    (void)tk_dly_tsk(PULLUP_SETTLE_MS);

    g_encoder[WHEEL_LEFT].state  = encoder_read_state(&g_encoder[WHEEL_LEFT]);
    g_encoder[WHEEL_RIGHT].state = encoder_read_state(&g_encoder[WHEEL_RIGHT]);

    for (i = 0U; i < 4U; i++)
    {
        out_w(IO_BANK0_INTR0 + reg_offset(pins[i]), edge_bits(pins[i]));
    }

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)wheel_encoder_isr;
    er = tk_def_int(ENCODER_INTNO, &dint);
    if (er < E_OK)
    {
        return (int32_t)er;
    }

    for (i = 0U; i < 4U; i++)
    {
        set_w(IO_BANK0_PROC0_INTE0 + (g_core * IO_BANK0_PROC_STRIDE)
              + reg_offset(pins[i]), edge_bits(pins[i]));
    }
    EnableInt(ENCODER_INTNO, ENCODER_INTPRI);

    return (int32_t)E_OK;
}

int32_t wheel_encoder_count(wheel_id_t wheel)
{
    return g_encoder[wheel].count;
}

uint32_t wheel_encoder_glitches(wheel_id_t wheel)
{
    return g_encoder[wheel].glitches;
}

void wheel_encoder_set_inverted(wheel_id_t wheel, bool b_inverted)
{
    g_encoder[wheel].b_inverted = b_inverted;
}

bool wheel_encoder_is_inverted(wheel_id_t wheel)
{
    return g_encoder[wheel].b_inverted;
}
