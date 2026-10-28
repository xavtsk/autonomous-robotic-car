#include "ultrasonic_sensor.h"
bool ultrasonic_pulse_mm(uint32_t us,uint16_t *mm)
{
    uint32_t value;
    if(!mm || us>24000U) { return false; }
    /* 343 m/s nominal speed of sound; round to whole millimetres. */
    value=(us*343U+1000U)/2000U;
    if(value<S5_RANGE_MIN_MM || value>S5_RANGE_MAX_MM) { return false; }
    *mm=(uint16_t)value;return true;
}
#if APP_S5
#include "../app.h"
#include <sys/sysdef.h>
#define S5_PIO 0x50200000U
#define S5_SM (S5_PIO+0x0C8U+2U*0x18U)
#define S5_ENABLE (1U<<2)
/* PIO0 SM2, instructions 24..31. Counter runs at 1 MHz, two cycles/loop.
 * Source is ultrasonic_echo.pio. No GPIO ISR, polling loop or encoder FIFO use. */
static const uint16_t program[8]={0x20A0,0xA02B,0x00DC,0x001E,0x005A,0x001E,0xA0C9,0x8020};
static uint32_t started; static bool active,ready;
#if S5_TRIG_GPIO != 1U || S5_ECHO_GPIO != 0U
#error "Buddy 5 currently claims GP1 TRIG / GP0 ECHO; reconcile resource claims before moving pins"
#endif
void ultrasonic_sensor_cancel(void)
{ out_w(S5_PIO+0x3000U,S5_ENABLE);active=false; }
bool ultrasonic_sensor_init(void)
{
    if(!S5_HARDWARE_CONFIRMED || (in_w(RESETS_RESET)&RESETS_RESET_PIO0) ||
       (in_w(S5_PIO)&S5_ENABLE)) { return false; }
    if(gpio_set_pin(S5_TRIG_GPIO,GPIO_MODE_OUT)!=E_OK ||
       gpio_set_pin(S5_ECHO_GPIO,GPIO_MODE_IN)!=E_OK) { return false; }
    (void)gpio_set_val(S5_TRIG_GPIO,0U);
    for(unsigned i=0;i<8;i++) { out_w(S5_PIO+0x48U+4U*(24U+i),program[i]); }
    out_w(S5_SM,(uint32_t)SYSCLK<<16);
    out_w(S5_SM+4U,(S5_ECHO_GPIO<<24)|(31U<<12)|(24U<<7));
    out_w(S5_SM+8U,0U);out_w(S5_SM+20U,S5_ECHO_GPIO<<15);
    ready=true;active=false;return true;
}
bool ultrasonic_sensor_trigger(uint32_t now)
{
    if(!ready || active || gpio_get_val(S5_ECHO_GPIO)) { return false; }
    ultrasonic_sensor_cancel();
    /* Toggle FIFO join to clear SM2 FIFOs, restart SM2 only. Atomic aliases
     * avoid overwriting Buddy 2's enable bits on the same PIO block. */
    out_w(S5_SM+8U,1UL<<31);out_w(S5_SM+8U,0U);
    out_w(S5_PIO+0x2000U,(1U<<6)|(1U<<10));
    out_w(S5_SM+16U,24U); /* injected JMP 24 */
    out_w(S5_PIO+0x2000U,S5_ENABLE);
    started=now;active=true;
    (void)gpio_set_val(S5_TRIG_GPIO,1U);
    uint32_t t=in_w(TIMER_TIMERAWL);
    while((uint32_t)(in_w(TIMER_TIMERAWL)-t)<12U) { /* trigger only, never echo */ }
    (void)gpio_set_val(S5_TRIG_GPIO,0U);return true;
}
bool ultrasonic_sensor_poll(uint32_t now,uint16_t *mm,bool *valid)
{
    uint32_t cycles;
    if(!active || !mm || !valid) { return false; }
    if((in_w(S5_PIO+4U)&(1U<<(8U+2U)))==0U) {
        cycles=in_w(S5_PIO+0x20U+2U*4U);ultrasonic_sensor_cancel();
        *mm=0;*valid=cycles<=12000U && ultrasonic_pulse_mm(cycles*2U,mm);return true;
    }
    if((uint32_t)(now-started)>=S5_ECHO_TIMEOUT_US) {
        ultrasonic_sensor_cancel();*mm=0;*valid=false;return true;
    }
    return false;
}
#else
bool ultrasonic_sensor_init(void) { return false; }
bool ultrasonic_sensor_trigger(uint32_t now) { (void)now;return false; }
bool ultrasonic_sensor_poll(uint32_t now,uint16_t *mm,bool *valid) { (void)now;(void)mm;(void)valid;return false; }
void ultrasonic_sensor_cancel(void) { }
#endif
