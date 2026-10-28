#define APP_H
#define APP_S5 1
#define S5_HARDWARE_CONFIRMED 1
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "sys/sysdef.h"
#define E_OK 0
#define TRUE 1
#define FALSE 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_IN 0
static uint32_t timer,control=3U,fifo=1000U;static bool empty=true,echo;
static unsigned program_writes,triggers,cc,wrap;static bool pwm_on;
uint32_t fake_read(uint32_t address)
{
    if(address==TIMER_TIMERAWL) { return timer++; }
    if(address==RESETS_RESET) { return 0; }
    if(address==RESETS_RESET_DONE) { return RESETS_RESET_PWM|RESETS_RESET_PIO0; }
    if(address==0x50200000U) { return control; }
    if(address==0x50200004U) { return empty?1U<<10:0U; }
    if(address==0x50200028U) { return fifo; }
    return 0;
}
void fake_write(uint32_t address,uint32_t value)
{
    if(address==0x50202000U) { control|=value&15U; }
    if(address==0x50203000U) { control&=~value; }
    if(address>=0x50200048U && address<0x502000c8U) {
        assert(address>=0x502000a8U);program_writes++;
    }
    assert((control&3U)==3U); /* encoder SM0/1 never stopped */
}
int gpio_set_pin(unsigned pin,unsigned mode)
{ assert((pin==1 && mode==GPIO_MODE_OUT)||(pin==0 && mode==GPIO_MODE_IN));return 0; }
int gpio_set_val(unsigned pin,unsigned value) { assert(pin==1);if(value)triggers++;return 0; }
unsigned gpio_get_val(unsigned pin) { assert(pin==0);return echo; }
int pwm_set_pin(unsigned pin) { assert(pin==12);return 0; }
int pwm_set_wrap(unsigned pin,uint32_t value) { assert(pin==12);wrap=value;return 0; }
int pwm_set_cc(unsigned pin,uint32_t value) { assert(pin==12);cc=value;return 0; }
int pwm_set_enabled(unsigned pin,bool enabled) { assert(pin==12);pwm_on=enabled;return 0; }
#include "../../app_program/subsystem5/ultrasonic_sensor.c"
#include "../../app_program/subsystem5/scan_servo.c"
int main(void)
{
    uint16_t mm;bool valid;
    assert(ultrasonic_sensor_init() && program_writes==8);
    assert(ultrasonic_sensor_trigger(0) && triggers==1 && (control&4U));
    assert(!ultrasonic_sensor_poll(100,&mm,&valid));
    empty=false;assert(ultrasonic_sensor_poll(3000,&mm,&valid) && valid && mm==343 && !(control&4U));
    empty=true;assert(ultrasonic_sensor_trigger(70000));
    assert(ultrasonic_sensor_poll(95000,&mm,&valid) && !valid);
    echo=true;assert(!ultrasonic_sensor_trigger(140000));echo=false;
    assert(scan_servo_init() && pwm_on && wrap==19999 && cc==1500);
    assert(!scan_servo_position(0) && !scan_servo_position(180));assert(scan_servo_position(30) && cc==1166);
    scan_servo_disable();assert(!pwm_on);
    puts("PASS: production PIO/PWM drivers with fake registers; confirmed pins, echo/timeout/stuck-high, servo limits, encoder SM ownership");
}
