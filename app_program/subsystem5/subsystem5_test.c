#include "subsystem5.h"
#include "ultrasonic_sensor.h"
#if APP_TEST
void subsystem5_test(void)
{
    uint16_t mm=0;s5_observation_t in={0},out={0};uint8_t packet[S5_PACKET_BYTES];
    bool ok=ultrasonic_pulse_mm(2000U,&mm) && mm==343U && !ultrasonic_pulse_mm(25000U,&mm) &&
        s5_observation_pack(&in,packet) && s5_observation_unpack(packet,&out);
    tm_printf((UB *)"[TEST] subsystem5 codec/range %s; physical tests NOT RUN\n",ok?"PASS":"FAIL");
}
#endif
