#include "subsystem4.h"
#include "tilt_estimation.h"
#include "../test_report.h"
#if APP_TEST
void subsystem4_test(void)
{
    float pitch=0, roll=0;
    const float acceleration[3]={0.5f,0.0f,0.8660254f};
    terrain_result_t source={0}, decoded={0};
    uint8_t packet[TERRAIN_PACKET_SIZE];
    TEST_INIT();
    TEST_CHECK("S4-U01",tilt_estimation(acceleration,&pitch,&roll) && pitch>29.9f && pitch<30.1f,"nose-up pitch sign");
    source.pitch_deg=-12.3f; source.flags=TERRAIN_TILT_VALID;
    terrain_pack(&source,packet); terrain_unpack(packet,&decoded);
    TEST_CHECK("S4-U02",decoded.pitch_deg < -12.29f && decoded.pitch_deg > -12.31f,"signed packed pitch");
    source.peak_m=0.6f; source.flags=TERRAIN_PEAK_VALID;
    terrain_pack(&source,packet); terrain_unpack(packet,&decoded);
    TEST_CHECK("S4-U03",!(decoded.flags&TERRAIN_PEAK_VALID),"out-of-range height invalid");
    TEST_SUMMARY("subsystem4");
}
#endif
