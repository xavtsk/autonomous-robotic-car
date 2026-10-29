/* Run the upstream course scenarios with the real S5 arbitration enabled.
 * Only the ultrasonic service is replaced with a fresh, clear-path reading.
 * Obstacle-present cases use controller_test.c. No hardware evidence here. */
#include <assert.h>
#include "s5_api.h"
#include "../integration/course_sim.c"

bool subsystem5_snapshot(s5_observation_t *out)
{
    *out = (s5_observation_t){.range_mm = 2000U,
                              .range_us = now, .range_valid = true};
    return true;
}
s5_snapshot_result_t subsystem5_read_snapshot(s5_observation_t *out)
{ return subsystem5_snapshot(out)?S5_SNAPSHOT_OK:S5_SNAPSHOT_UNAVAILABLE; }
bool subsystem5_request_scan(uint16_t id)
{
    (void)id;
    assert(!"Clear course must not request an obstacle scan");
    return false;
}
bool subsystem5_request_peek(uint16_t id, unsigned angle)
{
    (void)id; (void)angle;
    assert(!"Clear course must not request an obstacle peek");
    return false;
}
void subsystem5_cancel_scan(void) {}
void subsystem5_set_controller_status(unsigned state, unsigned fault, unsigned route)
{ (void)state; (void)fault; (void)route; }
uint8_t subsystem5_controller_status(void) { return 0U; }
motion_result_t motion_request_turn(float degrees, float speed)
{
    (void)degrees; (void)speed;
    assert(!"Clear course must not request an obstacle detour turn");
    return MOTION_ERR_BUSY;
}
