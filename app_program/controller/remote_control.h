#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
/* Fixed byte layouts shared with the dashboard; see docs/REMOTE_CONTROL_PROTOCOL.md. */
#include <stdint.h>
#include <stdbool.h>
#define REMOTE_VERSION 3U
#define REMOTE_COMMAND_SIZE 14U
#define REMOTE_STATUS_SIZE 14U
#define REMOTE_REPLY_SIZE 12U
#define REMOTE_MAX_AGE_US 2000000U
enum { REMOTE_START=1, REMOTE_STOP=2, REMOTE_RESET=3 };
enum { REMOTE_QUEUED=0, REMOTE_EXECUTED=1, REMOTE_REJECTED=2, REMOTE_CANCELLED=3 };
enum { REMOTE_NONE=0, REMOTE_INVALID=1, REMOTE_STALE=2, REMOTE_REPLAY=3,
       REMOTE_BUSY=4, REMOTE_NOT_READY=5, REMOTE_SUPERSEDED=6, REMOTE_UNAVAILABLE=7 };
enum { REMOTE_AVAILABLE=1, REMOTE_CAN_START=2, REMOTE_CAN_RESET=4, REMOTE_RUNNING=8 };
typedef struct { uint32_t id, epoch, time_us; uint8_t action; bool valid; } remote_request_t;
static inline uint32_t remote_get32(const uint8_t *p)
{ return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24; }
static inline void remote_put32(uint8_t *p,uint32_t n)
{ for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(n>>(8U*i)); }
static inline bool remote_newer(uint32_t a,uint32_t b)
{ return a!=b && (uint32_t)(a-b)<0x80000000U; }
/* Network callbacks never operate motors. The sampling owner executes through
 * exactly the same drive/calibration/fault checks as physical controls. */
void s3_remote_link(uint32_t epoch, bool up);
void s3_remote_receive(const uint8_t *packet, unsigned length);
unsigned s3_remote_reply_take(uint8_t *packet, unsigned capacity);
unsigned s3_remote_status_take(uint8_t *packet, unsigned capacity);
uint32_t s3_remote_stop_sequence(void);
void s3_remote_physical_stop(bool held);
#endif
