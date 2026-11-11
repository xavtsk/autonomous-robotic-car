/* Production MQTT bridge routing; remote owner is stubbed, never motors. */
#include "../subsystem1/host_kernel.h"
#include "../../app_program/subsystem1/subsystem1.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"
#include "../../app_program/controller/remote_control.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
ID mtx_buddy1_state=1;
static unsigned ordinary,received,reply_ready,status_ready,outbox_ready;
static uint32_t linked_epoch;
static bool linked_up;
unsigned int lwip_utk_rand(void) { return 0x11223344U; }
void s3_remote_link(uint32_t epoch,bool up) { linked_epoch=epoch;linked_up=up; }
void s3_remote_receive(const uint8_t *p,unsigned n)
{ assert(n==REMOTE_COMMAND_SIZE&&p[0]==3&&p[1]==REMOTE_STOP);received++; }
unsigned s3_remote_reply_take(uint8_t *p,unsigned n)
{ assert(n>=REMOTE_REPLY_SIZE);if(!reply_ready)return 0;reply_ready=0;memset(p,0,REMOTE_REPLY_SIZE);p[0]=3;return REMOTE_REPLY_SIZE; }
unsigned s3_remote_status_take(uint8_t *p,unsigned n)
{ assert(n>=REMOTE_STATUS_SIZE);if(!status_ready)return 0;status_ready=0;memset(p,0,REMOTE_STATUS_SIZE);p[0]=3;return REMOTE_STATUS_SIZE; }
BOOL buddy1_outbox_take(BUDDY1_TELEMETRY_MSG *m)
{ if(!outbox_ready)return 0;--outbox_ready;*m=(BUDDY1_TELEMETRY_MSG){.type=BUDDY1_TOPIC_MOTION,.payload={3}};return 1; }
ER buddy1_command_enqueue(const char *topic,const UB *p,UH n)
{ (void)topic;(void)p;(void)n;ordinary++;return E_TMOUT; }
ER buddy1_update_faults(UB mask,UB value) { (void)mask;(void)value;return E_OK; }
BOOL buddy1_snapshot(BUDDY1_SNAPSHOT *s) { (void)s;return 0; }
int main(void)
{
    T_MQTT_UTK_APP_MSG message;
    unsigned char packet[REMOTE_COMMAND_SIZE]={3,REMOTE_STOP};
    mqtt_utk_app_receive("car/cmd/control",packet,sizeof packet);
    assert(received==1&&ordinary==0); /* bypass even a full ordinary queue */
    mqtt_utk_app_receive("car/cmd/snapshot",packet,1);assert(ordinary==1);
    mqtt_utk_app_receive("car/telemetry/control",packet,sizeof packet);assert(received==1);
    mqtt_utk_app_receive("car/cmd/control-extra",packet,sizeof packet);assert(received==1&&ordinary==2);
    assert(mqtt_utk_app_link(1)&&!linked_up);
    mqtt_utk_app_connection(1);assert(linked_up&&linked_epoch==0x11223344U);
    mqtt_utk_app_connection(0);assert(!linked_up);
    reply_ready=status_ready=1;
    assert(mqtt_utk_app_take(&message)&&!strcmp(message.topic,"car/reply/control")&&message.payload_len==12&&message.qos==1&&!message.retain&&!message.routine);
    assert(mqtt_utk_app_take(&message)&&!strcmp(message.topic,"car/telemetry/control")&&message.payload_len==14&&message.qos==0&&!message.retain&&message.routine);
    assert(!mqtt_utk_app_take(&message));
    /* Status continuously due on a slow link cannot starve ordinary data. */
    status_ready=1;outbox_ready=2;
    assert(mqtt_utk_app_take(&message)&&!strcmp(message.topic,"car/telemetry/motion"));
    assert(mqtt_utk_app_take(&message)&&!strcmp(message.topic,"car/telemetry/control"));
    status_ready=1;
    assert(mqtt_utk_app_take(&message)&&!strcmp(message.topic,"car/telemetry/motion"));
    assert(mqtt_utk_app_take(&message)&&!strcmp(message.topic,"car/telemetry/control"));
    puts("PASS: production remote bridge FIFO bypass, reconnect hook, reply priority and routine non-retained status");
    return 0;
}
