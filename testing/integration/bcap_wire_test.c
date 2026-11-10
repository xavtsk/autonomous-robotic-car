/* Real capture encoder; vectors are independently decoded by Python. */
#include "barcode_capture.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static bcap_t cap;
static void vector(const char *tag,const unsigned char *p,unsigned n)
{ printf("%s ",tag);for(unsigned i=0;i<n;++i)printf("%02x",p[i]);puts(""); }
int main(void)
{
    unsigned char packet[BCAP_WIRE_MAX], guard[BCAP_WIRE_MAX];
    bcap_cursor_t cur={0}, before;
    unsigned n, packets=0;
    cap.run_start_us=0xfffffff0U;cap.episodes=1;cap.used=3;
    cap.dropped=7;cap.decoded=1;cap.letters[0]='Z';cap.letter_count=1;
    cap.episode[0]=(bcap_episode_t){.first=0,.count=3,.start_ms=20};
    cap.pool[0]=(bcap_sample_t){.raw=100,.dark4=0,.flags=1,.mm=0,.dt_ms=9};
    cap.pool[1]=(bcap_sample_t){.raw=3500,.dark4=250,.flags=34,.mm=1,.dt_ms=10};
    cap.pool[2]=(bcap_sample_t){.raw=200,.dark4=4,.flags=5,.mm=2,.dt_ms=0};
    memset(packet,0xa5,sizeof packet);memcpy(guard,packet,sizeof guard);before=cur;
    assert(!bcap_binary_next(&cap,&cur,packet,5));
    assert(!memcmp(&cur,&before,sizeof cur)&&!memcmp(packet,guard,sizeof guard));
    while((n=bcap_binary_next(&cap,&cur,packet,sizeof packet))!=0) {
        assert(n<=BCAP_WIRE_MAX&&packet[0]==2);vector("BCAP_VECTOR",packet,n);packets++;
    }
    assert(packets==4&&cur.end_done);
    /* Max-length summary, header timestamp and all 24 maximum-valued samples. */
    cap.letter_count=8;memcpy(cap.letters,"ABCDEFGZ",8);
    cap.used=24;cap.episode[0].count=24;cap.episode[0].start_ms=4294967U;
    for(unsigned i=0;i<24;++i)
        cap.pool[i]=(bcap_sample_t){.raw=4095,.dark4=250,.flags=0xf2,.mm=65535,.dt_ms=255};
    cur=(bcap_cursor_t){0};assert(bcap_binary_next(&cap,&cur,packet,sizeof packet)==21);
    assert(bcap_binary_next(&cap,&cur,packet,sizeof packet)==11);
    before=cur;memset(packet,0xa5,sizeof packet);memcpy(guard,packet,sizeof guard);
    assert(!bcap_binary_next(&cap,&cur,packet,163));
    assert(!memcmp(&cur,&before,sizeof cur)&&!memcmp(packet,guard,sizeof guard));
    assert(bcap_binary_next(&cap,&cur,packet,sizeof packet)==164);
    vector("BCAP_BOUNDARY",packet,164);
    /* Wireless batches do not change the six-sample USB text cursor. */
    bcap_cursor_t usb={.summary_done=1,.header_done=1};char line[160];
    assert(bcap_format_next(&cap,&usb,line,sizeof line)&&usb.sample==6);
    /* Invalid data never advances the cursor or silently truncates. */
    cur=(bcap_cursor_t){.summary_done=1,.header_done=1};before=cur;
    cap.pool[0].raw=4096;assert(!bcap_binary_next(&cap,&cur,packet,sizeof packet));
    assert(!memcmp(&cur,&before,sizeof cur));cap.pool[0].raw=0;
    cap.pool[0].dark4=251;assert(!bcap_binary_next(&cap,&cur,packet,sizeof packet));
    cap.pool[0].dark4=0;cap.pool[0].flags=3;
    assert(!bcap_binary_next(&cap,&cur,packet,sizeof packet));
    cap.pool[0].flags=0;cap.letter_count=9;cur=(bcap_cursor_t){0};
    assert(!bcap_binary_next(&cap,&cur,packet,sizeof packet));
    /* Multiple batches and episodes retain every sample, including short tails. */
    memset(&cap,0,sizeof cap);cap.run_start_us=123;cap.episodes=2;cap.used=76;
    cap.episode[0]=(bcap_episode_t){.first=0,.count=49,.start_ms=0};
    cap.episode[1]=(bcap_episode_t){.first=49,.count=27,.start_ms=1000};
    for(unsigned i=0;i<cap.used;++i)
        cap.pool[i]=(bcap_sample_t){.raw=i,.dark4=i,.flags=0,.mm=i,.dt_ms=10};
    cur=(bcap_cursor_t){0};packets=0;
    while((n=bcap_binary_next(&cap,&cur,packet,sizeof packet))!=0) {
        vector("BCAP_BATCH_VECTOR",packet,n);packets++;
    }
    assert(packets==9&&cur.end_done&&cur.episode==2);
    before=cur;memcpy(guard,packet,sizeof guard);
    assert(!bcap_binary_next(&cap,&cur,packet,sizeof packet));
    assert(!memcmp(&cur,&before,sizeof cur)&&!memcmp(packet,guard,sizeof guard));
    /* Final full batch reaches exactly the capture pool's boundary. */
    cap.episodes=1;cap.used=BCAP_POOL;cap.episode[0].count=BCAP_POOL;
    for(unsigned i=BCAP_POOL-24;i<BCAP_POOL;++i)
        cap.pool[i]=(bcap_sample_t){.raw=i,.dark4=0,.flags=0,.mm=i,.dt_ms=0};
    cur=(bcap_cursor_t){.summary_done=1,.header_done=1,.sample=BCAP_POOL-24};
    assert(bcap_binary_next(&cap,&cur,packet,sizeof packet)==164&&cur.sample==BCAP_POOL);
    vector("BCAP_TAIL",packet,164);
    assert(bcap_binary_next(&cap,&cur,packet,sizeof packet)==6&&cur.end_done);
    puts("PASS: binary barcode boundaries, zero bytes, exact lengths, cursor safety and rejected reserved ranges");
}
