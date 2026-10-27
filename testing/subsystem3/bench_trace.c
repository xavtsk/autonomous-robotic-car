/* Exercise the actual APP_IR_BENCH wrapper, not a reimplementation. Only its
 * RTOS/ADC/console boundary is fake; loops stop after one controlled iteration.
 * No scheduling, preemption, physical timing or hardware claim is made. */
#define APP_H
#define APP_IR_BENCH 1
#define APP_TEST 0
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ir_acquisition.h"

typedef int ID;
typedef int INT;
typedef int ER;
typedef int PRI;
typedef unsigned UW;
typedef unsigned char UB;
typedef void (*FP)(void);
typedef struct { int mtxatr; } T_CMTX;
typedef struct { int cycatr; FP cychdr; unsigned cyctim, cycphs; } T_CCYC;
#define E_OK 0
#define E_OBJ (-41)
#define E_PAR (-17)
#define E_NOSPT (-9)
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define TA_INHERIT 1
#define TA_HLNG 2
#define TP_PRC1 1U

static jmp_buf boundary;
static unsigned sleeps, delays, printed, reads;
static int locked;
static int32_t init_status, read_status;
static s3_sensor_frame_t next_frame;
static s3_acquisition_config_t opened;
static FP sample_entry, report_entry;

static ER tk_loc_mtx(ID id, int timeout)
{ assert(id > 0 && timeout == TMO_FEVR && !locked); locked = 1; return E_OK; }
static ER tk_unl_mtx(ID id)
{ assert(id > 0 && locked); locked = 0; return E_OK; }
static ID tk_cre_mtx(const T_CMTX *p)
{ assert(p->mtxatr == TA_INHERIT); return 1; }
static ID tk_cre_cyc(const T_CCYC *p)
{ assert(p->cycatr == TA_HLNG && p->cyctim == 10U && p->cychdr); return 2; }
static ER tk_sta_cyc(ID id) { assert(id > 0); return E_OK; }
static ER tk_stp_cyc(ID id) { assert(id > 0); return E_OK; }
static ER tk_del_cyc(ID id) { assert(id > 0); return E_OK; }
static ER tk_del_mtx(ID id) { assert(id > 0); return E_OK; }
static ER tk_ter_tsk(ID id) { assert(id > 0); return E_OK; }
static ER tk_del_tsk(ID id) { assert(id > 0); return E_OK; }
static ER tk_wup_tsk(ID id) { assert(id > 0); return E_OK; }
static ER tk_can_wup(ID id) { assert(id == TSK_SELF); return E_OK; }
static ER tk_slp_tsk(int timeout)
{
    assert(timeout == TMO_FEVR && !locked);
    if (++sleeps == 2U) longjmp(boundary, 1);
    return E_OK;
}
static ER tk_dly_tsk(unsigned duration)
{
    assert(duration == 200U && !locked);
    if (++delays == 2U) longjmp(boundary, 1);
    return E_OK;
}
static ID app_start_task(FP entry, PRI priority, UW processor, const char *name)
{
    assert(processor == TP_PRC1);
    if (strcmp(name, "ir-sample") == 0) {
        assert(priority == 16); sample_entry = entry; return 3;
    }
    assert(strcmp(name, "ir-report") == 0 && priority == 17);
    report_entry = entry; return 4;
}
static int tm_printf(const UB *format, ...)
{
    assert(!locked); /* Console output must remain outside the mutex. */
    ++printed;
    va_list ap; va_start(ap, format);
    int result = vprintf((const char *)format, ap);
    va_end(ap); return result;
}
int32_t ir_sensor_init(const s3_acquisition_config_t *config)
{ assert(!locked); opened = *config; return init_status; }
int32_t ir_sensor_read_frame(s3_sensor_frame_t *frame)
{
    assert(!locked); ++reads;
    if (read_status == E_OK) *frame = next_frame;
    return read_status;
}
void ir_sensor_close(void) { assert(!locked); }

#include "subsystem3.c"

static void once(FP entry)
{
    sleeps = delays = 0;
    if (setjmp(boundary) == 0) ((void (*)(INT, void *))entry)(0, NULL);
    assert(!locked);
}
static void report(const char *name)
{ printf("CASE %s\n", name); once(report_entry); }
static void sample(uint32_t timestamp, uint16_t raw, bool valid, int32_t error)
{
    next_frame = (s3_sensor_frame_t){0};
    next_frame.timestamp_us = timestamp;
    next_frame.sensor[0] = (s3_raw_sample_t){timestamp, raw, error, true, valid};
    init_status = read_status = E_OK;
    once(sample_entry);
    assert(opened.enabled_mask == 1U &&
           opened.channel[S3_SENSOR_BARCODE] == S3_ADC_CHANNEL_BARCODE);
}
int main(void)
{
    assert(subsystem3_init() == E_OK && subsystem3_start() == E_OK);
    assert(sample_entry && report_entry);
    report("not ready"); assert(printed == 0U && reads == 0U);
    init_status = -1234567;
    once(sample_entry); report("startup failure with empty snapshot");
    assert(reads == 0U);
    sample(0xffff0000U, 150, true, 0); report("valid white");
    sample(0xffff2710U, 4095, true, 0); report("valid saturated black");
    read_status = -2345678;
    once(sample_entry); report("read failure retains previous sample");
    sample(0xffff4e20U, 0, false, INT32_MIN); report("invalid sensor with full driver error");
    sample(0x00004e20U, 280, true, 0); report("sample after timestamp wrap and long gap");
    sample(0x00007530U, 280, true, 0); report("valid recovery at regular interval");
    assert(reads == 6U);
    puts("PASS: actual bench lifecycle/sampler/reporter diagnostics; ADC and console outside fake mutex");
    return 0;
}
