/* Stationary, opt-in diagnostics. No motion/controller dependency. */
#include "subsystem3.h"
#include "ir_sensor.h"
#include "line_navigation.h"
#include "ir_bench_config.h"

#if APP_IR_BENCH
typedef struct {
    s3_sensor_frame_t frame;
    s3_navigation_output_t output;
    uint32_t samples, max_gap_us, adc_errors, rejects, timing_errors, overflows;
    uint32_t event_count[5];
    s3_barcode_result_t last_barcode;
    s3_nav_action_t last_action;
    int32_t status;
    bool ready;
} s3_bench_snapshot_t;
static ID g_sample_task, g_report_task, g_cycle, g_mutex;
static s3_navigation_config_t g_config;
static s3_navigation_t g_navigation;
static s3_bench_snapshot_t g_published;

static void ir_tick(void *unused)
{
    (void)unused;
    if (g_sample_task > 0) { (void)tk_wup_tsk(g_sample_task); }
}
static void ir_sample_task(INT stacd, void *unused)
{
    /* Static working buffers keep the sampling task's stack bounded. */
    static s3_sensor_frame_t frame;
    static s3_navigation_output_t output;
    int32_t status;
    unsigned int i;
    (void)stacd; (void)unused;
    /* Opening ADC waits for successful startup of both tasks and cyclic. */
    (void)tk_slp_tsk(TMO_FEVR);
    status = ir_sensor_init(&g_config.acquisition);
    if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK) {
        g_published.status = status;
        g_published.ready = true;
        (void)tk_unl_mtx(g_mutex);
    }
    for (;;) {
        if (status == E_OK) {
            int32_t read_status = ir_sensor_read_frame(&frame);
            if (read_status == E_OK) {
                (void)s3_navigation_process(&g_navigation, &frame, &output);
            }
            /* No ADC read, processing or console output while holding lock. */
            if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK) {
                g_published.status = read_status;
                if (read_status == E_OK) {
                    g_published.frame = frame;
                    g_published.output = output;
                    g_published.samples++;
                    if (output.sample_gap_us > g_published.max_gap_us) {
                        g_published.max_gap_us = output.sample_gap_us;
                    }
                    for (i = 0U; i < S3_SENSOR_COUNT; i++) {
                        if (frame.sensor[i].enabled && !frame.sensor[i].valid) {
                            g_published.adc_errors++;
                        }
                    }
                    for (i = 0U; i < 5U; i++) {
                        if ((output.events & (1U << i)) != 0U) { g_published.event_count[i]++; }
                    }
                    if ((output.events & S3_EVENT_BARCODE) != 0U) {
                        g_published.last_barcode = output.barcode;
                        g_published.last_action = output.action;
                    }
                    if (output.barcode_status == S3_BARCODE_REJECTED) { g_published.rejects++; }
                    if (output.barcode_status == S3_BARCODE_TIMING_ERROR ||
                        output.sample_gap_us > g_config.max_sample_age_us) { g_published.timing_errors++; }
                    if (output.barcode_status == S3_BARCODE_OVERFLOW) { g_published.overflows++; }
                } else { g_published.adc_errors++; }
                (void)tk_unl_mtx(g_mutex);
            }
        }
        /* Do not replay queued ticks as a burst of artificial samples. */
        (void)tk_can_wup(TSK_SELF);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
static void ir_report_task(INT stacd, void *unused)
{
    static s3_bench_snapshot_t snapshot; /* Reporter owns its private copy. */
    static const char *const names[S3_SENSOR_COUNT] = {"barcode", "left", "right"};
    unsigned int i;
    (void)stacd; (void)unused;
    for (;;) {
        (void)tk_dly_tsk(IR_BENCH_REPORT_MS);
        if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK) { continue; }
        snapshot = g_published;
        (void)tk_unl_mtx(g_mutex);
        if (!snapshot.ready) { continue; }
        tm_printf((UB *)"[S3 BENCH] enabled=%u cal=PROVISIONAL status=%d samples=%u gap=%uus max=%uus target=%ums\n",
            (unsigned)g_config.acquisition.enabled_mask, (int)snapshot.status,
            (unsigned)snapshot.samples, (unsigned)snapshot.output.sample_gap_us, (unsigned)snapshot.max_gap_us,
            (unsigned)IR_BENCH_SAMPLE_MS);
        for (i = 0U; i < S3_SENSOR_COUNT; i++) {
            if ((g_config.acquisition.enabled_mask & S3_SENSOR_BIT(i)) == 0U) {
                tm_printf((UB *)"  %s GP%u DISABLED/unavailable\n", names[i], (unsigned)(26U+g_config.acquisition.channel[i]));
            } else {
                tm_printf((UB *)"  %s GP%u valid=%u raw=%u/4095 dark=%u/1000 %s white=%u black=%u adc_error=%d\n",
                    names[i], (unsigned)(26U+g_config.acquisition.channel[i]),
                    (unsigned)snapshot.output.sensor[i].valid, (unsigned)snapshot.frame.sensor[i].raw,
                    (unsigned)snapshot.output.sensor[i].darkness, s3_surface_name(snapshot.output.sensor[i].surface),
                    (unsigned)g_config.calibration[i].white_raw, (unsigned)g_config.calibration[i].black_raw,
                    (int)snapshot.frame.sensor[i].error);
            }
        }
        /* Integer mm/s formatting avoids printf float support; API uses m/s. */
        tm_printf((UB *)"  line valid=%u present=%u error=%d junction_candidate=%u steer_valid=%u L=%d R=%d mm/s PROPOSAL ONLY\n",
            (unsigned)snapshot.output.line.valid, (unsigned)snapshot.output.line.present,
            (int)snapshot.output.line.error, (unsigned)snapshot.output.line.junction_candidate,
            (unsigned)snapshot.output.steering.valid, (int)(snapshot.output.steering.left_mps*1000.0f),
            (int)(snapshot.output.steering.right_mps*1000.0f));
        tm_printf((UB *)"  events(total) barcode=%u junction=%u lost=%u reacquired=%u fault=%u; last=%c %s action=%s\n",
            (unsigned)snapshot.event_count[0], (unsigned)snapshot.event_count[1],
            (unsigned)snapshot.event_count[2], (unsigned)snapshot.event_count[3], (unsigned)snapshot.event_count[4],
            snapshot.last_barcode.letter ? snapshot.last_barcode.letter : '-',
            snapshot.last_barcode.reversed ? "reverse" : "forward", s3_action_name(snapshot.last_action));
        tm_printf((UB *)"  ADC errors=%u barcode rejects=%u timing=%u overflow=%u fault_mask=%u\n",
            (unsigned)snapshot.adc_errors, (unsigned)snapshot.rejects, (unsigned)snapshot.timing_errors,
            (unsigned)snapshot.overflows, (unsigned)snapshot.output.fault_mask);
    }
}
static void bench_cleanup(void)
{
    if (g_cycle > 0) { (void)tk_stp_cyc(g_cycle); }
    if (g_report_task > 0) { (void)tk_ter_tsk(g_report_task); (void)tk_del_tsk(g_report_task); }
    if (g_sample_task > 0) { (void)tk_ter_tsk(g_sample_task); (void)tk_del_tsk(g_sample_task); }
    if (g_cycle > 0) { (void)tk_del_cyc(g_cycle); }
    if (g_mutex > 0) { (void)tk_del_mtx(g_mutex); }
    g_report_task = g_sample_task = g_cycle = g_mutex = 0;
}
#endif
ER subsystem3_init(void)
{
#if APP_IR_BENCH
    T_CMTX mutex = {0};
    T_CCYC cycle = {0};
    const ir_calibration_t calibrations[S3_SENSOR_COUNT] = {
        {IR_BENCH_WHITE_RAW, IR_BENCH_BLACK_RAW, IR_BENCH_WHITE_LIMIT, IR_BENCH_BLACK_LIMIT},
        {IR_BENCH_LEFT_WHITE_RAW, IR_BENCH_LEFT_BLACK_RAW, IR_BENCH_WHITE_LIMIT, IR_BENCH_BLACK_LIMIT},
        {IR_BENCH_RIGHT_WHITE_RAW, IR_BENCH_RIGHT_BLACK_RAW, IR_BENCH_WHITE_LIMIT, IR_BENCH_BLACK_LIMIT}
    };
    unsigned int i;
    if (g_mutex > 0) { return E_OBJ; }
    g_config = s3_navigation_defaults();
    g_config.acquisition = (s3_acquisition_config_t){
        {IR_BENCH_BARCODE_CHANNEL, IR_BENCH_LEFT_CHANNEL, IR_BENCH_RIGHT_CHANNEL}, IR_BENCH_ENABLED_MASK};
    for (i = 0U; i < S3_SENSOR_COUNT; i++) { g_config.calibration[i] = calibrations[i]; }
    if (!s3_navigation_init(&g_navigation, &g_config)) { return E_PAR; }
    g_published = (s3_bench_snapshot_t){0};
    mutex.mtxatr = TA_INHERIT;
    g_mutex = tk_cre_mtx(&mutex);
    if (g_mutex < E_OK) { return (ER)g_mutex; }
    cycle.cycatr = TA_HLNG;
    cycle.cychdr = (FP)ir_tick;
    cycle.cyctim = IR_BENCH_SAMPLE_MS;
    cycle.cycphs = IR_BENCH_SAMPLE_MS;
    g_cycle = tk_cre_cyc(&cycle);
    if (g_cycle < E_OK) { ER error = (ER)g_cycle; bench_cleanup(); return error; }
#endif
    return E_OK;
}
ER subsystem3_start(void)
{
#if APP_IR_BENCH
    ER error;
    if (g_mutex <= 0 || g_cycle <= 0 || g_sample_task > 0) { return E_OBJ; }
    g_sample_task = app_start_task((FP)ir_sample_task, S3_PRI_MAIN, TP_PRC1, "ir-sample");
    if (g_sample_task < E_OK) { error = (ER)g_sample_task; bench_cleanup(); return error; }
    g_report_task = app_start_task((FP)ir_report_task, S3_PRI_REPORT, TP_PRC1, "ir-report");
    if (g_report_task < E_OK) { error = (ER)g_report_task; bench_cleanup(); return error; }
    error = tk_sta_cyc(g_cycle);
    if (error < E_OK) { bench_cleanup(); return error; }
#endif
    return E_OK;
}
