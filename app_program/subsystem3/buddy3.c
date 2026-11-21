/** @file buddy3.c
 * @brief Stationary, opt-in diagnostics.
 *
 * No motion/controller dependency.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 */
#include "buddy3.h"
#include "ir_sensor.h"
#include "line_navigation.h"
#include "ir_bench_config.h"
#include "navigation_codec.h"
#include "../controller/s3_integration.h"

#if APP_IR_BENCH
#define BENCH_EVENT_COUNT (5U)

typedef struct
{
    navigation_codec_raw_frame_t frame;
    navigation_codec_packet_t    output;
    uint32_t                     samples;
    uint32_t                     max_gap_us;
    uint32_t                     over_5ms;
    uint32_t                     diagnostic_barcodes;
    uint32_t                     adc_errors;
    uint32_t                     rejects;
    uint32_t                     timing_errors;
    uint32_t                     overflows;
    uint32_t                     event_count[BENCH_EVENT_COUNT];
    navigation_codec_barcode_t   last_barcode;
    unsigned int                 last_action : 3;
    int32_t                      status;
    unsigned int                 b_is_ready : 1;
} s3_bench_snapshot_t;

static ID                       gh_sample_task;
static ID                       gh_report_task;
static ID                       gh_sample_cycle;
static ID                       gh_bench_mutex;
static line_navigation_config_t g_config;
static line_navigation_t        g_navigation;
static s3_bench_snapshot_t      g_published;

static void ir_tick (void * p_unused);

static inline void
publish_bench_sample (const ir_acquisition_frame_t *       p_frame,
                      const line_navigation_output_t *     p_output,
                      const navigation_codec_raw_frame_t * p_packed_frame,
                      const navigation_codec_packet_t *    p_packed_output,
                      const navigation_codec_barcode_t *   p_packed_barcode);

static inline void
sample_bench_frame (ir_acquisition_frame_t *       p_frame,
                    line_navigation_output_t *     p_output,
                    navigation_codec_raw_frame_t * p_packed_frame,
                    navigation_codec_packet_t *    p_packed_output,
                    navigation_codec_barcode_t *   p_packed_barcode);

static void ir_sample_task (INT stacd, void * p_unused);

static void ir_report_task (INT stacd, void * p_unused);

static void bench_cleanup (void);
#endif
/** @brief Initialize the integrated motion owner or the stationary bench's
 * configured sensors, mutex and sampling timer according to the build
 * profile. Propagate initialization errors.
 */
ER subsystem3_init (void)
{
#if APP_S3_MOTION
    return (ER)s3_integration_init();
#elif APP_IR_BENCH
    T_CMTX                             mutex                         = {0};
    T_CCYC                             cycle                         = {0};
    const ir_reflectance_calibration_t calibrations[S3_SENSOR_COUNT] = {
        {IR_BENCH_WHITE_RAW, IR_BENCH_BLACK_RAW, IR_BENCH_WHITE_LIMIT,
         IR_BENCH_BLACK_LIMIT},
        {IR_BENCH_LEFT_WHITE_RAW, IR_BENCH_LEFT_BLACK_RAW, IR_BENCH_WHITE_LIMIT,
         IR_BENCH_BLACK_LIMIT},
        {IR_BENCH_RIGHT_WHITE_RAW, IR_BENCH_RIGHT_BLACK_RAW,
         IR_BENCH_WHITE_LIMIT, IR_BENCH_BLACK_LIMIT}};
    unsigned int i;
    if (gh_bench_mutex > 0)
    {
        return E_OBJ;
    }
    g_config             = line_navigation_defaults();
    g_config.acquisition = (ir_acquisition_config_t){{IR_BENCH_BARCODE_CHANNEL,
                                                      IR_BENCH_LEFT_CHANNEL,
                                                      IR_BENCH_RIGHT_CHANNEL},
                                                     IR_BENCH_ENABLED_MASK};
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        g_config.calibration[i] = calibrations[i];
    }
    if (!line_navigation_init(&g_navigation, &g_config))
    {
        return E_PAR;
    }
    g_published = (s3_bench_snapshot_t){0};
    {
        const line_navigation_output_t initial         = {0};
        const ir_acquisition_frame_t   initial_frame   = {0};
        const bc_decoder_result_t      initial_barcode = {0};
        if (!navigation_codec_pack(&initial, &g_published.output) ||
            !navigation_codec_raw_pack(&initial_frame, &g_published.frame) ||
            !navigation_codec_barcode_pack(&initial_barcode,
                                           &g_published.last_barcode))
        {
            return E_PAR;
        }
    }
    mutex.mtxatr   = TA_INHERIT;
    gh_bench_mutex = tk_cre_mtx(&mutex);
    if (gh_bench_mutex < E_OK)
    {
        return gh_bench_mutex;
    }
    /* T_CCYC uses the kernel FP address type. The cyclic handler receives
     * one void * extended-information argument. See tk/syscall.h. */
    cycle.cycatr    = TA_HLNG;
    cycle.cychdr    = (FP)ir_tick;
    cycle.cyctim    = IR_BENCH_SAMPLE_MS;
    cycle.cycphs    = IR_BENCH_SAMPLE_MS;
    gh_sample_cycle = tk_cre_cyc(&cycle);
    if (gh_sample_cycle < E_OK)
    {
        ER error = gh_sample_cycle;
        bench_cleanup();
        return error;
    }
#endif
    return E_OK;
}

/** @brief Start the integrated owner or the stationary bench sampling and
 * reporting tasks as configured. The bench releases its acquired
 * resources on startup failure.
 */
ER subsystem3_start (void)
{
#if APP_S3_MOTION
    return (ER)s3_integration_start();
#elif APP_IR_BENCH
    ER error;
    if (gh_bench_mutex <= 0 || gh_sample_cycle <= 0 || gh_sample_task > 0)
    {
        return E_OBJ;
    }
    /* Both bench entry points have the kernel INT and void * signature.
     * Pass them through the FP entry field used by app_start_task. */
    gh_sample_task =
        app_start_task((FP)ir_sample_task, S3_PRI_MAIN, TP_PRC1, "ir-sample");
    if (gh_sample_task < E_OK)
    {
        error = gh_sample_task;
        bench_cleanup();
        return error;
    }
    gh_report_task =
        app_start_task((FP)ir_report_task, S3_PRI_REPORT, TP_PRC1, "ir-report");
    if (gh_report_task < E_OK)
    {
        error = gh_report_task;
        bench_cleanup();
        return error;
    }
    error = tk_sta_cyc(gh_sample_cycle);
    if (error < E_OK)
    {
        bench_cleanup();
        return error;
    }
#endif
    return E_OK;
}

#if APP_IR_BENCH
/** @brief Wake the stationary bench sampling task if its handle is valid. The
 * kernel callback argument is unused.
 */
static void ir_tick (void * p_unused)
{
    (void)p_unused;
    if (gh_sample_task > 0)
    {
        (void)tk_wup_tsk(gh_sample_task);
    }
}

/** @brief Read IR frames on stationary bench wakes, process them without wheel
 * odometry and publish a coherent bench snapshot for reporting. Kernel
 * entry arguments are unused.
 */
static void ir_sample_task (INT stacd, void * p_unused)
{
    /* Static working buffers keep the sampling task's stack bounded. */
    static ir_acquisition_frame_t       frame;
    static line_navigation_output_t     output;
    static navigation_codec_packet_t    packed_output;
    static navigation_codec_raw_frame_t packed_frame;
    static navigation_codec_barcode_t   packed_barcode;
    int32_t                             status;
    (void)stacd;
    (void)p_unused;
    /* Opening ADC waits for successful startup of both tasks and cyclic. */
    (void)tk_slp_tsk(TMO_FEVR);
    status = ir_sensor_init(&g_config.acquisition);
    if (tk_loc_mtx(gh_bench_mutex, TMO_FEVR) == E_OK)
    {
        g_published.status     = status;
        g_published.b_is_ready = true;
        (void)tk_unl_mtx(gh_bench_mutex);
    }
    for (;;)
    {
        if (E_OK == status)
        {
            sample_bench_frame(&frame, &output, &packed_frame, &packed_output,
                               &packed_barcode);
        }
        /* Do not replay queued ticks as a burst of artificial samples. */
        (void)tk_can_wup(TSK_SELF);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/** @brief Periodically copy the bench snapshot and print its raw channels,
 * normalized readings and event counters for the operator.
 * @details Copy under the bench mutex, then release it before decoding or
 * printing. A missing snapshot or rejected packet skips this report. The
 * three decoded views share the copied snapshot's observations. This
 * stationary bench prints preview wheel speeds in mm/s for diagnosis.
 * @note The two kernel entry arguments are unused. The reporter has its
 * own private decoding storage.
 */
static void ir_report_task (INT stacd, void * p_unused)
{
    static s3_bench_snapshot_t snapshot; /* Reporter owns its private copy. */
    static ir_acquisition_frame_t   frame;
    static line_navigation_output_t output;
    static bc_decoder_result_t      last_barcode;
    static const char * const       names[S3_SENSOR_COUNT] = {"barcode", "left",
                                                              "right"};
    unsigned int                    index;
    (void)stacd;
    (void)p_unused;
    for (;;)
    {
        (void)tk_dly_tsk(IR_BENCH_REPORT_MS);
        if (tk_loc_mtx(gh_bench_mutex, TMO_FEVR) != E_OK)
        {
            continue;
        }
        snapshot = g_published;
        (void)tk_unl_mtx(gh_bench_mutex);
        if (!snapshot.b_is_ready ||
            !navigation_codec_unpack(&snapshot.output, &output) ||
            !navigation_codec_raw_unpack(&snapshot.frame, &frame) ||
            !navigation_codec_barcode_unpack(&snapshot.last_barcode,
                                             &last_barcode))
        {
            continue;
        }
        tm_printf((UB *)"[S3 BENCH] enabled=%u cal=PROVISIONAL status=%d "
                        "samples=%u gap=%uus max=%uus target=%ums\n",
                  (unsigned)g_config.acquisition.enabled_mask,
                  (int)snapshot.status, (unsigned)snapshot.samples,
                  (unsigned)output.sample_gap_us, (unsigned)snapshot.max_gap_us,
                  (unsigned)IR_BENCH_SAMPLE_MS);
        tm_printf((UB *)"  run=%s readiness=0x%x accepted_cal=0x%x odometry=%u "
                        "rearming=%u stop_requested=%u faults=0x%x\n",
                  (unsigned)S3_RUN_ACTIVE == output.lifecycle
                      ? "ACTIVE"
                      : ((unsigned)S3_RUN_FAULT_LATCHED == output.lifecycle
                             ? "FAULT-LATCHED"
                             : "INACTIVE"),
                  (unsigned)output.readiness_blockers,
                  (unsigned)output.accepted_calibration,
                  (unsigned)output.b_has_odometry,
                  (unsigned)output.b_is_rearming,
                  (unsigned)output.b_is_stop_requested,
                  (unsigned)output.latched_faults);
        tm_printf((UB *)"  readiness bits: 1=channels 2=calibration 4=sensors "
                        "8=odometry 16=fault; gaps>5ms=%u "
                        "diagnostic_letters=%u actionable_steering=%u\n",
                  (unsigned)snapshot.over_5ms,
                  (unsigned)snapshot.diagnostic_barcodes,
                  (unsigned)output.steering.b_is_valid);
        for (index = 0U; index < (unsigned)S3_SENSOR_COUNT; index++)
        {
            if ((g_config.acquisition.enabled_mask & S3_SENSOR_BIT(index)) ==
                0U)
            {
                tm_printf(
                    (UB *)"  %s GP%u DISABLED/unavailable\n", names[index],
                    (unsigned)(26U + g_config.acquisition.channel[index]));
            }
            else
            {
                tm_printf(
                    (UB *)"  %s GP%u valid=%u raw=%u/4095 dark=%u/1000 "
                          "%s white=%u black=%u adc_error=%d\n",
                    names[index],
                    (unsigned)(26U + g_config.acquisition.channel[index]),
                    (unsigned)output.sensor[index].b_is_valid,
                    (unsigned)frame.sensor[index].raw,
                    (unsigned)output.sensor[index].darkness,
                    line_navigation_surface_name(output.sensor[index].surface),
                    (unsigned)g_config.calibration[index].white_raw,
                    (unsigned)g_config.calibration[index].black_raw,
                    (int)frame.sensor[index].error);
            }
        }
        /* Integer mm/s formatting avoids printf float support; API uses m/s. */
        /* The stationary profile uses the default 0.20 m/s ceiling.
         * Convert the speed proposal to millimetres per second, then
         * truncate within 0..200 for the console. */
        tm_printf(
            (UB *)"  line valid=%u present=%u error=%d junction_candidate=%u "
                  "preview_valid=%u L=%d R=%d mm/s DIAGNOSTIC PREVIEW ONLY\n",
            (unsigned)output.line.b_is_valid,
            (unsigned)output.line.b_is_line_present, (int)output.line.error,
            (unsigned)output.line.b_has_junction_candidate,
            (unsigned)output.steering_preview.b_is_valid,
            (int)(output.steering_preview.left_mps * 1000.0f),
            (int)(output.steering_preview.right_mps * 1000.0f));
        tm_printf((UB *)"  events(total) barcode=%u junction=%u lost=%u "
                        "reacquired=%u fault=%u; last=%c %s action=%s\n",
                  (unsigned)snapshot.event_count[0],
                  (unsigned)snapshot.event_count[1],
                  (unsigned)snapshot.event_count[2],
                  (unsigned)snapshot.event_count[3],
                  (unsigned)snapshot.event_count[4],
                  last_barcode.letter ? (int)last_barcode.letter : '-',
                  last_barcode.b_is_reversed ? "reverse" : "forward",
                  line_navigation_action_name(snapshot.last_action));
        tm_printf((UB *)"  ADC errors=%u barcode rejects=%u timing=%u "
                        "overflow=%u fault_mask=%u\n",
                  (unsigned)snapshot.adc_errors, (unsigned)snapshot.rejects,
                  (unsigned)snapshot.timing_errors,
                  (unsigned)snapshot.overflows, (unsigned)output.fault_mask);
    }
}

/** @brief Release the stationary bench tasks, cyclic timer and mutex during
 * setup failure and clear their handles.
 */
static void bench_cleanup (void)
{
    if (gh_sample_cycle > 0)
    {
        (void)tk_stp_cyc(gh_sample_cycle);
    }
    if (gh_report_task > 0)
    {
        (void)tk_ter_tsk(gh_report_task);
        (void)tk_del_tsk(gh_report_task);
    }
    if (gh_sample_task > 0)
    {
        (void)tk_ter_tsk(gh_sample_task);
        (void)tk_del_tsk(gh_sample_task);
    }
    if (gh_sample_cycle > 0)
    {
        (void)tk_del_cyc(gh_sample_cycle);
    }
    if (gh_bench_mutex > 0)
    {
        (void)tk_del_mtx(gh_bench_mutex);
    }
    gh_report_task = gh_sample_task = gh_sample_cycle = gh_bench_mutex = 0;
}

/** @brief Store a validated frame and accumulate its counters.
 * The caller holds the benchmark publication mutex. The input
 * packets and sensor result remain owned by the sampling task.
 */
static inline void
publish_bench_sample (const ir_acquisition_frame_t *       p_frame,
                      const line_navigation_output_t *     p_output,
                      const navigation_codec_raw_frame_t * p_packed_frame,
                      const navigation_codec_packet_t *    p_packed_output,
                      const navigation_codec_barcode_t *   p_packed_barcode)
{
    unsigned int index;

    g_published.frame  = *p_packed_frame;
    g_published.output = *p_packed_output;
    g_published.samples++;
    if (p_output->sample_gap_us > 5000U)
    {
        g_published.over_5ms++;
    }
    if (p_output->sample_gap_us > g_published.max_gap_us)
    {
        g_published.max_gap_us = p_output->sample_gap_us;
    }
    for (index = 0U; index < (unsigned)S3_SENSOR_COUNT; index++)
    {
        if (p_frame->sensor[index].b_is_enabled &&
            !p_frame->sensor[index].b_is_valid)
        {
            g_published.adc_errors++;
        }
    }
    for (index = 0U; index < BENCH_EVENT_COUNT; index++)
    {
        if ((p_output->events & (1U << index)) != 0U)
        {
            g_published.event_count[index]++;
        }
    }
    if ((unsigned)S3_BARCODE_DECODED == p_output->barcode_status)
    {
        if (p_output->b_is_diagnostic)
        {
            g_published.diagnostic_barcodes++;
        }
        g_published.last_barcode = *p_packed_barcode;
        g_published.last_action  = p_output->action;
    }
    if ((unsigned)S3_BARCODE_REJECTED == p_output->barcode_status)
    {
        g_published.rejects++;
    }
    if ((unsigned)S3_BARCODE_TIMING_ERROR == p_output->barcode_status ||
        p_output->sample_gap_us > g_config.max_sample_age_us)
    {
        g_published.timing_errors++;
    }
    if ((unsigned)S3_BARCODE_OVERFLOW == p_output->barcode_status)
    {
        g_published.overflows++;
    }
}

/** @brief Acquire, prepare and publish one stationary benchmark frame.
 * The sampling task retains the caller-owned scratch buffers. The publication
 * mutex protects only the status, validated packet copies and counters.
 */
static inline void
sample_bench_frame (ir_acquisition_frame_t *       p_frame,
                    line_navigation_output_t *     p_output,
                    navigation_codec_raw_frame_t * p_packed_frame,
                    navigation_codec_packet_t *    p_packed_output,
                    navigation_codec_barcode_t *   p_packed_barcode)
{
    int32_t read_status = ir_sensor_read_frame(p_frame);
    if (E_OK == read_status)
    {
        (void)line_navigation_process(&g_navigation, p_frame, 0, p_output);
        if (!navigation_codec_pack(p_output, p_packed_output) ||
            !navigation_codec_raw_pack(p_frame, p_packed_frame) ||
            !navigation_codec_barcode_pack(&p_output->barcode,
                                           p_packed_barcode))
        {
            read_status = E_PAR; /* Never publish truncated sensor data. */
        }
    }
    /* No ADC read, processing or console output while holding lock. */
    if (tk_loc_mtx(gh_bench_mutex, TMO_FEVR) == E_OK)
    {
        g_published.status = read_status;
        if (E_OK == read_status)
        {
            publish_bench_sample(p_frame, p_output, p_packed_frame,
                                 p_packed_output, p_packed_barcode);
        }
        else
        {
            g_published.adc_errors++;
        }
        (void)tk_unl_mtx(gh_bench_mutex);
    }
}

#endif

/* End of file. */

