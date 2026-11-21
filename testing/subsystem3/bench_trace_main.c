/** @file bench_trace_main.c
 * @brief Exercise the actual APP_IR_BENCH wrapper, not a reimplementation.
 *
 * Only its
 * RTOS/ADC/console boundary is fake; loops stop after one controlled iteration.
 * No scheduling, preemption, physical timing or hardware claim is made.
 */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bench_trace_main.h"

static jmp_buf                 boundary;
static unsigned                sleeps;
static unsigned                delays;
static unsigned                printed;
static unsigned                reads;
static int                     locked;
static int32_t                 init_status;
static int32_t                 read_status;
static ir_acquisition_frame_t  next_frame;
static ir_acquisition_config_t opened;
static FP                      sample_entry = NULL;
static FP                      report_entry = NULL;

/** @brief Require an existing unlocked mutex and an indefinite wait, then
 * mark it acquired.
 */
ER tk_loc_mtx (ID h_mutex, int timeout)
{
    assert(h_mutex > 0 && TMO_FEVR == timeout && !locked);
    locked = 1;
    return E_OK;
}

/** @brief Require the acquired fake mutex, then release it and report
 * success.
 */
ER tk_unl_mtx (ID h_mutex)
{
    assert(h_mutex > 0 && locked);
    locked = 0;
    return E_OK;
}

/** @brief Verify the priority-inheritance attribute and return the configured
 * fake mutex ID.
 */
ID tk_cre_mtx (const T_CMTX * p_mutex_spec)
{
    assert(TA_INHERIT == p_mutex_spec->mtxatr);
    return 1;
}

/** @brief Require a high-level 10 ms cyclic handler and return ID two.
 */
ID tk_cre_cyc (const T_CCYC * p_cycle_spec)
{
    assert(TA_HLNG == p_cycle_spec->cycatr && 10U == p_cycle_spec->cyctim &&
           p_cycle_spec->cychdr);
    return 2;
}

/** @brief Require a valid positive object ID before acknowledging the kernel
 * request.
 */
ER tk_sta_cyc (ID h_cycle)
{
    assert(h_cycle > 0);
    return E_OK;
}

/** @brief Require a valid positive object ID before acknowledging the kernel
 * request.
 */
ER tk_stp_cyc (ID h_cycle)
{
    assert(h_cycle > 0);
    return E_OK;
}

/** @brief Require a valid positive object ID before acknowledging the kernel
 * request.
 */
ER tk_del_cyc (ID h_cycle)
{
    assert(h_cycle > 0);
    return E_OK;
}

/** @brief Require a valid positive object ID before acknowledging the kernel
 * request.
 */
ER tk_del_mtx (ID h_mutex)
{
    assert(h_mutex > 0);
    return E_OK;
}

/** @brief Require a valid positive object ID before acknowledging the kernel
 * request.
 */
ER tk_ter_tsk (ID h_task)
{
    assert(h_task > 0);
    return E_OK;
}

/** @brief Require a valid positive object ID before acknowledging the kernel
 * request.
 */
ER tk_del_tsk (ID h_task)
{
    assert(h_task > 0);
    return E_OK;
}

/** @brief Require a valid positive object ID before acknowledging the kernel
 * request.
 */
ER tk_wup_tsk (ID h_task)
{
    assert(h_task > 0);
    return E_OK;
}

/** @brief Require a current-task wake cancellation and return success.
 */
ER tk_can_wup (ID h_task)
{
    assert(TSK_SELF == h_task);
    return E_OK;
}

/** @brief Require an unlocked indefinite sleep and return control to the
 * fixture at the second sleep boundary.
 */
ER tk_slp_tsk (int timeout)
{
    assert(TMO_FEVR == timeout && !locked);
    if (++sleeps == 2U)
    {
        longjmp(boundary, 1);
    }
    return E_OK;
}

/** @brief Require an unlocked 200 ms wait and return control to the test on
 * its second delay.
 */
ER tk_dly_tsk (unsigned duration)
{
    assert(200U == duration && !locked);
    if (++delays == 2U)
    {
        longjmp(boundary, 1);
    }
    return E_OK;
}

/** @brief Check the bench task's name, core and priority, store its entry
 * and return the corresponding fixture ID.
 */
ID app_start_task (FP p_entry, PRI priority, UW processor, const char * p_name)
{
    assert(TP_PRC1 == processor);
    if (strcmp(p_name, "ir-sample") == 0)
    {
        assert(16 == priority);
        sample_entry = p_entry;
        return 3;
    }
    assert(strcmp(p_name, "ir-report") == 0 && 17 == priority);
    report_entry = p_entry;
    return 4;
}

/** @brief Check that the fixture is unlocked, count the print and forward it
 * to standard output.
 */
int tm_printf (const UB * p_format, ...)
{
    assert(!locked); /* Console output must remain outside the mutex. */
    ++printed;
    va_list arguments;
    va_start(arguments, p_format);
    int result = vprintf((const char *)p_format, arguments);
    va_end(arguments);
    return result;
}

/** @brief Copy the ADC configuration while unlocked and return the injected
 * initialization result.
 */
int32_t ir_sensor_init (const ir_acquisition_config_t * p_config)
{
    assert(!locked);
    opened = *p_config;
    return init_status;
}

/** @brief Count the unlocked read. Copy the next fixture frame only on a
 * successful read.
 */
int32_t ir_sensor_read_frame (ir_acquisition_frame_t * p_frame)
{
    assert(!locked);
    ++reads;
    if (E_OK == read_status)
    {
        *p_frame = next_frame;
    }
    return read_status;
}

/** @brief Check that the bench releases the ADC while outside its mutex.
 */
void ir_sensor_close (void)
{
    assert(!locked);
}

/** @brief Run the selected bench task entry until its fixture boundary and
 * check that it released the mutex.
 */
static void run_bench_task (FP p_entry)
{
    sleeps = delays = 0;
    if (setjmp(boundary) == 0)
    {
        ((void (*)(INT, void *))p_entry)(0, NULL);
    }
    assert(!locked);
}

/** @brief Print the case name and run the bench reporting task to its
 * fixture boundary.
 */
static void print_bench_case (const char * p_name)
{
    printf("CASE %s\n", p_name);
    run_bench_task(report_entry);
}

/** @brief Stage one timestamped barcode reading with the supplied validity
 * and error. Run the sampler and check its channel configuration.
 */
static void sample_bench_frame (uint32_t timestamp, uint16_t raw,
                                bool b_is_valid, int32_t error)
{
    /* clang-format off */
    next_frame              = (ir_acquisition_frame_t){0};
    next_frame.timestamp_us = timestamp;
    next_frame.sensor[0]    =
        (ir_acquisition_sample_t){timestamp, raw, error, true, b_is_valid};
    init_status             = read_status = E_OK;
    /* clang-format on */
    run_bench_task(sample_entry);
    assert(1U == opened.enabled_mask &&
           S3_ADC_CHANNEL_BARCODE == opened.channel[S3_SENSOR_BARCODE]);
}

/** @brief Start both stationary bench tasks against the fake ADC and kernel.
 * Check acquisition, console reporting and mutex ownership.
 */
int main (void)
{
    assert(subsystem3_init() == E_OK && subsystem3_start() == E_OK);
    assert(sample_entry && report_entry);
    print_bench_case("not ready");
    assert(0U == printed && 0U == reads);
    init_status = -1234567;
    run_bench_task(sample_entry);
    print_bench_case("startup failure with empty snapshot");
    assert(0U == reads);
    sample_bench_frame(0xffff0000U, 150, true, 0);
    print_bench_case("valid white");
    sample_bench_frame(0xffff2710U, 4095, true, 0);
    print_bench_case("valid saturated black");
    read_status = -2345678;
    run_bench_task(sample_entry);
    print_bench_case("read failure retains previous sample");
    sample_bench_frame(0xffff4e20U, 0, false, INT32_MIN);
    print_bench_case("invalid sensor with full driver error");
    sample_bench_frame(0x00004e20U, 280, true, 0);
    print_bench_case("sample after timestamp wrap and long gap");
    sample_bench_frame(0x00007530U, 280, true, 0);
    print_bench_case("valid recovery at regular interval");
    assert(6U == reads);
    puts("PASS: actual bench lifecycle/sampler/reporter diagnostics; ADC "
         "and console outside fake mutex");
    return 0;
}

/* End of file. */

