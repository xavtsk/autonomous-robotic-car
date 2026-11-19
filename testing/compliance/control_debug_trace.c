/**
 * @file control_debug_trace.c
 * @brief Deterministic boundary traces for the public debug wire codec.
 *
 * This host-only program exercises the same wide validation API against the
 * fixed baseline and candidate. It prints values and destination effects,
 * including failed operations, rather than a generic success message.
 */
#include "app_program/controller/control_debug.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    const char * p_name;
    size_t offset;
    uint8_t width;
    bool b_signed;
} debug_trace_field_t;

#define DEBUG_TRACE_U(name, bits) \
    {#name, offsetof(control_debug_sample_t, name), (bits), false},
#define DEBUG_TRACE_S(name, bits) \
    {#name, offsetof(control_debug_sample_t, name), (bits), true},

static const debug_trace_field_t g_fields[] =
{
    CONTROL_DEBUG_FIELDS(DEBUG_TRACE_U, DEBUG_TRACE_S)
};

#undef DEBUG_TRACE_U
#undef DEBUG_TRACE_S

/** Print the exact wire bytes, with no dependence on host struct layout. */
static void debug_trace_packet (const control_debug_packet_t * p_packet)
{
    size_t index = 0U;
    for (index = 0U; index < sizeof(p_packet->bytes); ++index)
    {
        (void)printf("%02x", (unsigned int)p_packet->bytes[index]);
    }
}

/** Print all public output values as their exact 32-bit object representations. */
static void debug_trace_sample (const control_debug_sample_t * p_sample)
{
    size_t index = 0U;
    for (index = 0U; index < (sizeof(g_fields) / sizeof(g_fields[0])); ++index)
    {
        uint32_t value = 0U;
        /* Character access and memcpy preserve signed object bits without
         * aliasing through an incompatible uint32_t pointer. */
        const unsigned char * p_bytes = (const unsigned char *)p_sample;
        (void)memcpy(&value, &p_bytes[g_fields[index].offset], sizeof(value));
        (void)printf(" %s=%08" PRIx32, g_fields[index].p_name, value);
    }
}

/** Record acceptance, exact bytes, and both success/failure output effects. */
static void debug_trace_case (const char * p_name, int64_t value,
                              const control_debug_sample_t * p_sample)
{
    control_debug_packet_t packet;
    control_debug_sample_t decoded;
    bool b_packed = false;
    bool b_unpacked = false;
    (void)memset(&packet, 0xA5, sizeof(packet));
    (void)memset(&decoded, 0x5A, sizeof(decoded));
    b_packed = control_debug_pack(p_sample, &packet);
    b_unpacked = control_debug_unpack(&packet, &decoded);
    (void)printf("case=%s value=%" PRId64 " pack=%u unpack=%u wire=", p_name,
                 value, (unsigned int)b_packed,
                 (unsigned int)b_unpacked);
    debug_trace_packet(&packet);
    debug_trace_sample(&decoded);
    (void)putchar('\n');
}

/** Exercise each field independently; signed values include both sentinels. */
static void debug_trace_boundaries (void)
{
    size_t index = 0U;
    for (index = 0U; index < (sizeof(g_fields) / sizeof(g_fields[0])); ++index)
    {
        const debug_trace_field_t * p_field = &g_fields[index];
        const int64_t limit = INT64_C(1) << p_field->width;
        const int64_t minimum = p_field->b_signed ? -(limit / 2) : 0;
        const int64_t maximum = p_field->b_signed ? ((limit / 2) - 1) :
                                                   (limit - 1);
        const int64_t probes[] =
        {
            minimum - 1, minimum, minimum + 1, -1, 0, 1,
            maximum - 1, maximum, maximum + 1, INT32_MIN, UINT32_MAX
        };
        size_t probe = 0U;
        for (probe = 0U; probe < (sizeof(probes) / sizeof(probes[0])); ++probe)
        {
            control_debug_sample_t sample = {0};
            unsigned char * p_bytes = (unsigned char *)&sample;
            sample.motion_available = 1U;
            if (p_field->b_signed)
            {
                if ((probes[probe] < INT32_MIN) || (probes[probe] > INT32_MAX))
                {
                    /* Outside the public input type: do not narrow first. */
                }
                else
                {
                    /* The explicit bounds above make this conversion exact. */
                    const int32_t value = (int32_t)probes[probe];
                    (void)memcpy(&p_bytes[p_field->offset], &value,
                                 sizeof(value));
                    debug_trace_case(p_field->p_name, probes[probe], &sample);
                }
            }
            else if ((probes[probe] >= 0) && (probes[probe] <= UINT32_MAX))
            {
                /* The explicit bounds above make this conversion exact. */
                const uint32_t value = (uint32_t)probes[probe];
                (void)memcpy(&p_bytes[p_field->offset], &value, sizeof(value));
                debug_trace_case(p_field->p_name, probes[probe], &sample);
            }
            else
            {
                /* Negative or overflowing values are not unsigned inputs. */
            }
        }
    }
}

/** Run deterministic boundary, null, dependency, and corrupt-packet cases. */
int main (void)
{
    control_debug_sample_t sample = {0};
    control_debug_sample_t decoded;
    control_debug_packet_t packet;
    size_t index = 0U;
    debug_trace_boundaries();
    sample.motion_valid = 1U;
    debug_trace_case("valid-without-available", 1, &sample);
    sample.motion_valid = 0U;
    debug_trace_case("null-input", 0, NULL);
    (void)printf("null-destination pack=%u unpack=%u\n",
                 (unsigned int)control_debug_pack(&sample, NULL),
                 (unsigned int)control_debug_unpack(NULL, NULL));
    for (index = 0U; index < CONTROL_DEBUG_PACKET_SIZE; ++index)
    {
        bool b_result = false;
        (void)control_debug_pack(&sample, &packet);
        packet.bytes[index] = 0xFFU;
        (void)memset(&decoded, 0x5A, sizeof(decoded));
        b_result = control_debug_unpack(&packet, &decoded);
        (void)printf("corrupt-byte=%zu unpack=%u wire=",
                     index, (unsigned int)b_result);
        debug_trace_packet(&packet);
        debug_trace_sample(&decoded);
        (void)putchar('\n');
    }
    return 0;
}
