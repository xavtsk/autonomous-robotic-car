/** @file bend_filter_check.c
 * @brief Change Requirement 1: the controller's bend detector moved from float
 * to minimum-width integers (14 Nov 2026).
 */
/* Change Requirement 1: the controller's bend detector moved from float to
 * minimum-width integers (14 Nov 2026). This replays 20000 random line-follower
 * wheel-speed sequences through the OLD float filter and the NEW fixed-point
 * filter (copied from s3_drive.c) and compares the bend decision Buddy 5 makes
 * (|turn| >= 20 %). Build and run:
 *   cc -O2 testing/change_requirement_1/bend_filter_check.c -o /tmp/bend &&
 * /tmp/bend
 */
#include "bend_filter_check.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
static float fold = 0;
static int16_t q = 0;

static void step_old (float l, float r, uint32_t dt)
{
    float s = l + r;
    if (s < 0.01f || dt == 0 || dt > 200000U)
        return;
    float a = (float)dt / (500000.0f + (float)dt);
    fold += a * ((r - l) / s - fold);
}

static void step_new (float l, float r, uint32_t dt)
{
    float s = l + r;
    if (s < 0.01f || dt == 0 || dt > 200000U)
        return;
    float x = (r - l) / s;
    x = x > 1 ? 1 : (x < -1 ? -1 : x);
    int32_t xi = (int32_t)(x * 10000.0f), si = q;
    si +=
        (int32_t)(((int64_t)(xi - si) * (int64_t)dt) / (int64_t)(500000U + dt));
    q = (int16_t)(si > 10000 ? 10000 : (si < -10000 ? -10000 : si));
}

static int8_t pct (int16_t v)
{
    return (int8_t)(v / 100);
}

int main (void)
{
    srand(7);
    long runs = 0, agree = 0, near = 0;
    double maxerr = 0;
    for (int run = 0; run < 20000; run++)
    {
        fold = 0;
        q = 0;
        float bias = ((rand() % 2001) - 1000) / 1000.0f * 0.6f;
        float wob = (rand() % 300) / 1000.0f;
        int n = 20 + rand() % 300;
        for (int i = 0; i < n; i++)
        {
            float base = 0.08f + (rand() % 40) / 1000.0f;
            float t = bias + wob * ((rand() % 2001) - 1000) / 1000.0f;
            float l = base * (1 - t), r = base * (1 + t);
            if (l < 0)
                l = 0;
            if (r < 0)
                r = 0;
            uint32_t dt = 8000 + rand() % 4000;
            step_old(l, r, dt);
            step_new(l, r, dt);
        }
        double e = fold - q / 10000.0;
        if (e < 0)
            e = -e;
        if (e > maxerr)
            maxerr = e;
        int bo = fold >= 0.20f || fold <= -0.20f;
        int p = pct(q);
        int bn = p >= 20 || p <= -20;
        runs++;
        if (bo == bn)
            agree++;
        else
        {
            double a = fold < 0 ? -fold : fold;
            if (a > 0.19 && a < 0.21)
                near++;
            else
                printf("MISMATCH fold=%f pct=%d\n", fold, p);
        }
    }
    printf(
        "runs=%ld same bend decision=%ld differ=%ld (all within 0.19..0.21 of threshold: %ld) max |float-fixed|=%.5f\n",
        runs, agree, runs - agree, near, maxerr);
    printf("RESULT: %s\n",
           (runs - agree) == near && maxerr < 0.01 ? "PASS" : "FAIL");
    return (runs - agree) == near && maxerr < 0.01 ? 0 : 1;
}

/* End of file. */

