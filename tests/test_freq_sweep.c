/* Host test for the stepped-sine current sweep (ADR-047). Samples at the 20 kHz fast rate and checks:
 * stays within [bias-amp, bias+amp], steps through every frequency for ~dwell each, ends past end_hz.
 *   gcc -std=c11 -I include tests/test_freq_sweep.c src/mc_freq_sweep.c -lm -o t && ./t
 */
#include "mc_freq_sweep.h"
#include <stdio.h>
#include <math.h>

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); g_fail++; } } while (0)

int main(void)
{
    const float dt = 1.0f / 20000.0f;      /* 20 kHz fast loop */
    MC_FreqSweep_t s;
    MC_FreqSweep_Init(&s);

    /* 10 -> 50 Hz, step 10, dwell 0.05 s, bias 1.0 A, amplitude 0.5 A => 5 frequencies. */
    const float start = 10.0f, end = 50.0f, step = 10.0f, dwell = 0.05f, bias = 1.0f, amp = 0.5f;
    MC_FreqSweep_Start(&s, start, end, step, dwell, bias, amp);
    CHECK(MC_FreqSweep_Active(&s), "should be active after start");

    float maxv = -1e9f, minv = 1e9f;
    float seen_hz[16]; int n_seen = 0;
    float last_hz = -1.0f;
    long  ticks = 0;
    const long max_ticks = (long)((dwell * 6.0f) / dt);   /* enough for 5 dwells + margin */

    while (MC_FreqSweep_Active(&s) && ticks < max_ticks)
    {
        float v = MC_FreqSweep_Sample(&s, dt);
        if (v > maxv) maxv = v;
        if (v < minv) minv = v;
        float f = MC_FreqSweep_CurrentHz(&s);
        if (f != last_hz) { if (n_seen < 16) seen_hz[n_seen++] = f; last_hz = f; }
        ticks++;
    }

    printf("frequencies visited: ");
    for (int i = 0; i < n_seen; i++) printf("%.0f ", seen_hz[i]);
    printf("\noutput range [%.3f, %.3f] (expect ~[%.3f, %.3f]); ended active=%d after %.3f s\n",
           minv, maxv, bias - amp, bias + amp, MC_FreqSweep_Active(&s), ticks * dt);

    CHECK(!MC_FreqSweep_Active(&s),                 "sweep should finish");
    CHECK(n_seen == 5,                              "should visit 5 frequencies, got %d", n_seen);
    CHECK(fabsf(seen_hz[0] - 10.0f) < 1e-3f,        "first freq should be 10, got %.3f", seen_hz[0]);
    CHECK(n_seen >= 5 && fabsf(seen_hz[4] - 50.0f) < 1e-3f, "last freq should be 50");
    CHECK(maxv <= bias + amp + 1e-3f,              "max %.3f exceeds bias+amp", maxv);
    CHECK(minv >= bias - amp - 1e-3f,              "min %.3f below bias-amp", minv);
    CHECK(maxv > bias + amp - 0.05f,               "amplitude not reached (max %.3f)", maxv);

    printf(g_fail ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
