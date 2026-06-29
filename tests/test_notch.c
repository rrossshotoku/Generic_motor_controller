/* Host test for the notch biquad (ADR-048). Drives sinusoids through and checks the magnitude response:
 * deep null at f0, ~-3 dB at the band edges f0±bw/2, ~unity far from the band.
 *   gcc -std=c11 -I include tests/test_notch.c src/mc_notch.c -lm -o t && ./t
 */
#include "mc_notch.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979324f
static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); g_fail++; } } while (0)

/* Steady-state gain at ftest (input amplitude 1): peak |output| over the last quarter of a 2 s run. */
static float gain(float f0, float bw, float fs, float ftest)
{
    MC_Notch_t n; MC_Notch_Init(&n); MC_Notch_SetParams(&n, f0, bw, fs);
    const int N = (int)(fs * 2.0f);
    float peak = 0.0f;
    for (int i = 0; i < N; i++)
    {
        float y = MC_Notch_Update(&n, sinf(2.0f * PI * ftest * (float)i / fs));
        if (i > (3 * N) / 4 && fabsf(y) > peak) peak = fabsf(y);
    }
    return peak;
}

int main(void)
{
    const float fs = 1000.0f, f0 = 55.0f, bw = 30.0f;   /* covers ~40..70 Hz at the 1 kHz medium rate */
    float g_c  = gain(f0, bw, fs, 55.0f);
    float g_lo = gain(f0, bw, fs, 40.0f);
    float g_hi = gain(f0, bw, fs, 70.0f);
    float g_p1 = gain(f0, bw, fs, 5.0f);
    float g_p2 = gain(f0, bw, fs, 200.0f);
    printf("gain  @55=%.3f (null)  @40=%.3f @70=%.3f (edges ~0.71)  @5=%.3f @200=%.3f (pass)\n",
           g_c, g_lo, g_hi, g_p1, g_p2);
    CHECK(g_c  < 0.10f,                  "center null too shallow: %.3f", g_c);
    CHECK(g_lo > 0.55f && g_lo < 0.85f,  "low edge not ~-3 dB: %.3f", g_lo);
    CHECK(g_hi > 0.55f && g_hi < 0.85f,  "high edge not ~-3 dB: %.3f", g_hi);
    CHECK(g_p1 > 0.92f,                  "low passband not flat: %.3f", g_p1);
    CHECK(g_p2 > 0.92f,                  "high passband not flat: %.3f", g_p2);
    printf(g_fail ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
