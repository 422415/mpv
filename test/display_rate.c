/* SPDX-License-Identifier: LGPL-2.1-or-later */
#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include "video/out/display_rate.h"

static double choose(double fps, const unsigned *modes, int n)
{
    struct mp_display_rate rate = {.fps = fps, .variable = fps <= 0};
    double best = 0, score = -1;
    for (int i = 0; i < n; i++) {
        double hz = mp_display_rate_from_gdi(modes[i]);
        double s = mp_display_rate_score(rate, hz);
        if (s > score) { best = hz; score = s; }
    }
    return best;
}

int main(void)
{
    const unsigned tv[] = {23, 24, 25, 29, 30, 50, 59, 60};
    const unsigned integer_modes[] = {60, 48, 30, 24, 72, 120, 100, 75, 96, 144};
    const unsigned exact_higher[] = {24, 47, 144};
    double pts[600];
    for (int n = 0; n < 600; n++)
        pts[n] = round(n * 1001.0 / 24) / 1000;
    // Millisecond-rounded 23.976 timestamps remain CFR, including B-frame
    // packet reordering. Exact higher modes outrank low near-multiples.
    double temp = pts[25]; pts[25] = pts[28]; pts[28] = temp;
    double fps = mp_display_rate_analyze(pts, 600);
    assert(fabs(fps - 24000.0 / 1001) < 0.001);
    assert(fabs(choose(fps, tv, 8) - 24000.0 / 1001) < 0.001);
    assert(choose(fps, integer_modes, 10) == 24);
    assert(fabs(choose(fps, exact_higher, 3) - 48000.0 / 1001) < 0.001);

    // Two long CFR sections are mixed as a whole. Sampling just the opening
    // would choose a low rate and force a later switch.
    for (int n = 300; n < 600; n++)
        pts[n] = round(300 * 1001.0 / 24 + (n - 300) * 1001.0 / 30) / 1000;
    assert(mp_display_rate_analyze(pts, 600) == 0);
    assert(choose(0, tv, 8) == 60);
    assert(choose(0, integer_modes, 10) == 144);

    double t = 0;
    for (int n = 0; n < 600; n++) {
        pts[n] = round(t * 1000) / 1000;
        t += n % 2 ? 1.0 / 24 : 1.0 / 30;
    }
    assert(mp_display_rate_analyze(pts, 600) == 0);
    assert(mp_display_rate_analyze(NULL, 0) == 0);
    pts[10] = NAN;
    assert(mp_display_rate_analyze(pts, 600) == 0);
    const unsigned ntsc_only[] = {23, 24, 59, 60};
    assert(choose(25, ntsc_only, 4) == 60);
    puts("PASS whole-file CFR/mixed/VFR; rounded/reordered PTS; unknown cadence; refresh selection");
    return 0;
}
