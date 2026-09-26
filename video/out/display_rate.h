/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef MP_DISPLAY_RATE_H
#define MP_DISPLAY_RATE_H

#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

struct mp_display_rate {
    double fps;
    bool variable;
    bool apply;
};

static inline int mp_display_pts_compare(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

// A short opening sample cannot distinguish CFR from a locally steady part of
// VFR. Only a complete packet scan can establish a fixed rate for this playback.
// Packet order can differ from presentation order (B-frames), so sort first.
// Return zero for mixed/unknown cadence; millisecond rounding is not variation.
static inline double mp_display_rate_analyze(double *pts, int count)
{
    if (count < 12)
        return 0;
    for (int i = 0; i < count; i++) {
        if (!isfinite(pts[i]))
            return 0;
    }
    qsort(pts, count, sizeof(*pts), mp_display_pts_compare);
    double shortest = INFINITY, longest = 0;
    for (int i = 1; i < count; i++) {
        double dt = pts[i] - pts[i - 1];
        if (dt <= 0 || dt > 0.5)
            return 0;
        shortest = fmin(shortest, dt);
        longest = fmax(longest, dt);
    }
    double mean = (pts[count - 1] - pts[0]) / (count - 1);
    return longest - shortest <= 0.0015 + mean * 0.01 ? 1 / mean : 0;
}

// Prefer the lowest exact multiple for CFR, then a near multiple (for example,
// 24 Hz for 23.976 fps on displays without fractional modes). Exact matches
// always win. Otherwise use the highest available progressive mode.
static inline double mp_display_rate_score(struct mp_display_rate rate, double hz)
{
    if (!(hz > 1) || (!rate.variable && !(rate.fps > 0)))
        return -1;
    if (rate.variable)
        return hz;
    double multiple = round(hz / rate.fps);
    if (!rate.variable && multiple >= 1) {
        double error = fabs(hz / (rate.fps * multiple) - 1);
        if (error < 0.0005)
            return 1000000 - hz;
        // Allow NTSC/integer pairs plus millisecond timestamp rounding, but
        // never PAL/NTSC mismatches such as 25 fps on a 24 Hz display.
        if (error < 0.002)
            return 500000 - hz;
    }
    return hz;
}

// GDI mode enumeration uses integer labels for fractional NTSC modes, as does
// mpv's existing GDI refresh-rate reader. Keep those modes distinct from 24/60.
static inline double mp_display_rate_from_gdi(unsigned hz)
{
    switch (hz) {
    case 23: case 29: case 47: case 59: case 71: case 89: case 95:
    case 119: case 143: case 164: case 239: case 359: case 479:
        return (hz + 1) / 1.001;
    default:
        return hz > 1 ? hz : 0;
    }
}
#endif
