/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef MP_DISPLAY_RATE_H
#define MP_DISPLAY_RATE_H

#include <stdbool.h>
#include <math.h>

struct mp_display_rate {
    double fps;
    bool variable;
    bool apply;
};

struct mp_display_cadence {
    bool have_pts;
    bool variable_confirmed; // retained for this file, including across seeks
    double last_pts, speed;
    double duration, shortest, longest;
    int count, confirmations;
    struct mp_display_rate candidate;
};

// Reset a measurement window without forgetting that this file is VFR.
// A new file must explicitly zero the whole state instead.
static inline void mp_display_cadence_reset(struct mp_display_cadence *c)
{
    *c = (struct mp_display_cadence){
        .variable_confirmed = c->variable_confirmed,
    };
}

// Sample filter-output timestamps, before presentation drops/repeats. Require
// two consecutive two-second windows; millisecond container rounding is not VFR.
static inline bool mp_display_cadence_sample(struct mp_display_cadence *c,
                                            double pts, double speed,
                                            struct mp_display_rate *out)
{
    if (!isfinite(pts) || !isfinite(speed) || speed <= 0) {
        mp_display_cadence_reset(c);
        return false;
    }
    if (!c->have_pts || speed != c->speed || pts < c->last_pts ||
        pts - c->last_pts > 0.5 * speed)
    {
        *c = (struct mp_display_cadence){
            .have_pts = true, .last_pts = pts, .speed = speed,
            .variable_confirmed = c->variable_confirmed,
        };
        return false;
    }
    if (pts == c->last_pts)
        return false; // core may revisit a frame while waiting for the VO
    double dt = (pts - c->last_pts) / speed;
    c->last_pts = pts;
    if (!c->count || dt < c->shortest)
        c->shortest = dt;
    if (dt > c->longest)
        c->longest = dt;
    c->duration += dt;
    c->count++;
    if (c->duration < 2 || c->count < 12)
        return false;

    double mean = c->duration / c->count;
    struct mp_display_rate rate = {
        .fps = 1 / mean,
        .variable = c->longest - c->shortest > 0.0015 + mean * 0.01,
    };
    bool same = c->confirmations && rate.variable == c->candidate.variable &&
                (rate.variable || fabs(rate.fps / c->candidate.fps - 1) < 0.0008);
    c->confirmations = same ? 2 : 1;
    c->candidate = rate;
    c->duration = c->shortest = c->longest = 0;
    c->count = 0;
    // Genuine VFR often contains long locally fixed-rate stretches. Once two
    // windows confirm variation, do not let those stretches switch the display
    // back down and restart the cycle. A single mixed transition window still
    // cannot latch this, so ordinary CFR section changes remain supported.
    if (c->confirmations == 2 && rate.variable)
        c->variable_confirmed = true;
    rate.variable |= c->variable_confirmed;
    *out = rate;
    return c->confirmations == 2;
}

// Prefer the lowest exact multiple for CFR, then a near multiple (for example,
// 24 Hz for 23.976 fps on displays without fractional modes). Exact matches
// always win. Otherwise use the highest available progressive mode.
static inline double mp_display_rate_score(struct mp_display_rate rate, double hz)
{
    if (!(hz > 1) || !(rate.fps > 0))
        return -1;
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
