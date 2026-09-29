/* Experimental fixed-refresh camera cadence, LGPL-2.1-or-later. */
#ifndef MP_CAMERA_CADENCE_H
#define MP_CAMERA_CADENCE_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// The pan experiment preserves every source-frame boundary on an integer
// display cadence. Allow the usual 23.976/24 clock correction, not 24->60.
static inline bool mp_camera_pan_smoothing_needed(double frame_duration,
                                                  double vsync_duration,
                                                  int refreshes)
{
    if (!isfinite(frame_duration) || !isfinite(vsync_duration) ||
        frame_duration <= 0 || vsync_duration <= 0)
        return false;
    double ratio = frame_duration / vsync_duration;
    return isfinite(ratio) && ratio >= fmax(1.99, refreshes - 0.01) &&
           fabs(ratio - round(ratio)) <= 0.01;
}

static inline bool mp_camera_cadence_needed(double frame_duration,
                                            double vsync_duration)
{
    if (!isfinite(frame_duration) || !isfinite(vsync_duration) ||
        frame_duration <= 0 || vsync_duration <= 0)
        return false;
    double ratio = frame_duration / vsync_duration;
    // Already-matched repetition needs no correction. Nor can this schedule
    // reproduce all source frames on a slower display. No Hz-specific limit.
    return isfinite(ratio) && ratio > 1.01 &&
           fabs(ratio - round(ratio)) > 0.01;
}

static inline double mp_camera_cadence_group(double vsync_ratio)
{
    return exp2(fmax(0, floor(-log2(vsync_ratio))));
}

// Return the camera sample offset from this display tick, in source frames.
// This is the inverse-rate schedule used by the offline emulate-60 prototype.
static inline double mp_camera_cadence_offset(double tick, double vsync_ratio)
{
    double q = mp_camera_cadence_group(vsync_ratio);
    double delta = 1.0 / (q * vsync_ratio) - 1.0;
    double phase = fmod(tick, 2 * q);
    double sign = phase < q ? 1.0 : -1.0;
    return (q * 0.5 - fmod(phase, q) + sign * q * delta * 0.5) * vsync_ratio - 0.5;
}

struct mp_camera_cadence_clock {
    bool valid;
    uint64_t source_signature;
    double pts, vsync, group, phase, sample_pts;
};

// If camera holds divide a source drawing's display cadence, start a hold
// when that drawing becomes visible. A seek/startup correction can otherwise
// leave the hold straddling two drawings: imperfect motion estimates then
// move the background on a refresh which was supposed to remain unchanged.
static inline void mp_camera_pan_align_source(struct mp_camera_cadence_clock *c,
                                              uint64_t signature,
                                              double frame_duration,
                                              double vsync_duration,
                                              int refreshes)
{
    double ratio = frame_duration / vsync_duration;
    double ticks = round(ratio);
    if (refreshes > 1 && ticks >= refreshes && fabs(ratio - ticks) <= 0.01 &&
        fmod(ticks, refreshes) == 0 && signature != c->source_signature)
        c->valid = false;
    c->source_signature = signature;
}

// Hold camera position for an integer number of refreshes without holding the
// source drawing. A source change within the hold can require a negative
// translation using the preceding pair. Keep queue/source timestamps unchanged.
static inline double mp_camera_pan_sample(struct mp_camera_cadence_clock *c,
                                          double pts, double frame_duration,
                                          double vsync_duration, int refreshes)
{
    if (refreshes <= 1) {
        c->valid = false;
        return 0;
    }
    double elapsed = pts - c->pts;
    bool reset = !c->valid || elapsed < 0 || c->group != refreshes ||
                 fabs(vsync_duration - c->vsync) > c->vsync * 0.01;
    if (reset) {
        c->phase = 0;
        c->sample_pts = pts;
    } else {
        double ticks = floor(elapsed / vsync_duration + 0.5);
        bool advance = c->phase + ticks >= refreshes;
        c->phase = fmod(c->phase + ticks, refreshes);
        if (advance)
            c->sample_pts = pts - c->phase * vsync_duration;
    }
    c->valid = true;
    c->pts = pts;
    c->vsync = vsync_duration;
    c->group = refreshes;
    return (c->sample_pts - pts) / frame_duration;
}

static inline double mp_camera_cadence_sample(struct mp_camera_cadence_clock *c,
                                             double pts, double frame_duration,
                                             double vsync_duration)
{
    double ratio = vsync_duration / frame_duration;
    double group = mp_camera_cadence_group(ratio);
    double elapsed = pts - c->pts;
    // Advance from the last presentation, never divide the whole episode by
    // today's refresh estimate. Small clock corrections must not change the
    // phase of already elapsed display ticks. Re-anchor when the rate/group
    // changes substantially, or after a playback discontinuity.
    bool reset = !c->valid || elapsed < 0 || group != c->group ||
                 fabs(vsync_duration - c->vsync) > c->vsync * 0.01;
    if (reset) {
        c->phase = 0;
    } else {
        double ticks = floor(elapsed / vsync_duration + 0.5);
        c->phase = fmod(c->phase + ticks, 2 * group);
    }
    c->valid = true;
    c->pts = pts;
    c->vsync = vsync_duration;
    c->group = group;
    return mp_camera_cadence_offset(c->phase, ratio);
}
#endif
