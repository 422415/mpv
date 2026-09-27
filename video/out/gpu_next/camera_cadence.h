/* Experimental fixed-refresh camera cadence, LGPL-2.1-or-later. */
#ifndef MP_CAMERA_CADENCE_H
#define MP_CAMERA_CADENCE_H
#include <math.h>
#include <stdint.h>

// Return the camera sample offset from this display tick, in source frames.
// This is the inverse-rate schedule used by the offline emulate-60 prototype.
static inline double mp_camera_cadence_offset(int64_t tick, double vsync_ratio)
{
    int q = 1 << (int)fmax(0, floor(log2(1.0 / vsync_ratio)));
    double delta = 1.0 / (q * vsync_ratio) - 1.0;
    int phase = (int)(tick % (2 * q));
    double sign = phase < q ? 1.0 : -1.0;
    return (q * 0.5 - phase % q + sign * q * delta * 0.5) * vsync_ratio - 0.5;
}
#endif
