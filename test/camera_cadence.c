#include <assert.h>
#include "video/out/gpu_next/camera_cadence.h"

static void check_rate(double source, double display, bool needed)
{
    double duration = 1 / source, vsync = 1 / display, ratio = vsync / duration;
    assert(mp_camera_cadence_needed(duration, vsync) == needed);
    if (!needed)
        return;
    struct mp_camera_cadence_clock clock = {0};
    double last = -1;
    for (int tick = 0; tick < 512; tick++) {
        double offset = mp_camera_cadence_sample(&clock, tick * vsync, duration, vsync);
        assert(fabs(offset - mp_camera_cadence_offset(tick, ratio)) < 1e-12);
        assert(isfinite(offset) && offset >= -1 && offset <= 1e-12);
        double camera = tick * ratio + offset;
        assert(camera >= last - 1e-10);
        last = camera;
    }
}

int main(void)
{
    // At 24->60, each emulated camera position is held for two refreshes.
    const double camera[] = {0, 0, 0.6, 0.6, 1.6, 1.6, 2.2, 2.2};
    for (int i = 0; i < 8; i++)
        assert(fabs(i * 0.4 + mp_camera_cadence_offset(i, 0.4) - camera[i]) < 1e-12);
    // The original poses still advance at their source PTS, independently.
    assert(floor(2 * 0.4) == 0 && floor(3 * 0.4) == 1);
    // Fractional 23.976->60 retains the same grouping without long-run drift.
    double v = (24000.0 / 1001.0) / 60.0;
    assert(fabs(mp_camera_cadence_offset(0, v)) < 1e-12);
    assert(fabs(mp_camera_cadence_offset(4000001, v) - mp_camera_cadence_offset(1, v)) < 1e-12);

    // Different actual rates exercise each grouping size, including a ratio
    // beyond the old arbitrary 32x limit. No 60 Hz assumption in the scheduler.
    check_rate(24000.0 / 1001, 30, true);
    check_rate(24, 50, true);
    check_rate(25, 60, true);
    check_rate(24, 75, true);
    check_rate(24, 165, true);
    check_rate(25, 240, true);
    check_rate(7, 240, true);
    check_rate(24000.0 / 1001, 24000.0 / 1001, false);
    check_rate(30000.0 / 1001, 60000.0 / 1001, false);
    check_rate(24, 144, false);
    check_rate(60, 30, false);
    assert(!mp_camera_cadence_needed(0, 1.0 / 60));
    assert(!mp_camera_cadence_needed(1.0 / 24, NAN));
    assert(!mp_camera_cadence_needed(INFINITY, 1.0 / 60));

    struct mp_camera_cadence_clock clock = {0};
    double frame = 1.0 / 24, vsync = 1.0 / 60;
    mp_camera_cadence_sample(&clock, 0, frame, vsync);
    mp_camera_cadence_sample(&clock, 3600, frame, vsync);
    // A small timing adjustment an hour into a file still advances one tick.
    // Dividing the whole elapsed hour by the new interval jumps ~22 ticks.
    vsync *= 1.0001;
    double pts = 3600 + vsync;
    double offset = mp_camera_cadence_sample(&clock, pts, frame, vsync);
    assert(fabs(offset + vsync / frame) < 1e-12);
    // An OSD redraw does not advance cadence; a missed presentation does.
    assert(mp_camera_cadence_sample(&clock, pts, frame, vsync) == offset);
    offset = mp_camera_cadence_sample(&clock, pts + 2 * vsync, frame, vsync);
    assert(fabs(offset - (vsync / frame - 1)) < 1e-12);

    // Monitor/refresh changes can retain the same grouping size. Re-anchor
    // explicitly instead of interpreting the old clock at the new rate.
    vsync = 1.0 / 75;
    pts += 1;
    assert(fabs(mp_camera_cadence_sample(&clock, pts, frame, vsync)) < 1e-12);
    assert(fabs(mp_camera_cadence_sample(&clock, pts + vsync, frame, vsync)
                + vsync / frame) < 1e-12);
    // Re-anchor across a VFR grouping change and a backwards seek as well.
    frame = 1.0 / 50;
    assert(fabs(mp_camera_cadence_sample(&clock, pts + 2 * vsync, frame, vsync)) < 1e-12);
    assert(fabs(mp_camera_cadence_sample(&clock, 10, frame, vsync)) < 1e-12);
    clock.valid = false; // pause, F8 off/on, or VO queue reset
    assert(fabs(mp_camera_cadence_sample(&clock, 500, frame, vsync)) < 1e-12);
    return 0;
}
