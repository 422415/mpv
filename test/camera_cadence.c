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
    // Pan smoothing targets exact source/display multiples, including the
    // fractional source rate used with a nominal 72 Hz desktop mode.
    assert(mp_camera_pan_smoothing_needed(1001.0 / 24000, 1.0 / 72));
    assert(mp_camera_pan_smoothing_needed(1.0 / 24, 1.0 / 48));
    assert(mp_camera_pan_smoothing_needed(1.0 / 24, 1.0 / 120));
    assert(!mp_camera_pan_smoothing_needed(1.0 / 24, 1.0 / 60));
    assert(!mp_camera_pan_smoothing_needed(1.0 / 24, 1.0 / 24));
    assert(!mp_camera_pan_smoothing_needed(0, 1.0 / 72));

    // 36 fps camera on a 72 Hz display: camera positions hold for two ticks,
    // original drawings for three. In particular, tick 3 changes drawing but
    // retains tick 2's camera, requiring -1/3 of the previous source motion.
    struct mp_camera_cadence_clock pan = {0};
    for (int tick = 0; tick < 12; tick++) {
        double pts = tick / 72.0;
        double offset = mp_camera_pan_sample(&pan, pts, 1.0 / 24, 1.0 / 72, true);
        double camera = pts * 24 + offset;
        assert(fabs(camera - (tick / 2) * 2.0 / 3) < 1e-12);
        if (tick == 3) {
            assert(floor(pts * 24) == 1);
            assert(fabs(offset + 1.0 / 3) < 1e-12);
        }
        // An OSD redraw must leave the camera phase unchanged.
        assert(mp_camera_pan_sample(&pan, pts, 1.0 / 24, 1.0 / 72, true) == offset);
    }
    // A missed refresh advances the hold clock; toggling re-anchors it.
    assert(fabs(mp_camera_pan_sample(&pan, 13.0 / 72, 1.0 / 24, 1.0 / 72, true)
                + 1.0 / 3) < 1e-12);
    assert(mp_camera_pan_sample(&pan, 14.0 / 72, 1.0 / 24, 1.0 / 72, false) == 0);
    assert(mp_camera_pan_sample(&pan, 15.0 / 72, 1.0 / 24, 1.0 / 72, true) == 0);
    assert(mp_camera_pan_sample(&pan, 0, 1.0 / 24, 1.0 / 72, true) == 0);

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
