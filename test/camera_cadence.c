#include <assert.h>
#include "video/out/gpu_next/camera_cadence.h"

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
    return 0;
}
