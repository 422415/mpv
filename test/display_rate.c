/* SPDX-License-Identifier: LGPL-2.1-or-later */
#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include "video/out/display_rate.h"

static struct mp_display_rate feed(struct mp_display_cadence *c, double *pts,
                                   double fps1, double fps2, int frames, bool rounded)
{
    struct mp_display_rate found = {0}, rate;
    for (int i = 0; i < frames; i++) {
        *pts += 1 / (i % 2 ? fps2 : fps1);
        double timestamp = rounded ? round(*pts * 1000) / 1000 : *pts;
        if (mp_display_cadence_sample(c, timestamp, 1, &rate))
            found = rate;
    }
    assert(found.fps > 0);
    return found;
}

static double choose(struct mp_display_rate rate, const unsigned *modes, int n)
{
    double best = 0, score = -1;
    for (int i = 0; i < n; i++) {
        double hz = mp_display_rate_from_gdi(modes[i]);
        double s = mp_display_rate_score(rate, hz);
        if (s > score) {
            best = hz;
            score = s;
        }
    }
    return best;
}

int main(void)
{
    const unsigned tv[] = {23, 24, 25, 29, 30, 50, 59, 60};
    const unsigned limited[] = {50, 59};
    // The reported display exposes integer modes only. The original selector
    // incorrectly kept 144 Hz for a 23.976 fps file instead of choosing 24 Hz.
    const unsigned integer_modes[] = {60, 48, 30, 24, 72, 120, 100, 75, 96, 144};
    struct mp_display_cadence c = {0};
    double pts = 0;
    struct mp_display_rate rate = feed(&c, &pts, 24000.0/1001, 24000.0/1001, 240, true);
    assert(!rate.variable);
    assert(fabs(choose(rate, tv, 8) - 24000.0/1001) < 0.001);
    assert(choose(rate, integer_modes, 10) == 24);
    // A higher exact multiple still beats a lower approximate match.
    const unsigned exact_higher[] = {24, 47, 144};
    assert(fabs(choose(rate, exact_higher, 3) - 48000.0/1001) < 0.001);
    rate = feed(&c, &pts, 24, 30, 300, true);
    assert(rate.variable);
    assert(choose(rate, tv, 8) == 60);
    assert(choose(rate, integer_modes, 10) == 144);
    assert(fabs(choose(rate, limited, 2) - 60000.0/1001) < 0.001);
    rate = feed(&c, &pts, 30000.0/1001, 30000.0/1001, 300, true);
    assert(!rate.variable);
    assert(choose(rate, integer_modes, 10) == 30);
    assert(fabs(choose(rate, tv, 8) - 30000.0/1001) < 0.001);
    assert(fabs(choose(rate, limited, 2) - 60000.0/1001) < 0.001);
    rate = feed(&c, &pts, 24000.0/1001, 24000.0/1001, 240, true);
    assert(!rate.variable);
    assert(fabs(choose(rate, tv, 8) - 24000.0/1001) < 0.001);

    // Repeated core visits do not create zero-duration frames. A seek clears
    // confidence; a short burst after it cannot initiate a display switch.
    assert(!mp_display_cadence_sample(&c, c.last_pts, 1, &rate));
    assert(!mp_display_cadence_sample(&c, 1, 1, &rate));
    for (int i = 1; i <= 24; i++)
        assert(!mp_display_cadence_sample(&c, 1 + i / 24.0, 1, &rate));
    assert(!mp_display_cadence_sample(&c, 2.1, 2, &rate));
    assert(c.count == 0); // speed change starts a new measurement window
    assert(!mp_display_cadence_sample(&c, NAN, 1, &rate));
    assert(!c.have_pts);
    const struct mp_display_rate pal = {.fps = 25};
    const unsigned ntsc_only[] = {23, 24, 59, 60};
    assert(choose(pal, ntsc_only, 4) == 60);
    puts("PASS CFR/VFR; integer-only and limited TV modes; exact-match priority; seek/speed reset");
    return 0;
}
