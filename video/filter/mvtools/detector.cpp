// Standalone adapter for the MVTools path used by AJN 3.2.1 rife_cuda.py:
// Super(pel=1), Analyse(isb=True), SCDetection(thscd1=240, thscd2=130).
// SPDX-License-Identifier: GPL-2.0-or-later
#include "ajn_mvtools.h"
#include "vendor/GroupOfPlanes.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <memory>
#include <vector>

namespace {
struct Super {
    MVGroupOfFrames frames{};
    std::vector<uint8_t> storage[3];
    uint8_t *planes[3]{};
    ptrdiff_t strides[3]{};
    double pts = NAN;
    Super(const ajn_mvtools_frame &f, int levels, int mode) {
        int bytes = (f.bits + 7) / 8;
        int x = 1 << f.sub_w, y = 1 << f.sub_h;
        // Same padded pyramid layout as mv.Super. Only pel=1 is required.
        for (int p = 0; p < (mode == YPLANE ? 1 : 3); ++p) {
            int xp = p ? x : 1, yp = p ? y : 1;
            strides[p] = ((f.width / xp + 32 / xp) * bytes + 63) & ~63;
            auto size = PlaneSuperOffset(p, f.height / yp, levels, 1,
                                        16 / yp, strides[p], y);
            storage[p].resize(static_cast<size_t>(size));
            planes[p] = storage[p].data();
        }
        mvgofInit(&frames, levels, f.width, f.height, 1, 16, 16, mode, 0,
                  x, y, f.bits);
        mvgofUpdate(&frames, planes, strides);
    }
    ~Super() { mvgofDeinit(&frames); }
    void fill(const ajn_mvtools_frame &f, MVPlaneSet mode) {
        mvgofResetState(&frames);
        for (int p = 0; p < (mode == YPLANE ? 1 : 3); ++p)
            mvfFillPlane(frames.frames[0], f.planes[p], f.strides[p], p);
        mvgofReduce(&frames, mode, 2); // Super's default bilinear rfilter.
        mvgofPad(&frames, mode);
        mvgofRefine(&frames, mode, 2); // No-op refinement for pel=1.
        pts = f.pts;
    }
};
struct Motion {
    GroupOfPlanes gop{};
    explicit Motion(const MVAnalysisData &a) {
        gopInit(&gop, 8, 8, a.nLvCount, 1, a.nMotionFlags, 0, 0, 0,
                a.nBlkX, a.nBlkY, a.xRatioUV, a.yRatioUV, 0, a.bitsPerSample);
    }
    ~Motion() { gopDeinit(&gop); }
    void reset() {
        // Analyse starts each pair with zero predictors. Reusing the previous
        // pair's vectors would introduce temporal prediction and change cuts.
        for (int i = 0; i < gop.nLevelCount; ++i) {
            auto *plane = gop.planes[i];
            std::memset(plane->vectors, 0, plane->nBlkCount * sizeof(VECTOR));
        }
    }
};
struct Detector {
    ajn_mvtools_frame shape{};
    std::unique_ptr<Super> previous, current;
    std::unique_ptr<Motion> motion;
    std::vector<uint8_t> vectors;
    MVAnalysisData analysis{};
    MVPlaneSet mode = YPLANE;
    int recent_changed[8]{};
    int recent_count = 0, recent_next = 0;
    double enhanced_pts = NAN;
    void reset_enhanced() {
        std::fill(std::begin(recent_changed), std::end(recent_changed), 0);
        recent_count = recent_next = 0;
        enhanced_pts = NAN;
    }
    bool matches(const ajn_mvtools_frame &f) const {
        return previous && shape.width == f.width && shape.height == f.height &&
            shape.bits == f.bits && shape.sub_w == f.sub_w && shape.sub_h == f.sub_h &&
            (shape.planes[1] != nullptr) == (f.planes[1] != nullptr);
    }
    void initialize(const ajn_mvtools_frame &f) {
        mode = f.planes[1] ? YUVPLANES : YPLANE;
        int x = 1 << f.sub_w, y = 1 << f.sub_h;
        int superLevels = 0;
        while (PlaneHeightLuma(f.height, superLevels, y, 16) >= y * 2 &&
               PlaneWidthLuma(f.width, superLevels, x, 16) >= x * 2) ++superLevels;
        analysis = {};
        auto &a = analysis;
        a.nBlkSizeX = a.nBlkSizeY = 8;
        a.nPel = a.nDeltaFrame = a.isBackward = 1;
        a.nWidth = f.width; a.nHeight = f.height;
        a.nBlkX = f.width / 8; a.nBlkY = f.height / 8;
        a.nHPadding = a.nVPadding = 16;
        a.xRatioUV = x; a.yRatioUV = y; a.bitsPerSample = f.bits;
        a.nMotionFlags = MOTION_IS_BACKWARD | (mode == YUVPLANES ? MOTION_USE_CHROMA_MOTION : 0);
#ifdef AJN_MVTOOLS_SSE2
        a.nMotionFlags |= MOTION_USE_SIMD;
#endif
        while (((a.nBlkX * 8) >> a.nLvCount) / 8 > 0 &&
               ((a.nBlkY * 8) >> a.nLvCount) / 8 > 0) ++a.nLvCount;
        previous = std::make_unique<Super>(f, superLevels, mode);
        current = std::make_unique<Super>(f, superLevels, mode);
        motion = std::make_unique<Motion>(a);
        vectors.resize(gopGetArraySize(&motion->gop));
        shape = f;
        reset_enhanced();
    }
};
bool valid(const ajn_mvtools_frame &f) {
    if (f.width < 16 || f.height < 16 || f.width > 8192 || f.height > 8192 ||
        f.bits < 8 || f.bits > 16 || f.sub_w < 0 || f.sub_w > 1 ||
        f.sub_h < 0 || f.sub_h > 1 || !std::isfinite(f.pts) ||
        (f.width % (1 << f.sub_w)) || (f.height % (1 << f.sub_h)) ||
        (!!f.planes[1] != !!f.planes[2])) return false;
    for (int p = 0; p < (f.planes[1] ? 3 : 1); ++p)
        if (!f.planes[p] || f.strides[p] < (f.width >> (p ? f.sub_w : 0)) * ((f.bits + 7) / 8)) return false;
    return true;
}
bool appearance_changed(const ajn_mvtools_frame &a, const ajn_mvtools_frame &b) {
    // At most 240x135 luma samples, normalized to 32 eight-bit bins. Coarse
    // bins tolerate small decode/resize differences and video grain.
    int hist[2][32]{};
    int sx = (a.width + 239) / 240, sy = (a.height + 134) / 135;
    int samples = 0;
    int64_t sum[2]{}, squared[2]{}, product = 0;
    const ajn_mvtools_frame *frames[] = {&a, &b};
    for (int y = 0; y < a.height; y += sy) {
        for (int x = 0; x < a.width; x += sx) {
            unsigned values[2];
            for (int i = 0; i < 2; ++i) {
                const auto &f = *frames[i];
                const auto *row = f.planes[0] + y * f.strides[0];
                unsigned value;
                if (f.bits == 8) value = row[x];
                else {
                    uint16_t pixel;
                    std::memcpy(&pixel, row + x * 2, sizeof(pixel));
                    value = pixel >> (f.bits - 8);
                }
                ++hist[i][std::min(value >> 3, 31u)];
                values[i] = std::min(value, 255u);
                sum[i] += values[i];
                squared[i] += values[i] * values[i];
            }
            product += values[0] * values[1];
            ++samples;
        }
    }
    int difference = 0;
    for (int i = 0; i < 32; ++i)
        difference += std::abs(hist[0][i] - hist[1][i]);
    // Histogram total variation >= 0.08 AND spatial correlation < 0.5.
    // The spatial check rejects illumination changes and shape-preserving
    // effects which can change the histogram without changing the shot.
    if (difference * 100 < samples * 16) return false;
    double covariance = double(samples) * product - double(sum[0]) * sum[1];
    double variance0 = double(samples) * squared[0] - double(sum[0]) * sum[0];
    double variance1 = double(samples) * squared[1] - double(sum[1]) * sum[1];
    return covariance <= 0 || covariance * covariance < 0.25 * variance0 * variance1;
}
}
int ajn_mvtools_version(void) { return AJN_MVTOOLS_ABI; }
void *ajn_mvtools_create(void) { return new (std::nothrow) Detector; }
void ajn_mvtools_destroy(void *context) { delete static_cast<Detector *>(context); }
void ajn_mvtools_reset(void *context) {
    auto *d = static_cast<Detector *>(context);
    if (d) {
        if (d->previous) d->previous->pts = d->current->pts = NAN;
        d->reset_enhanced();
    }
}
int ajn_mvtools_analyze_enhanced(void *context, const ajn_mvtools_frame *a,
    const ajn_mvtools_frame *b, int thscd1, int thscd2, ajn_mvtools_result *out) {
    int error = ajn_mvtools_analyze(context, a, b, thscd1, thscd2, out);
    if (error) return error;
    auto &d = *static_cast<Detector *>(context);
    if (d.enhanced_pts != a->pts) d.reset_enhanced();
    int recent_max = *std::max_element(std::begin(d.recent_changed),
                                      std::end(d.recent_changed));
    // Recover weaker cuts only after a quiet shot (at most 2% changed blocks
    // in each of the last eight pairs). Motion bursts and illumination effects
    // need the independent appearance check; there is no lookahead or holdoff
    // on legacy cuts. The first three pairs after a reset use legacy decisions.
    if (!out->cut && d.recent_count >= 3 &&
        out->changed_blocks > 75 * out->total_blocks / 256 &&
        recent_max <= out->total_blocks / 50 && appearance_changed(*a, *b))
        out->cut = 1;
    d.recent_changed[d.recent_next] = out->changed_blocks;
    d.recent_next = (d.recent_next + 1) % 8;
    d.recent_count = std::min(d.recent_count + 1, 8);
    d.enhanced_pts = b->pts;
    return 0;
}
int ajn_mvtools_analyze(void *context, const ajn_mvtools_frame *a,
    const ajn_mvtools_frame *b, int thscd1, int thscd2, ajn_mvtools_result *out) {
    if (!context || !a || !b || !out || !valid(*a) || !valid(*b) ||
        a->width != b->width || a->height != b->height || a->bits != b->bits ||
        a->sub_w != b->sub_w || a->sub_h != b->sub_h ||
        (!!a->planes[1] != !!b->planes[1]) || b->pts <= a->pts ||
        thscd1 < 0 || thscd1 > 16320 || thscd2 < 0 || thscd2 > 255) return -1;
    try {
        auto &d = *static_cast<Detector *>(context);
        if (!d.matches(*a)) d.initialize(*a);
        if (d.current->pts == a->pts) d.previous.swap(d.current);
        if (d.previous->pts != a->pts) d.previous->fill(*a, d.mode);
        d.current->fill(*b, d.mode);
        auto &motion = *d.motion;
        motion.reset();
        auto scale = [bits = a->bits](int value) {
            return static_cast<int>(value * ((1 << bits) - 1) / 255.0 + 0.5);
        };
        // isb=True compares frame n with n+1. Preserve that direction.
        gopSearchMVs(&motion.gop, &d.previous->frames, &d.current->frames,
            SearchHex2, 2, 1, scale(1000), scale(1200), 50, 2, 1,
            d.vectors.data(), 0, nullptr, 0, 50, 0, scale(10000), 24, 1, 0,
            SearchExhaustive);
        // scaleThSCD and fpobIsSceneChange, with their original integer rounding.
        int64_t sad = thscd1;
        if (d.mode == YUVPLANES)
            sad += sad / (d.analysis.xRatioUV * d.analysis.yRatioUV) * 2;
        sad = static_cast<int64_t>(sad * ((1 << a->bits) - 1) / 255.0 + 0.5);
        int total = d.analysis.nBlkX * d.analysis.nBlkY;
        int count = 0;
        for (int i = 0; i < total; ++i)
            count += motion.gop.planes[0]->vectors[i].sad > sad;
        int threshold = thscd2 * total / 256;
        *out = {count > threshold, count, total, threshold, sad};
        return 0;
    } catch (const std::exception &) {
        return -2; // A C ABI cannot propagate a C++ exception into the player.
    }
}
