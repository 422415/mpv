// Standalone adapter for the MVTools path used by AJN 3.2.1 rife_cuda.py:
// Super(pel=1), Analyse(isb=True), SCDetection(thscd1=240, thscd2=130).
// SPDX-License-Identifier: GPL-2.0-or-later
#include "ajn_mvtools.h"
#include "vendor/GroupOfPlanes.h"
#include <algorithm>
#include <cmath>
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
struct Detector {
    ajn_mvtools_frame shape{};
    std::unique_ptr<Super> previous, current;
    MVAnalysisData analysis{};
    MVPlaneSet mode = YPLANE;
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
        while (((a.nBlkX * 8) >> a.nLvCount) / 8 > 0 &&
               ((a.nBlkY * 8) >> a.nLvCount) / 8 > 0) ++a.nLvCount;
        previous = std::make_unique<Super>(f, superLevels, mode);
        current = std::make_unique<Super>(f, superLevels, mode);
        shape = f;
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
struct Motion {
    GroupOfPlanes gop{};
    explicit Motion(const MVAnalysisData &a) {
        gopInit(&gop, 8, 8, a.nLvCount, 1, a.nMotionFlags, 0, 0, 0,
                a.nBlkX, a.nBlkY, a.xRatioUV, a.yRatioUV, 0, a.bitsPerSample);
    }
    ~Motion() { gopDeinit(&gop); }
};
}
int ajn_mvtools_version(void) { return AJN_MVTOOLS_ABI; }
void *ajn_mvtools_create(void) { return new (std::nothrow) Detector; }
void ajn_mvtools_destroy(void *context) { delete static_cast<Detector *>(context); }
void ajn_mvtools_reset(void *context) {
    auto *d = static_cast<Detector *>(context);
    if (d && d->previous) d->previous->pts = d->current->pts = NAN;
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
        Motion motion(d.analysis);
        std::vector<uint8_t> vectors(gopGetArraySize(&motion.gop));
        auto scale = [bits = a->bits](int value) {
            return static_cast<int>(value * ((1 << bits) - 1) / 255.0 + 0.5);
        };
        // isb=True compares frame n with n+1. Preserve that direction.
        gopSearchMVs(&motion.gop, &d.previous->frames, &d.current->frames,
            SearchHex2, 2, 1, scale(1000), scale(1200), 50, 2, 1,
            vectors.data(), 0, nullptr, 0, 50, 0, scale(10000), 24, 1, 0,
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
