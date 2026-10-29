// Focused regression checks for the legacy algorithm adapter.
#include "ajn_mvtools.h"
#include "vendor/SADFunctions.h"
#include "vendor/Luma.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

static void require(bool value, const char *message) {
    if (!value) { std::fprintf(stderr, "%s\n", message); std::exit(1); }
}
static void kernels_match() {
    // Every block shape used by 8x8 analysis and its chroma planes. Source
    // blocks are packed by Analyse; reference rows have their own stride.
    for (int bits : {8, 16}) for (int width : {4, 8}) for (int height : {4, 8}) {
        int bytes = bits / 8, pitch = (width + 5) * bytes;
        std::vector<uint8_t> a(width * height * bytes), b(pitch * (height + 1));
        auto scalar = selectSADFunction(width, height, bits, 0, 0);
        auto simd = selectSADFunction(width, height, bits, 1, 0);
#ifdef AJN_MVTOOLS_SSE2
        require(simd != scalar, "SSE2 SAD dispatch must be active");
#endif
        for (int pattern = 0; pattern < 3; ++pattern) {
            for (size_t i = 0; i < a.size(); ++i)
                a[i] = pattern == 0 ? 0 : pattern == 1 ? 255 : static_cast<uint8_t>(i * 73 + 19);
            for (size_t i = 0; i < b.size(); ++i)
                b[i] = pattern == 0 ? 255 : pattern == 1 ? 0 : static_cast<uint8_t>(i * 37 + 29);
            require(scalar(a.data(), width * bytes, b.data() + bytes, pitch) ==
                    simd(a.data(), width * bytes, b.data() + bytes, pitch), "SSE2 SAD must equal scalar, including 16-bit extremes");
            if (width == 8 && height == 8) {
                auto luma = selectLumaFunction(width, height, bits, 0);
                auto fastLuma = selectLumaFunction(width, height, bits, 1);
                require(luma(b.data() + bytes, pitch) == fastLuma(b.data() + bytes, pitch), "SSE2 luma must equal scalar");
            }
        }
    }
}
static void matches_fresh(void *reused, const ajn_mvtools_frame &a, const ajn_mvtools_frame &b) {
    void *fresh = ajn_mvtools_create();
    ajn_mvtools_result expected{}, actual{};
    require(ajn_mvtools_analyze(fresh, &a, &b, 240, 130, &expected) == 0 &&
            ajn_mvtools_analyze(reused, &a, &b, 240, 130, &actual) == 0, "fresh and reused analysis");
    require(actual.cut == expected.cut && actual.changed_blocks == expected.changed_blocks &&
            actual.total_blocks == expected.total_blocks && actual.block_count_threshold == expected.block_count_threshold &&
            actual.block_sad_threshold == expected.block_sad_threshold, "reused motion workspace must not retain previous predictors");
    ajn_mvtools_destroy(fresh);
}
struct Image {
    ajn_mvtools_frame f{};
    std::vector<uint8_t> p[3];
    Image(int w, int h, int bits, bool chroma, double pts) {
        f.width = w; f.height = h; f.bits = bits; f.pts = pts;
        f.sub_w = f.sub_h = chroma ? 1 : 0;
        for (int n = 0; n < (chroma ? 3 : 1); ++n) {
            f.strides[n] = (w >> (n ? f.sub_w : 0)) * ((bits + 7) / 8);
            p[n].resize(f.strides[n] * (h >> (n ? f.sub_h : 0)));
            f.planes[n] = p[n].data();
        }
    }
    void fill(int plane, int value) {
        if (f.bits == 8) std::fill(p[plane].begin(), p[plane].end(), static_cast<uint8_t>(value));
        else for (size_t i = 0; i < p[plane].size(); i += 2) {
            p[plane][i] = value & 255; p[plane][i + 1] = value >> 8;
        }
    }
};
static void enhanced_cuts() {
    for (int bits : {8, 16}) {
        // A localized hard cut: the rest of the image stays dark, so fewer
        // than half of the blocks change. Test the full analysis path.
        Image a(128, 64, bits, false, 0), b(128, 64, bits, false, 1);
        a.fill(0, 0); b.fill(0, 0);
        int value = 80 * ((1 << bits) - 1) / 255;
        for (int y = 0; y < 64; ++y) for (int x = 40; x < 88; ++x) {
            size_t at = y * a.f.strides[0] + x * (bits / 8);
            a.p[0][at] = value & 255;
            if (bits == 16) a.p[0][at + 1] = value >> 8;
        }
        void *d = ajn_mvtools_create();
        ajn_mvtools_result legacy{}, enhanced{};
        require(ajn_mvtools_analyze(d, &a.f, &b.f, 240, 130, &legacy) == 0 && !legacy.cut &&
                legacy.changed_blocks > 75 * legacy.total_blocks / 256, "localized cut fixture must be missed by legacy");
        ajn_mvtools_reset(d);
        require(ajn_mvtools_analyze_enhanced(d, &a.f, &b.f, 240, 130, &enhanced) == 0 && !enhanced.cut,
                "enhanced mode must not invent quiet history after reset");
        ajn_mvtools_reset(d);
        for (int i = 0; i < 3; ++i) {
            ajn_mvtools_frame next = a.f; a.f.pts = i; next.pts = i + 1;
            require(ajn_mvtools_analyze_enhanced(d, &a.f, &next, 240, 130, &enhanced) == 0 && !enhanced.cut,
                    "static warmup must remain continuous");
        }
        a.f.pts = 3; b.f.pts = 4;
        require(ajn_mvtools_analyze_enhanced(d, &a.f, &b.f, 240, 130, &enhanced) == 0 && enhanced.cut &&
                enhanced.changed_blocks == legacy.changed_blocks, "enhanced mode must recover a localized cut at both depths");
        // Repeating the same amount of motion is not another quiet-shot cut.
        b.f.pts = 4; a.f.pts = 5;
        require(ajn_mvtools_analyze_enhanced(d, &b.f, &a.f, 240, 130, &enhanced) == 0, "motion return");
        b.f.pts = 6;
        require(ajn_mvtools_analyze_enhanced(d, &a.f, &b.f, 240, 130, &enhanced) == 0 && !enhanced.cut,
                "ongoing motion must not repeatedly trigger weak cuts");
        // Explicit seek and timestamp discontinuity discard quiet history.
        ajn_mvtools_reset(d); a.f.pts = 0; b.f.pts = 1;
        require(ajn_mvtools_analyze_enhanced(d, &a.f, &b.f, 240, 130, &enhanced) == 0 && !enhanced.cut,
                "enhanced seek must discard motion history");
        b.fill(0, (1 << bits) - 1); a.f.pts = 10; b.f.pts = 11;
        require(ajn_mvtools_analyze_enhanced(d, &a.f, &b.f, 240, 130, &enhanced) == 0 && enhanced.cut,
                "enhanced mode must retain strong legacy cuts without warmup");
        ajn_mvtools_destroy(d);
    }
}
int main(int argc, char **argv) {
    kernels_match();
    enhanced_cuts();
    void *d = ajn_mvtools_create(); require(d, "create");
    ajn_mvtools_result r{};
    Image a(64, 64, 8, false, 0), b(64, 64, 8, false, 1);
    require(ajn_mvtools_analyze(d, &a.f, &b.f, 240, 130, &r) == 0 && !r.cut, "identical frames");
    b.fill(0, 4); ajn_mvtools_reset(d);
    require(ajn_mvtools_analyze(d, &a.f, &b.f, 240, 130, &r) == 0 && r.cut && r.changed_blocks == 64 &&
            r.block_sad_threshold == 240 && r.block_count_threshold == 32, "legacy thresholds");
    require(ajn_mvtools_analyze(d, &a.f, &b.f, 256, 130, &r) == 0 && !r.cut, "strict SAD comparison");
    Image ca(64, 64, 10, true, 0), cb(64, 64, 10, true, 1);
    cb.fill(1, 400); cb.fill(2, 400);
    require(ajn_mvtools_analyze(d, &ca.f, &cb.f, 240, 130, &r) == 0 && r.cut &&
            r.block_sad_threshold == 1444, "10-bit chroma and threshold rounding");
    // A seek must discard a cached super frame even when timestamps repeat.
    cb.fill(1, 0); cb.fill(2, 0); ajn_mvtools_reset(d);
    require(ajn_mvtools_analyze(d, &ca.f, &cb.f, 240, 130, &r) == 0 && !r.cut, "seek reset");
    require(ajn_mvtools_analyze(d, &b.f, &a.f, 240, 130, &r) == -1, "reversed timestamps");
    require(ajn_mvtools_analyze(d, &a.f, &b.f, 16321, 130, &r) == -1, "threshold boundary");
    int width = argc > 1 ? std::atoi(argv[1]) : 1280;
    int height = argc > 2 ? std::atoi(argv[2]) : 720;
    Image panA(width, height, 8, true, 0), panB(width, height, 8, true, 1);
    // Textured content translated by four pixels; the border is replicated.
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x)
        panA.p[0][y * width + x] = static_cast<uint8_t>((x / 8 * 17 + y / 8 * 31) % 220 + 16);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x)
        panB.p[0][y * width + x] = panA.p[0][y * width + std::max(0, x - 4)];
    panA.fill(1, 128); panA.fill(2, 128); panB.fill(1, 128); panB.fill(2, 128);
    auto start = std::chrono::steady_clock::now();
    require(ajn_mvtools_analyze(d, &panA.f, &panB.f, 240, 130, &r) == 0 && !r.cut, "pan must remain continuous");
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    Image next(width, height, 8, true, 2);
    next.fill(1, 128); next.fill(2, 128);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x)
        next.p[0][y * width + x] = panB.p[0][y * width + std::max(0, x - 4)];
    matches_fresh(d, panB.f, next.f);
    panA.f.pts = 3;
    matches_fresh(d, next.f, panA.f);
    // Reinitialization and seek reset must also preserve fresh-pair results.
    matches_fresh(d, ca.f, cb.f);
    ajn_mvtools_reset(d);
    next.f.pts = 4;
    matches_fresh(d, panA.f, next.f);
    std::printf("PASS enhanced localized cuts/motion/reset, SIMD/scalar parity, workspace reuse, legacy thresholds, chroma, depth, reset and pan; %dx%d first pair %.3f ms (%d/%d changed blocks)\n",
                 width, height, ms, r.changed_blocks, r.total_blocks);
    ajn_mvtools_destroy(d);
}
