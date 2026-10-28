// Focused regression checks for the legacy algorithm adapter.
#include "ajn_mvtools.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

static void require(bool value, const char *message) {
    if (!value) { std::fprintf(stderr, "%s\n", message); std::exit(1); }
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
int main(int argc, char **argv) {
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
    std::printf("PASS legacy thresholds, chroma, depth, reset and pan; %dx%d first pair %.3f ms (%d/%d changed blocks)\n",
                 width, height, ms, r.changed_blocks, r.total_blocks);
    ajn_mvtools_destroy(d);
}
