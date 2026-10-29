// AJN's optional scene classifier. The legacy MVTools search is unchanged.
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <array>
#include <cmath>
#include <vector>

namespace enhanced {
struct Moments {
    int histogram[2][32]{};
    int n = 0;
    int64_t sum[2]{}, square[2]{}, product = 0, absolute = 0;
    void add(int a, int b) {
        ++n; ++histogram[0][a / 8]; ++histogram[1][b / 8];
        sum[0] += a; sum[1] += b;
        square[0] += a * a; square[1] += b * b;
        product += a * b; absolute += std::abs(a - b);
    }
    double difference() const { return n ? double(absolute) / n : 0; }
    double variance(int i) const {
        return n ? std::max(0.0, double(square[i]) / n - std::pow(double(sum[i]) / n, 2)) : 0;
    }
    double correlation() const {
        double va = double(n) * square[0] - double(sum[0]) * sum[0];
        double vb = double(n) * square[1] - double(sum[1]) * sum[1];
        return va > 0 && vb > 0 ? (double(n) * product - double(sum[0]) * sum[1]) / std::sqrt(va * vb) : 0;
    }
    double distance() const {
        int d = 0;
        for (int i = 0; i < 32; ++i) d += std::abs(histogram[0][i] - histogram[1][i]);
        return n ? d / (2.0 * n) : 0;
    }
    bool dominated() const {
        for (const auto &h : histogram)
            for (int count : h) if (count > n * .85) return true;
        return false;
    }
};
inline int pixel(const ajn_mvtools_frame &f, int x, int y) {
    const auto *row = f.planes[0] + y * f.strides[0];
    if (f.bits == 8) return row[x];
    uint16_t v; std::memcpy(&v, row + x * 2, sizeof(v));
    return v >> (f.bits - 8);
}
struct Appearance { Moments all, changed, matched; };
inline Appearance measure(const ajn_mvtools_frame &a, const ajn_mvtools_frame &b,
                          const VECTOR *vectors, int nx, int ny, int64_t threshold) {
    Appearance s;
    int sx = (a.width + 239) / 240, sy = (a.height + 134) / 135;
    for (int y = 0; y < ny * 8; y += sy) for (int x = 0; x < nx * 8; x += sx) {
        int av = pixel(a, x, y), bv = pixel(b, x, y);
        s.all.add(av, bv);
        const auto &v = vectors[(y / 8) * nx + x / 8];
        if (v.sad <= threshold) continue;
        s.changed.add(av, bv);
        // MVTools pel=1 vectors map the previous picture into the current one.
        s.matched.add(av, pixel(b, std::clamp(x + v.x, 0, b.width - 1),
                                  std::clamp(y + v.y, 0, b.height - 1)));
    }
    return s;
}

struct Point { int x, y; std::array<float, 36> descriptor; };
inline std::vector<float> thumbnail(const ajn_mvtools_frame &f, int w, int h) {
    std::vector<float> out(w * h);
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
        int x0 = x * f.width / w, x1 = (x + 1) * f.width / w;
        int y0 = y * f.height / h, y1 = (y + 1) * f.height / h, sum = 0;
        for (int yy = y0; yy < y1; ++yy) for (int xx = x0; xx < x1; ++xx) sum += pixel(f, xx, yy);
        out[y * w + x] = float(sum) / ((x1 - x0) * (y1 - y0));
    }
    return out;
}
inline std::vector<Point> points(const std::vector<float> &a,
                                 const std::vector<float> &b, int w, int h) {
    std::vector<float> score(w * h);
    for (int y = 6; y < h - 6; ++y) for (int x = 6; x < w - 6; ++x) {
        int i = y * w + x;
        if (std::abs(a[i] - b[i]) <= 3) continue;
        float gx = (a[i + 1] - a[i - 1]) * .5f, gy = (a[i + w] - a[i - w]) * .5f;
        score[i] = std::min(gx * gx, gy * gy);
    }
    std::vector<Point> out;
    for (int n = 0; n < 96; ++n) {
        auto at = std::max_element(score.begin(), score.end());
        if (*at < 4) break;
        int i = int(at - score.begin()), x = i % w, y = i / w;
        Point p{x, y, {}}; int k = 0; float sum = 0, norm = 0;
        for (int yy = y - 5; yy <= y + 5; yy += 2)
            for (int xx = x - 5; xx <= x + 5; xx += 2) sum += (p.descriptor[k++] = a[yy * w + xx]);
        for (auto &v : p.descriptor) { v -= sum / 36; norm += v * v; }
        if (norm > 100) {
            norm = std::sqrt(norm);
            for (auto &v : p.descriptor) v /= norm;
            out.push_back(p);
        }
        for (int yy = y - 4; yy <= y + 4; ++yy)
            for (int xx = x - 4; xx <= x + 4; ++xx) score[yy * w + xx] = 0;
    }
    return out;
}
// Feature correspondence is reserved for uncertain candidates. Weak-cut
// recovery needs distinct detail; overturning a strong cut needs more matching
// features. Ignore stationary matches: shared titles/borders are not continuity.
enum class Correspondence { Unknown, Distinct, Moving };
inline Correspondence correspondence(const ajn_mvtools_frame &a, const ajn_mvtools_frame &b,
                                     int required_matches) {
    int w = std::min(120, a.width), h = std::max(1, std::min(a.height, (a.height * w + a.width / 2) / a.width));
    if (h > 68) { w = std::max(1, w * 68 / h); h = 68; }
    if (w < 13 || h < 13) return Correspondence::Unknown;
    auto ta = thumbnail(a, w, h), tb = thumbnail(b, w, h);
    auto pa = points(ta, tb, w, h), pb = points(tb, ta, w, h);
    if (pa.size() < 12 || pb.size() < 12) return Correspondence::Unknown;
    std::vector<float> distance(pa.size() * pb.size());
    for (size_t i = 0; i < pa.size(); ++i) for (size_t j = 0; j < pb.size(); ++j) {
        float dot = 0;
        for (int k = 0; k < 36; ++k) dot += pa[i].descriptor[k] * pb[j].descriptor[k];
        distance[i * pb.size() + j] = 1 - dot;
    }
    std::vector<std::pair<Point, Point>> matches;
    for (size_t i = 0; i < pa.size(); ++i) {
        float best = 2, second = 2; size_t at = 0;
        for (size_t j = 0; j < pb.size(); ++j) {
            float d = distance[i * pb.size() + j];
            if (d < best) { second = best; best = d; at = j; }
            else second = std::min(second, d);
        }
        if (best >= .18f || best >= .65f * second) continue;
        bool mutual = true;
        for (size_t k = 0; k < pa.size(); ++k) if (distance[k * pb.size() + at] < best) { mutual = false; break; }
        int dx = pb[at].x - pa[i].x, dy = pb[at].y - pa[i].y;
        if (mutual && dx * dx + dy * dy > 6.25) matches.emplace_back(pa[i], pb[at]);
    }
    auto inliers = [&](size_t anchor, double cs, double sn) {
        int count = 0; const auto &o = matches[anchor];
        for (const auto &m : matches) {
            double x = m.first.x - o.first.x, y = m.first.y - o.first.y;
            double dx = cs * x - sn * y + o.second.x - m.second.x;
            double dy = sn * x + cs * y + o.second.y - m.second.y;
            count += dx * dx + dy * dy < 9;
        }
        return count;
    };
    for (size_t i = 0; i < matches.size(); ++i) {
        if (inliers(i, 1, 0) >= required_matches) return Correspondence::Moving;
        for (size_t j = 0; j < i; ++j) {
            double x = matches[i].first.x - matches[j].first.x, y = matches[i].first.y - matches[j].first.y;
            double u = matches[i].second.x - matches[j].second.x, v = matches[i].second.y - matches[j].second.y;
            double d = x * x + y * y;
            if (d < 25) continue;
            double cs = (x * u + y * v) / d, sn = (x * v - y * u) / d, scale = cs * cs + sn * sn;
            if (scale > .25 && scale < 4 && inliers(i, cs, sn) >= required_matches) return Correspondence::Moving;
        }
    }
    return Correspondence::Distinct;
}
}
