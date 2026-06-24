/*
 * Copyright (C) 2006 Evgeniy Stepanov <eugeni.stepanov@gmail.com>
 *
 * This file is part of mpv.
 *
 * mpv is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * mpv is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with mpv.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

#include "config.h"

#include <ass/ass.h>
#include <ass/ass_types.h>
#if HAVE_SUBRANDR
#include <subrandr/subrandr.h>
#endif

#include "common/common.h"
#include "packer.h"
#include "img_convert.h"
#include "osd.h"
#include "video/out/bitmap_packer.h"
#include "video/mp_image.h"

#define ASS_ATLAS_INITIAL_SIZE 1024
#define ASS_ATLAS_MAX_DIM 16384
#define ASS_ATLAS_MAX_REFS 4096

struct packed_ass_ref {
    void *bitmap;
    int stride;
    int w, h;
    uint32_t color;
    uint64_t bitmap_hash;
    float blur_x, blur_y;
    int src_x, src_y;
};

struct mp_sub_packer {
    struct sub_bitmap *cached_parts; // only for the array memory
    struct packed_ass_ref *cached_ass_refs;
    int num_cached_ass_refs;
    struct packed_ass_ref *ass_atlas_refs;
    int num_ass_atlas_refs;
    int *ass_atlas_indices;
    struct sub_bitmap_dirty_rect *ass_atlas_dirty;
    int num_ass_atlas_dirty;
    struct mp_image *ass_atlas_img;
    int ass_atlas_w, ass_atlas_h;
    int ass_atlas_used_w, ass_atlas_used_h;
    int ass_atlas_x, ass_atlas_y, ass_atlas_row_h;
    struct sub_bitmap *cached_subrandr_images;
    struct mp_image *cached_img;
    struct sub_bitmaps cached_subs;
    bool cached_subs_valid;
    struct sub_bitmap rgba_imgs[MP_SUB_BB_LIST_MAX];
    struct bitmap_packer *packer;
};

static void fill_padding_1(uint8_t *base, int w, int h, int stride, int padding);

// Free with talloc_free().
struct mp_sub_packer *mp_sub_packer_alloc(void *ta_parent)
{
    struct mp_sub_packer *p = talloc_zero(ta_parent, struct mp_sub_packer);
    p->packer = talloc_zero(p, struct bitmap_packer);
    p->packer->padding = 1; // assume bilinear sampling
    return p;
}

static bool pack(struct mp_sub_packer *p, struct sub_bitmaps *res, int imgfmt)
{
    packer_set_size(p->packer, res->num_parts);

    for (int n = 0; n < res->num_parts; n++)
        p->packer->in[n] = (struct pos){res->parts[n].w, res->parts[n].h};

    if (p->packer->count == 0 || packer_pack(p->packer) < 0)
        return false;

    struct pos bb[2];
    packer_get_bb(p->packer, bb);

    res->packed_w = bb[1].x;
    res->packed_h = bb[1].y;

    if (!p->cached_img || p->cached_img->w < res->packed_w ||
                          p->cached_img->h < res->packed_h ||
                          p->cached_img->imgfmt != imgfmt)
    {
        talloc_free(p->cached_img);
        p->cached_img = mp_image_alloc(imgfmt, p->packer->w, p->packer->h);
        if (!p->cached_img) {
            packer_reset(p->packer);
            return false;
        }
        talloc_steal(p, p->cached_img);
    }

    if (!mp_image_make_writeable(p->cached_img)) {
        packer_reset(p->packer);
        return false;
    }

    res->packed = p->cached_img;

    for (int n = 0; n < res->num_parts; n++) {
        struct sub_bitmap *b = &res->parts[n];
        struct pos pos = p->packer->result[n];

        b->src_x = pos.x;
        b->src_y = pos.y;
    }

    return true;
}

static bool ass_ref_matches(struct packed_ass_ref *ref, struct sub_bitmap *b)
{
    return ref->bitmap == b->bitmap && ref->stride == b->stride &&
           ref->w == b->w && ref->h == b->h &&
           ref->color == b->libass.color &&
           ref->blur_x == b->libass.blur_x &&
           ref->blur_y == b->libass.blur_y;
}

static uint64_t ass_bitmap_hash(struct sub_bitmap *b)
{
    uint64_t h = 1469598103934665603ULL;
    uint8_t *src = b->bitmap;

    for (int y = 0; y < b->h; y++) {
        for (int x = 0; x < b->w; x++) {
            h ^= src[x];
            h *= 1099511628211ULL;
        }
        src += b->stride;
    }

    return h;
}

static bool ass_atlas_ref_matches(struct mp_sub_packer *p, struct packed_ass_ref *ref,
                                  struct sub_bitmap *b, uint64_t hash)
{
    if (ref->w != b->w || ref->h != b->h ||
        ref->bitmap_hash != hash ||
        ref->blur_x != b->libass.blur_x ||
        ref->blur_y != b->libass.blur_y ||
        !p->ass_atlas_img)
        return false;

    uint8_t *src = b->bitmap;
    uint8_t *dst = p->ass_atlas_img->planes[0] +
                   ref->src_y * p->ass_atlas_img->stride[0] + ref->src_x;
    for (int y = 0; y < b->h; y++) {
        if (memcmp(src, dst, b->w) != 0)
            return false;
        src += b->stride;
        dst += p->ass_atlas_img->stride[0];
    }
    return true;
}

static int next_pow2_limit(int v, int limit)
{
    int r = 1;
    while (r < v && r < limit)
        r *= 2;
    return r >= v ? r : 0;
}

static void reset_ass_atlas(struct mp_sub_packer *p)
{
    p->num_ass_atlas_refs = 0;
    p->ass_atlas_w = p->ass_atlas_h = 0;
    p->ass_atlas_used_w = p->ass_atlas_used_h = 0;
    p->ass_atlas_x = p->ass_atlas_y = p->ass_atlas_row_h = 0;
    p->num_ass_atlas_dirty = 0;
    talloc_free(p->ass_atlas_img);
    p->ass_atlas_img = NULL;
}

static bool ensure_ass_atlas_img(struct mp_sub_packer *p, int want_w, int want_h)
{
    if (want_w > ASS_ATLAS_MAX_DIM || want_h > ASS_ATLAS_MAX_DIM)
        return false;

    int new_w = p->ass_atlas_w ? p->ass_atlas_w : ASS_ATLAS_INITIAL_SIZE;
    int new_h = p->ass_atlas_h ? p->ass_atlas_h : ASS_ATLAS_INITIAL_SIZE;
    new_w = next_pow2_limit(MPMAX(new_w, want_w), ASS_ATLAS_MAX_DIM);
    new_h = next_pow2_limit(MPMAX(new_h, want_h), ASS_ATLAS_MAX_DIM);
    if (!new_w || !new_h)
        return false;

    if (p->ass_atlas_img && p->ass_atlas_img->w >= new_w &&
        p->ass_atlas_img->h >= new_h && p->ass_atlas_img->imgfmt == IMGFMT_Y8)
        return mp_image_make_writeable(p->ass_atlas_img);

    struct mp_image *old = p->ass_atlas_img;
    struct mp_image *img = mp_image_alloc(IMGFMT_Y8, new_w, new_h);
    if (!img)
        return false;
    talloc_steal(p, img);

    if (old) {
        int copy_w = MPMIN(old->w, img->w);
        int copy_h = MPMIN(old->h, img->h);
        memcpy_pic(img->planes[0], old->planes[0], copy_w, copy_h,
                   img->stride[0], old->stride[0]);
        talloc_free(old);
    }

    p->ass_atlas_img = img;
    p->ass_atlas_w = img->w;
    p->ass_atlas_h = img->h;
    return mp_image_make_writeable(p->ass_atlas_img);
}

static int find_ass_atlas_ref(struct mp_sub_packer *p, struct sub_bitmap *b,
                              uint64_t hash)
{
    for (int n = 0; n < p->num_ass_atlas_refs; n++) {
        if (ass_atlas_ref_matches(p, &p->ass_atlas_refs[n], b, hash))
            return n;
    }
    return -1;
}

static bool alloc_ass_atlas_rect(struct mp_sub_packer *p, int w, int h,
                                 int *out_x, int *out_y)
{
    int padding = p->packer->padding;
    int rw = w + padding * 2;
    int rh = h + padding * 2;
    if (rw <= 0 || rh <= 0 || rw > ASS_ATLAS_MAX_DIM || rh > ASS_ATLAS_MAX_DIM)
        return false;

    int want_w = p->ass_atlas_w ? p->ass_atlas_w : ASS_ATLAS_INITIAL_SIZE;
    want_w = MPMAX(want_w, rw);
    want_w = next_pow2_limit(want_w, ASS_ATLAS_MAX_DIM);
    if (!want_w)
        return false;

    if (!p->ass_atlas_w) {
        if (!ensure_ass_atlas_img(p, want_w, MPMAX(rh, ASS_ATLAS_INITIAL_SIZE)))
            return false;
    } else if (rw > p->ass_atlas_w) {
        if (!ensure_ass_atlas_img(p, want_w, p->ass_atlas_h))
            return false;
    }

    if (p->ass_atlas_x + rw > p->ass_atlas_w) {
        p->ass_atlas_x = 0;
        p->ass_atlas_y += p->ass_atlas_row_h;
        p->ass_atlas_row_h = 0;
    }

    int want_h = p->ass_atlas_y + rh;
    if (want_h > p->ass_atlas_h &&
        !ensure_ass_atlas_img(p, p->ass_atlas_w, want_h))
        return false;

    *out_x = p->ass_atlas_x + padding;
    *out_y = p->ass_atlas_y + padding;
    p->ass_atlas_x += rw;
    p->ass_atlas_row_h = MPMAX(p->ass_atlas_row_h, rh);
    p->ass_atlas_used_w = MPMAX(p->ass_atlas_used_w, p->ass_atlas_x);
    p->ass_atlas_used_h = MPMAX(p->ass_atlas_used_h, p->ass_atlas_y + rh);
    return true;
}

static void mark_ass_atlas_dirty(struct mp_sub_packer *p, int x, int y,
                                 int w, int h)
{
    int pad = p->packer->padding;
    struct sub_bitmap_dirty_rect rc = {
        .x0 = MPCLAMP(x - pad, 0, p->ass_atlas_w),
        .y0 = MPCLAMP(y - pad, 0, p->ass_atlas_h),
        .x1 = MPCLAMP(x + w + pad, 0, p->ass_atlas_w),
        .y1 = MPCLAMP(y + h + pad, 0, p->ass_atlas_h),
    };
    if (rc.x0 >= rc.x1 || rc.y0 >= rc.y1)
        return;

    // Atlas allocations are row-linear. Merge adjacent rectangles in the same
    // row to avoid turning one subtitle frame into dozens of tiny GPU uploads.
    for (int n = 0; n < p->num_ass_atlas_dirty; n++) {
        struct sub_bitmap_dirty_rect *old = &p->ass_atlas_dirty[n];
        if (old->y0 == rc.y0 && old->y1 == rc.y1 &&
            rc.x0 <= old->x1 && rc.x1 >= old->x0)
        {
            old->x0 = MPMIN(old->x0, rc.x0);
            old->x1 = MPMAX(old->x1, rc.x1);
            return;
        }
    }

    MP_TARRAY_APPEND(p, p->ass_atlas_dirty, p->num_ass_atlas_dirty, rc);
}

static bool cache_ass_bitmap(struct mp_sub_packer *p, struct sub_bitmap *b,
                             struct packed_ass_ref *ref)
{
    uint8_t *base = p->ass_atlas_img->planes[0];
    int stride = p->ass_atlas_img->stride[0];
    void *pdata = base + ref->src_y * stride + ref->src_x;
    memcpy_pic(pdata, b->bitmap, b->w, b->h, stride, b->stride);
    fill_padding_1(pdata, b->w, b->h, stride, p->packer->padding);
    mark_ass_atlas_dirty(p, ref->src_x, ref->src_y, b->w, b->h);
    return true;
}

static bool pack_libass_cached(struct mp_sub_packer *p, struct sub_bitmaps *res,
                               bool *content_changed)
{
    if (res->num_parts > ASS_ATLAS_MAX_REFS ||
        p->num_ass_atlas_refs > ASS_ATLAS_MAX_REFS)
    {
        reset_ass_atlas(p);
        return false;
    }

    *content_changed = false;
    p->num_ass_atlas_dirty = 0;
    MP_TARRAY_GROW(p, p->ass_atlas_indices, res->num_parts);

    for (int n = 0; n < res->num_parts; n++) {
        struct sub_bitmap *b = &res->parts[n];
        uint64_t hash = ass_bitmap_hash(b);
        int idx = find_ass_atlas_ref(p, b, hash);
        if (idx >= 0) {
            p->ass_atlas_indices[n] = idx;
            continue;
        }

        if (p->num_ass_atlas_refs >= ASS_ATLAS_MAX_REFS) {
            reset_ass_atlas(p);
            return false;
        }

        int src_x, src_y;
        if (!alloc_ass_atlas_rect(p, b->w, b->h, &src_x, &src_y)) {
            reset_ass_atlas(p);
            return false;
        }

        MP_TARRAY_GROW(p, p->ass_atlas_refs, p->num_ass_atlas_refs);
        idx = p->num_ass_atlas_refs++;
        struct packed_ass_ref *ref = &p->ass_atlas_refs[idx];
        *ref = (struct packed_ass_ref){
            .bitmap = b->bitmap,
            .stride = b->stride,
            .w = b->w,
            .h = b->h,
            .color = b->libass.color,
            .bitmap_hash = hash,
            .blur_x = b->libass.blur_x,
            .blur_y = b->libass.blur_y,
            .src_x = src_x,
            .src_y = src_y,
        };
        if (!cache_ass_bitmap(p, b, ref)) {
            reset_ass_atlas(p);
            return false;
        }
        p->ass_atlas_indices[n] = idx;
        *content_changed = true;
    }

    if (!p->ass_atlas_img)
        return false;

    res->packed = p->ass_atlas_img;
    res->packed_w = p->ass_atlas_used_w;
    res->packed_h = p->ass_atlas_used_h;
    res->packed_dirty = p->ass_atlas_dirty;
    res->num_packed_dirty = p->num_ass_atlas_dirty;

    uint8_t *base = res->packed->planes[0];
    int stride = res->packed->stride[0];
    for (int n = 0; n < res->num_parts; n++) {
        struct sub_bitmap *b = &res->parts[n];
        int idx = p->ass_atlas_indices[n];
        if (idx < 0 || idx >= p->num_ass_atlas_refs)
            return false;

        struct packed_ass_ref *ref = &p->ass_atlas_refs[idx];
        b->src_x = ref->src_x;
        b->src_y = ref->src_y;
        b->bitmap = base + b->src_y * stride + b->src_x;
        b->stride = stride;
    }

    return true;
}

static void save_ass_source_refs(struct mp_sub_packer *p,
                                 struct sub_bitmaps *res)
{
    MP_TARRAY_GROW(p, p->cached_ass_refs, res->num_parts);
    p->num_cached_ass_refs = res->num_parts;

    for (int n = 0; n < res->num_parts; n++) {
        struct sub_bitmap *b = &res->parts[n];
        p->cached_ass_refs[n] = (struct packed_ass_ref){
            .bitmap = b->bitmap,
            .stride = b->stride,
            .w = b->w,
            .h = b->h,
            .color = b->libass.color,
            .blur_x = b->libass.blur_x,
            .blur_y = b->libass.blur_y,
        };
    }
}

static void save_ass_packed_positions(struct mp_sub_packer *p,
                                      struct sub_bitmaps *res)
{
    for (int n = 0; n < res->num_parts && n < p->num_cached_ass_refs; n++) {
        p->cached_ass_refs[n].src_x = res->parts[n].src_x;
        p->cached_ass_refs[n].src_y = res->parts[n].src_y;
    }
}

static bool reuse_ass_packing(struct mp_sub_packer *p, struct sub_bitmaps *res,
                              int format)
{
    if (format != SUBBITMAP_LIBASS || !p->cached_subs_valid ||
        p->cached_subs.format != SUBBITMAP_LIBASS || !p->cached_subs.packed ||
        p->num_cached_ass_refs != res->num_parts)
        return false;

    for (int n = 0; n < res->num_parts; n++) {
        if (!ass_ref_matches(&p->cached_ass_refs[n], &res->parts[n]))
            return false;
    }

    res->packed = p->cached_subs.packed;
    res->packed_w = p->cached_subs.packed_w;
    res->packed_h = p->cached_subs.packed_h;

    uint8_t *base = res->packed->planes[0];
    int stride = res->packed->stride[0];
    for (int n = 0; n < res->num_parts; n++) {
        struct sub_bitmap *b = &res->parts[n];
        b->src_x = p->cached_ass_refs[n].src_x;
        b->src_y = p->cached_ass_refs[n].src_y;
        b->bitmap = base + b->src_y * stride + b->src_x;
        b->stride = stride;
    }

    return true;
}

static void fill_padding_1(uint8_t *base, int w, int h, int stride, int padding)
{
    for (int row = 0; row < h; ++row) {
        uint8_t *row_ptr = base + row * stride;
        uint8_t left_pixel = row_ptr[0];
        uint8_t right_pixel = row_ptr[w - 1];

        for (int i = 1; i <= padding; ++i)
            row_ptr[-i] = left_pixel;

        for (int i = 0; i < padding; ++i)
            row_ptr[w + i] = right_pixel;
    }

    int row_bytes = (w + 2 * padding);
    uint8_t *top_row = base - padding;
    for (int i = 1; i <= padding; ++i)
        memcpy(base - i * stride - padding, top_row, row_bytes);

    uint8_t *last_row = base + (h - 1) * stride - padding;
    for (int i = 0; i < padding; ++i)
        memcpy(base + (h + i) * stride - padding, last_row, row_bytes);
}

static void fill_padding_4(uint8_t *base, int w, int h, int stride, int padding)
{
    for (int row = 0; row < h; ++row) {
        uint32_t *row_ptr = (uint32_t *)(base + row * stride);
        uint32_t left_pixel = row_ptr[0];
        uint32_t right_pixel = row_ptr[w - 1];

        for (int i = 1; i <= padding; ++i)
            row_ptr[-i] = left_pixel;

        for (int i = 0; i < padding; ++i)
            row_ptr[w + i] = right_pixel;
    }

    int row_bytes = (w + 2 * padding) * 4;
    uint8_t *top_row = base - padding * 4;
    for (int i = 1; i <= padding; ++i)
        memcpy(base - i * stride - padding * 4, top_row, row_bytes);

    uint8_t *last_row = base + (h - 1) * stride - padding * 4;
    for (int i = 0; i < padding; ++i)
        memcpy(base + (h + i) * stride - padding * 4, last_row, row_bytes);
}

static void draw_ass_rgba(unsigned char *src, int src_w, int src_h,
                          int src_stride, unsigned char *dst, size_t dst_stride,
                          int dst_x, int dst_y, uint32_t color)
{
    const unsigned int r = (color >> 24) & 0xff;
    const unsigned int g = (color >> 16) & 0xff;
    const unsigned int b = (color >>  8) & 0xff;
    const unsigned int a = 0xff - (color & 0xff);

    dst += dst_y * dst_stride + dst_x * 4;

    for (int y = 0; y < src_h; y++, dst += dst_stride, src += src_stride) {
        uint32_t *dstrow = (uint32_t *) dst;
        for (int x = 0; x < src_w; x++) {
            const unsigned int v = src[x];
            int rr = (r * a * v);
            int gg = (g * a * v);
            int bb = (b * a * v);
            int aa =      a * v;
            uint32_t dstpix = dstrow[x];
            unsigned int dstb =  dstpix        & 0xFF;
            unsigned int dstg = (dstpix >>  8) & 0xFF;
            unsigned int dstr = (dstpix >> 16) & 0xFF;
            unsigned int dsta = (dstpix >> 24) & 0xFF;
            dstb = (bb       + dstb * (255 * 255 - aa)) / (255 * 255);
            dstg = (gg       + dstg * (255 * 255 - aa)) / (255 * 255);
            dstr = (rr       + dstr * (255 * 255 - aa)) / (255 * 255);
            dsta = (aa * 255 + dsta * (255 * 255 - aa)) / (255 * 255);
            dstrow[x] = dstb | (dstg << 8) | (dstr << 16) | (dsta << 24);
        }
    }
}

static bool pack_libass(struct mp_sub_packer *p, struct sub_bitmaps *res)
{
    if (!pack(p, res, IMGFMT_Y8))
        return false;

    int padding = p->packer->padding;
    uint8_t *base = res->packed->planes[0];
    int stride = res->packed->stride[0];

    for (int n = 0; n < res->num_parts; n++) {
        struct sub_bitmap *b = &res->parts[n];
        void *pdata = base + b->src_y * stride + b->src_x;
        memcpy_pic(pdata, b->bitmap, b->w, b->h, stride, b->stride);
        fill_padding_1(pdata, b->w, b->h, stride, padding);

        b->bitmap = pdata;
        b->stride = stride;
    }

    return true;
}

static bool pack_rgba(struct mp_sub_packer *p, struct sub_bitmaps *res)
{
    struct mp_rect bb_list[MP_SUB_BB_LIST_MAX];
    int num_bb = mp_get_sub_bb_list(res, bb_list, MP_SUB_BB_LIST_MAX);

    struct sub_bitmaps imgs = {
        .change_id = res->change_id,
        .format = SUBBITMAP_BGRA,
        .parts = p->rgba_imgs,
        .num_parts = num_bb,
    };

    for (int n = 0; n < imgs.num_parts; n++) {
        imgs.parts[n].w = bb_list[n].x1 - bb_list[n].x0;
        imgs.parts[n].h = bb_list[n].y1 - bb_list[n].y0;
    }

    if (!pack(p, &imgs, IMGFMT_BGRA))
        return false;

    int padding = p->packer->padding;
    uint8_t *base = imgs.packed->planes[0];
    int stride = imgs.packed->stride[0];

    for (int n = 0; n < num_bb; n++) {
        struct mp_rect bb = bb_list[n];
        struct sub_bitmap *b = &imgs.parts[n];

        b->x = bb.x0;
        b->y = bb.y0;
        b->w = b->dw = mp_rect_w(bb);
        b->h = b->dh = mp_rect_h(bb);
        b->stride = stride;
        b->bitmap = base + b->stride * b->src_y + b->src_x * 4;
        memset_pic(b->bitmap, 0, b->w * 4, b->h, b->stride);

        for (int i = 0; i < res->num_parts; i++) {
            struct sub_bitmap *s = &res->parts[i];

            // Assume mp_get_sub_bb_list() never splits sub bitmaps
            // So we don't clip/adjust the size of the sub bitmap
            if (s->x >= b->x + b->w || s->x + s->w <= b->x ||
                s->y >= b->y + b->h || s->y + s->h <= b->y)
                continue;

            draw_ass_rgba(s->bitmap, s->w, s->h, s->stride,
                          b->bitmap, b->stride,
                          s->x - b->x, s->y - b->y,
                          s->libass.color);
        }
        fill_padding_4(b->bitmap, b->w, b->h, b->stride, padding);
    }

    *res = imgs;
    return true;
}

// Pack the contents of image_lists[0] to image_lists[num_image_lists-1] into
// a single image, and make *out point to it. *out is completely overwritten.
// If libass reports only position changes, keep the old atlas and update only
// destination coordinates. A full repack is needed only for content changes.
// preferred_osd_format can be set to a desired sub_bitmap_format. Currently,
// only SUBBITMAP_LIBASS is supported for position-only reuse.
void mp_sub_packer_pack_ass(struct mp_sub_packer *p, ASS_Image **image_lists,
                        int num_image_lists, int image_lists_changed, bool video_color_space,
                        int preferred_osd_format, struct sub_bitmaps *out)
{
    int format = preferred_osd_format == SUBBITMAP_BGRA ? SUBBITMAP_BGRA
                                                        : SUBBITMAP_LIBASS;

    if (p->cached_subs_valid && image_lists_changed == 0 &&
        p->cached_subs.format == format)
    {
        *out = p->cached_subs;
        return;
    }

    *out = (struct sub_bitmaps){.change_id = 1};

    struct sub_bitmaps res = {
        .change_id = 1,
        .format = SUBBITMAP_LIBASS,
        .parts = p->cached_parts,
        .video_color_space = video_color_space,
    };

    for (int n = 0; n < num_image_lists; n++) {
        for (struct ass_image *img = image_lists[n]; img; img = img->next) {
            if (img->w == 0 || img->h == 0)
                continue;
            MP_TARRAY_GROW(p, p->cached_parts, res.num_parts);
            res.parts = p->cached_parts;
            struct sub_bitmap *b = &res.parts[res.num_parts];
            b->bitmap = img->bitmap;
            b->stride = img->stride;
            b->libass.color = img->color;
            b->libass.blur_x = img->blur_x;
            b->libass.blur_y = img->blur_y;
            b->dw = b->w = img->w;
            b->dh = b->h = img->h;
            b->x = img->dst_x;
            b->y = img->dst_y;
            res.num_parts++;
        }
    }

    if (image_lists_changed == 1 && reuse_ass_packing(p, &res, format)) {
        res.change_id = 0;
        *out = res;
        p->cached_subs = res;
        p->cached_subs.change_id = 0;
        p->cached_subs_valid = true;
        return;
    }

    p->cached_subs_valid = false;
    if (format == SUBBITMAP_LIBASS) {
        bool content_changed = false;
        if (pack_libass_cached(p, &res, &content_changed)) {
            res.change_id = content_changed ? 1 : 0;
            *out = res;
            p->cached_subs = res;
            p->cached_subs.change_id = 0;
            p->cached_subs.packed_dirty = NULL;
            p->cached_subs.num_packed_dirty = 0;
            p->cached_subs_valid = true;
            return;
        }
        save_ass_source_refs(p, &res);
    } else {
        p->num_cached_ass_refs = 0;
    }

    bool r = false;
    if (format == SUBBITMAP_BGRA) {
        r = pack_rgba(p, &res);
    } else {
        r = pack_libass(p, &res);
    }

    if (!r)
        return;

    if (format == SUBBITMAP_LIBASS)
        save_ass_packed_positions(p, &res);

    *out = res;
    p->cached_subs = res;
    p->cached_subs.change_id = 0;
    p->cached_subs_valid = true;
}

#if HAVE_SUBRANDR
// Pack the images in `res` into a BGRA8 atlas and populate `res->parts`
// with instances of the images as described by `pass`.
static bool pack_subrandr(struct mp_sub_packer *p, struct sub_bitmaps *res,
                          struct sbr_instanced_raster_pass *pass)
{
    if (!pack(p, res, IMGFMT_BGRA))
        return false;

    int padding = p->packer->padding;
    sbr_bgra8 *base = (sbr_bgra8 *)res->packed->planes[0];
    int byte_stride = res->packed->stride[0];
    int pixel_stride = byte_stride / 4;
    struct sbr_output_instance *instances = sbr_instanced_raster_pass_get_instances(pass);

    for (int n = 0; n < res->num_parts; n++) {
        struct sub_bitmap *b = &res->parts[n];
        sbr_output_image_rasterize_into(b->subrandr.image, pass, b->src_x, b->src_y,
                                        base, res->packed_w, res->packed_h, pixel_stride);

        void *pdata = base + b->src_y * pixel_stride + b->src_x;
        fill_padding_4(pdata, b->w, b->h, byte_stride, padding);
        b->bitmap = pdata;
    }

    res->parts = NULL;
    res->num_parts = 0;
    res->format = SUBBITMAP_BGRA;
    for (struct sbr_output_instance *instance = instances; instance; instance = instance->next) {
        if (!instance->base->user_data)
            continue;

        MP_TARRAY_GROW(p, p->cached_parts, res->num_parts);
        res->parts = p->cached_parts;
        struct sub_bitmap *inst_b = &res->parts[res->num_parts];
        struct sub_bitmap *image_b = &p->cached_subrandr_images[(size_t)instance->base->user_data - 1];

        *inst_b = (struct sub_bitmap){
            .x = instance->dst_x,      .y = instance->dst_y,
            .dw = instance->dst_width, .dh = instance->dst_height,
            .w = instance->src_width,  .h = instance->src_height,
            .src_x = image_b->src_x + instance->src_off_x,
            .src_y = image_b->src_y + instance->src_off_y,
            .bitmap = (sbr_bgra8 *)image_b->bitmap
                      + pixel_stride * instance->src_off_y + instance->src_off_x,
            .stride = byte_stride,
        };
        res->num_parts++;
    }

    return true;
}

// Pack the images in `pass` into a single image, make `out` point to it,
// and populate `out->parts` to correctly describe all instances in `pass`.
void mp_sub_packer_pack_sbr(struct mp_sub_packer *p, sbr_instanced_raster_pass *pass,
                            struct sub_bitmaps *out)
{
    *out = (struct sub_bitmaps){.change_id = 1};
    p->cached_subs_valid = false;

    struct sub_bitmaps res = {
        .change_id = (unsigned)p->cached_subs.change_id + 1,
        .format = SUBBITMAP_SUBRANDR,
        .parts = p->cached_parts,
        .video_color_space = false,
    };

    struct sbr_output_instance *instances = sbr_instanced_raster_pass_get_instances(pass);
    for (struct sbr_output_instance *instance = instances; instance; instance = instance->next) {
        // If `user_data` is non-null then this base image was already appended.
        if (instance->base->user_data)
            continue;

        MP_TARRAY_GROW(p, p->cached_subrandr_images, res.num_parts);
        res.parts = p->cached_subrandr_images;
        struct sub_bitmap *b = &res.parts[res.num_parts];
        b->subrandr.image = instance->base;
        b->w = instance->base->width, b->h = instance->base->height;
        // Store the index of the `sub_bitmap` for this output image into `user_data`.
        // This index is incremented by one to differentiate index `0` from an absent index.
        // Will be read in `pack_subrandr` after packing to determine where each image
        // was actually packed to in the atlas.
        instance->base->user_data = (void *)(uintptr_t)(res.num_parts + 1);
        res.num_parts++;
    }

    if (!pack_subrandr(p, &res, pass))
        return;

    *out = res;
    p->cached_subs = res;
    p->cached_subs_valid = true;
}

const struct sub_bitmaps *mp_sub_packer_get_cached(struct mp_sub_packer *p) {
    if (p->cached_subs_valid)
        return &p->cached_subs;
    return NULL;
}
#endif
