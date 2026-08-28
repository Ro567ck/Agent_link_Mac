#include "cat_screen.h"

#include "config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include <cstdlib>

#define TAG "CatScreen"

namespace {
// RGB565, native endian. Swap at write time if the panel needs it (see CAT_SCREEN_BYTE_SWAP).
constexpr uint16_t Rgb(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
constexpr uint16_t kBg      = Rgb(250, 246, 235);   // warm off-white
constexpr uint16_t kInk     = Rgb(60,  55,  70);    // soft charcoal for the outline
constexpr uint16_t kBlush   = Rgb(250, 190, 190);   // cheeks / inner ear
constexpr uint16_t kNoseCol = Rgb(235, 140, 150);
}  // namespace

CatScreen::~CatScreen() {
    if (buf_) heap_caps_free(buf_);
}

esp_err_t CatScreen::Init(uint16_t w, uint16_t h) {
    if (w == 0 || h == 0) return ESP_ERR_INVALID_ARG;
    const size_t bytes = static_cast<size_t>(w) * h * 2u;
    // PSRAM: 115KB at 240x240 would be a big chunk of internal RAM. DrawBitmap copies through its own
    // internal stripe buffer, so a PSRAM source is fine.
    buf_ = static_cast<uint16_t*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM));
    if (!buf_) {
        ESP_LOGE(TAG, "framebuffer alloc failed (%uB PSRAM)", static_cast<unsigned>(bytes));
        return ESP_ERR_NO_MEM;
    }
    w_ = w;
    h_ = h;
    DrawCat();
    ESP_LOGI(TAG, "idle screen ready (%ux%u, %uB PSRAM)", w, h, static_cast<unsigned>(bytes));
    return ESP_OK;
}

// ── primitives ──

void CatScreen::Px(int x, int y, uint16_t color) {
    if (x < 0 || y < 0 || x >= w_ || y >= h_) return;
#if CAT_SCREEN_BYTE_SWAP
    color = static_cast<uint16_t>(__builtin_bswap16(color));
#endif
    buf_[static_cast<size_t>(y) * w_ + x] = color;
}

void CatScreen::HLine(int x0, int x1, int y, uint16_t color) {
    if (x0 > x1) { const int t = x0; x0 = x1; x1 = t; }
    for (int x = x0; x <= x1; ++x) Px(x, y, color);
}

void CatScreen::Clear(uint16_t color) {
    for (int y = 0; y < h_; ++y) HLine(0, w_ - 1, y, color);
}

void CatScreen::FillRect(int x, int y, int w, int h, uint16_t color) {
    for (int r = 0; r < h; ++r) HLine(x, x + w - 1, y + r, color);
}

void CatScreen::FillCircle(int cx, int cy, int r, uint16_t color) {
    for (int dy = -r; dy <= r; ++dy) {
        const int dx = static_cast<int>(__builtin_sqrtf(static_cast<float>(r * r - dy * dy)));
        HLine(cx - dx, cx + dx, cy + dy, color);
    }
}

// Filled ellipse — the head and body are wider than tall, which reads more like a cat than a circle.
void CatScreen::Ellipse(int cx, int cy, int rx, int ry, uint16_t color) {
    if (rx <= 0 || ry <= 0) return;
    for (int dy = -ry; dy <= ry; ++dy) {
        const float t = 1.0f - (static_cast<float>(dy) * dy) / (static_cast<float>(ry) * ry);
        if (t <= 0.0f) continue;
        const int dx = static_cast<int>(rx * __builtin_sqrtf(t));
        HLine(cx - dx, cx + dx, cy + dy, color);
    }
}

// Bresenham with a round brush, so strokes have caps and read as hand-drawn lines.
void CatScreen::Stroke(int x0, int y0, int x1, int y1, int thickness, uint16_t color) {
    const int r = thickness / 2;
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        if (r <= 0) Px(x0, y0, color); else FillCircle(x0, y0, r, color);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Flat-bottomed triangle fill, used for the ears.
void CatScreen::FillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color) {
    const int ymin = (y0 < y1 ? (y0 < y2 ? y0 : y2) : (y1 < y2 ? y1 : y2));
    const int ymax = (y0 > y1 ? (y0 > y2 ? y0 : y2) : (y1 > y2 ? y1 : y2));
    for (int y = ymin; y <= ymax; ++y) {
        int lo = 1 << 30, hi = -(1 << 30);
        // Clip each edge against this scanline and keep the span between the crossings.
        const int xs[3] = {x0, x1, x2}, ys[3] = {y0, y1, y2};
        for (int e = 0; e < 3; ++e) {
            const int ax = xs[e], ay = ys[e], bx = xs[(e + 1) % 3], by = ys[(e + 1) % 3];
            if (ay == by) continue;
            if (y < (ay < by ? ay : by) || y > (ay > by ? ay : by)) continue;
            const int x = ax + (bx - ax) * (y - ay) / (by - ay);
            if (x < lo) lo = x;
            if (x > hi) hi = x;
        }
        if (hi >= lo) HLine(lo, hi, y, color);
    }
}

// ── the cat ──
// Sitting cat, line-art. Drawing order matters: each shape is filled in ink and then re-filled inset in
// the background colour, so a later shape's interior erases the earlier outlines it overlaps. That is
// what turns a pile of ellipses into a single continuous outline. Proportions scale off the short side.
void CatScreen::DrawCat() {
    Clear(kBg);

    const int cx = w_ / 2;
    const int s  = (w_ < h_ ? w_ : h_);
    const int lw = s / 48 + 2;                   // outline weight (~7px at 240)

    const int head_cy = h_ * 41 / 100;
    const int head_rx = s * 26 / 100;
    const int head_ry = s * 23 / 100;
    const int head_bottom = head_cy + head_ry;

    // Tail first — everything else draws over where it meets the body.
    const int tail_x = cx + s * 27 / 100;
    const int tail_y = h_ - s * 12 / 100;
    Stroke(tail_x, tail_y, tail_x + s * 13 / 100, tail_y - s * 13 / 100, lw, kInk);
    Stroke(tail_x + s * 13 / 100, tail_y - s * 13 / 100, tail_x + s * 16 / 100, tail_y + s * 2 / 100, lw, kInk);

    // Body: sits just under the head and runs off the bottom edge, so the cat is seated.
    const int body_cy = head_bottom + s * 26 / 100;
    const int body_rx = s * 30 / 100;
    const int body_ry = s * 28 / 100;
    Ellipse(cx, body_cy, body_rx, body_ry, kInk);
    Ellipse(cx, body_cy, body_rx - lw, body_ry - lw, kBg);

    // Ears: solid ink triangles. Their lower halves get erased by the head interior below, which is
    // exactly the classic look — ear outlines merging into the head.
    const int ear_out = head_rx * 88 / 100;
    const int ear_in  = head_rx * 26 / 100;
    const int ear_ty  = head_cy - head_ry - s * 10 / 100;
    for (int side = -1; side <= 1; side += 2) {
        FillTriangle(cx + side * ear_out, head_cy - head_ry * 55 / 100,
                     cx + side * (ear_out - s * 5 / 100), ear_ty,
                     cx + side * ear_in,  head_cy - head_ry * 92 / 100, kInk);
    }

    // Head over the top of the body and the ear bases.
    Ellipse(cx, head_cy, head_rx, head_ry, kInk);
    Ellipse(cx, head_cy, head_rx - lw, head_ry - lw, kBg);

    // Inner ear, drawn after the head so it is not erased.
    for (int side = -1; side <= 1; side += 2) {
        FillTriangle(cx + side * (ear_out - lw), head_cy - head_ry * 72 / 100,
                     cx + side * (ear_out - s * 5 / 100), ear_ty + lw * 2,
                     cx + side * (ear_in + lw * 2), head_cy - head_ry * 88 / 100, kBlush);
    }

    // Whiskers: two per side, starting clear of the head outline so they never cross it.
    const int wh_y = head_cy + s * 5 / 100;
    for (int side = -1; side <= 1; side += 2) {
        const int x0 = cx + side * (head_rx + lw);
        // Straddle the nose line rather than the eyes, or they read as coming off the eyes.
        for (int i = 0; i < 2; ++i) {
            const int dy = (i == 0) ? -s * 1 / 100 : s * 6 / 100;
            Stroke(x0, wh_y + dy, x0 + side * s * 13 / 100, wh_y + dy - s * 3 / 100, lw / 2 + 1, kInk);
        }
    }

    // Eyes: solid dots read far better than arcs at this size.
    const int eye_dx = head_rx * 42 / 100;
    const int eye_y  = head_cy - s * 3 / 100;
    for (int side = -1; side <= 1; side += 2)
        FillCircle(cx + side * eye_dx, eye_y, s * 4 / 100, kInk);

    // Cheeks, tucked below and outside the eyes.
    for (int side = -1; side <= 1; side += 2)
        FillCircle(cx + side * head_rx * 62 / 100, eye_y + s * 7 / 100, s * 4 / 100, kBlush);

    // Nose + the classic cat mouth.
    const int nose_y = eye_y + s * 8 / 100;
    FillCircle(cx, nose_y, s * 3 / 100, kNoseCol);
    for (int side = -1; side <= 1; side += 2)
        Stroke(cx, nose_y + s * 2 / 100, cx + side * s * 5 / 100, nose_y + s * 6 / 100, lw / 2 + 1, kInk);
}
