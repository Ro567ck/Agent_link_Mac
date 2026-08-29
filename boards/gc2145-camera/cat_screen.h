#pragma once
// Idle screen: a line-art cat, drawn once into a PSRAM framebuffer and blitted whenever the board
// goes back to standby. The framebuffer is built at boot and reused, so showing the idle screen costs
// one DrawBitmap and no CPU after that.
//
// The whiskers are the one exception: they are kept OUT of that framebuffer and rendered on demand into
// a narrow band (RenderBand), so the board can wag them during a study phase by blitting ~40 rows per
// step instead of regenerating and pushing all 240x240. Everything the band overlaps — head outline,
// cheeks, nose — is copied back from the framebuffer each time, so only the whiskers ever move.
//
// Pixels are native-endian uint16 RGB565, matching what the camera used to hand the ST7789 directly
// (that path ran with CAMERA_RGB565_BYTE_SWAP=0). If red/blue look swapped on your panel, flip
// CAT_SCREEN_BYTE_SWAP in config.h rather than editing the drawing code.

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

class CatScreen {
public:
    CatScreen() = default;
    ~CatScreen();
    CatScreen(const CatScreen&) = delete;
    CatScreen& operator=(const CatScreen&) = delete;

    // Allocate the framebuffer (w*h*2 bytes, PSRAM) and draw the cat into it.
    esp_err_t Init(uint16_t w, uint16_t h);

    // Framebuffer for St7789Lcd::DrawBitmap, or nullptr if Init() failed. Contains the cat WITHOUT
    // whiskers — blit RenderBand(0) over it to get the cat at rest.
    const void* Pixels() const { return buf_; }
    uint16_t    Width()  const { return w_; }
    uint16_t    Height() const { return h_; }
    bool        Ready()  const { return buf_ != nullptr; }

    // ── Whisker band ──
    // False when the band buffer could not be allocated; the whiskers are then baked into the
    // framebuffer at rest, so the cat still looks right and the caller simply skips animating.
    bool        Animated()  const { return band_ != nullptr; }
    uint16_t    BandTop()   const { return band_top_; }
    uint16_t    BandHeight()const { return band_h_; }
    // The band with the whisker tips displaced by wag_px (0 = rest). Returns a buffer owned by
    // CatScreen and overwritten by the next call, so blit it before calling again. Not reentrant:
    // one caller only (on this board, the capture task).
    const void* RenderBand(int wag_px);

private:
    void Clear(uint16_t color);
    void Px(int x, int y, uint16_t color);
    void HLine(int x0, int x1, int y, uint16_t color);
    void FillRect(int x, int y, int w, int h, uint16_t color);
    void FillCircle(int cx, int cy, int r, uint16_t color);
    void FillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color);
    void Stroke(int x0, int y0, int x1, int y1, int thickness, uint16_t color);
    void Ellipse(int cx, int cy, int rx, int ry, uint16_t color);
    void DrawCat();

    struct Whisker { int x0, y0, x1, y1, thick; };   // root (x0,y0) -> tip (x1,y1)
    // Single source of the whisker layout: the band is sized from it and every frame is drawn from it,
    // so the two cannot drift apart and clip a stroke. Returns how many entries were filled.
    int  WhiskerGeometry(Whisker out[4]) const;
    void DrawWhiskers(int wag_px);

    uint16_t* buf_      = nullptr;
    uint16_t* band_     = nullptr;   // w_ * band_h_ scratch for the animated rows
    uint16_t  band_top_ = 0;
    uint16_t  band_h_   = 0;
    // Every primitive writes through here, so the same drawing code fills either the whole framebuffer
    // or just the band (which starts at target_y0_ in cat coordinates) with no duplicated geometry.
    uint16_t* target_    = nullptr;
    int       target_y0_ = 0;
    int       target_h_  = 0;
    uint16_t  w_   = 0;
    uint16_t  h_   = 0;
};
