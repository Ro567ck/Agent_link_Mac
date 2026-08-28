#pragma once
// Idle screen: a line-art cat, drawn once into a PSRAM framebuffer and blitted whenever the board
// goes back to standby. Nothing animates — the buffer is built at boot and reused, so showing the
// idle screen costs one DrawBitmap and no CPU after that.
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

    // Framebuffer for St7789Lcd::DrawBitmap, or nullptr if Init() failed.
    const void* Pixels() const { return buf_; }
    uint16_t    Width()  const { return w_; }
    uint16_t    Height() const { return h_; }
    bool        Ready()  const { return buf_ != nullptr; }

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

    uint16_t* buf_ = nullptr;
    uint16_t  w_   = 0;
    uint16_t  h_   = 0;
};
