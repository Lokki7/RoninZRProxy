#pragma once

#include <stdint.h>

/**
 * @brief Draw ASCII text with a tiny 5x7 font into an RGB565 framebuffer.
 *
 * @param fb RGB565 framebuffer (row-major), size fb_w*fb_h
 * @param fb_w framebuffer width in pixels
 * @param fb_h framebuffer height in pixels
 * @param x left position (px)
 * @param y top position (px)
 * @param s null-terminated string
 * @param fg RGB565 color for glyph pixels
 * @param bg RGB565 color for background pixels
 * @param scale integer scale factor (>=1)
 */
void rs3_draw_text_5x7(uint16_t *fb, int fb_w, int fb_h, int x, int y, const char *s, uint16_t fg, uint16_t bg, int scale);


