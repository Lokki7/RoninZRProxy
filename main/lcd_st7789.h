#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    int w;
    int h;
} rs3_lcd_info_t;

/**
 * @brief Initialize SPI bus + ST7789 panel and turn on backlight.
 */
esp_err_t rs3_lcd_init(void);

/**
 * @brief Query LCD width/height.
 */
rs3_lcd_info_t rs3_lcd_get_info(void);

/**
 * @brief Push a full-screen RGB565 framebuffer to the display.
 */
esp_err_t rs3_lcd_draw_full(const uint16_t *fb);


