#pragma once

#include <stdbool.h>

#include "esp_err.h"

/**
 * @brief Initialize CST816 touch controller (I2C + RST/INT pins).
 *
 * Uses the shared I2C bus created by `rs3_pmu_init_and_enable_lcd_power()`.
 */
esp_err_t rs3_touch_init(void);

/**
 * @brief Read one touch point (if any).
 *
 * @param out_x X coordinate (0..RS3_LCD_H_RES-1)
 * @param out_y Y coordinate (0..RS3_LCD_V_RES-1)
 * @return true if touch is currently detected and coordinates are valid
 */
bool rs3_touch_get_point(int *out_x, int *out_y);


