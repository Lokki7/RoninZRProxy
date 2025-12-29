#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initialize AXP2101 over I2C and enable the board power rails required for LCD.
 *
 * Minimal implementation based on Waveshare's XPowersLib example:
 * https://github.com/waveshareteam/ESP32-S3-Touch-LCD-1.83/tree/main/examples/ESP-IDF/01_AXP2101
 */
esp_err_t rs3_pmu_init_and_enable_lcd_power(void);

/**
 * @brief Get the shared I2C master bus handle used on this board (PMU + touch live on it).
 *
 * @return i2c_master_bus_handle_t or NULL if not initialized yet.
 */
i2c_master_bus_handle_t rs3_pmu_get_i2c_bus(void);


