#pragma once

// Board: Waveshare ESP32-S3-Touch-LCD-1.83 (ST7789P 240x284, AXP2101 PMU)
// Pin reference: Waveshare repo (Arduino pin_config.h)
// https://github.com/waveshareteam/ESP32-S3-Touch-LCD-1.83

#include "driver/i2c_master.h"
#include "driver/spi_master.h"

// ---- LCD (ST7789P over SPI) ----
#define RS3_LCD_HOST           SPI2_HOST
#define RS3_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define RS3_LCD_H_RES          240
#define RS3_LCD_V_RES          284

#define RS3_LCD_PIN_MOSI 7
#define RS3_LCD_PIN_SCLK 6
#define RS3_LCD_PIN_CS   5
#define RS3_LCD_PIN_DC   4
#define RS3_LCD_PIN_RST  38
#define RS3_LCD_PIN_BKLT 40

// ---- PMU (AXP2101 over I2C) ----
#define RS3_PMU_I2C_PORT    I2C_NUM_0
#define RS3_PMU_I2C_SDA     15
#define RS3_PMU_I2C_SCL     14
#define RS3_PMU_I2C_FREQ_HZ 100000
#define RS3_PMU_I2C_ADDR    0x34

// ---- Touch (CST816x over I2C) ----
// Same I2C bus as PMU (SDA=15, SCL=14). Pins from Waveshare Arduino pin_config.h.
#define RS3_TOUCH_I2C_ADDR  0x15
#define RS3_TOUCH_PIN_RST   39
#define RS3_TOUCH_PIN_INT   13


