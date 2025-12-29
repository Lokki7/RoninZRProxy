#include "touch_cst816.h"

#include <string.h>

#include "board_config.h"
#include "pmu_axp2101.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "touch_cst816";

// CST816 register map (subset)
#define CST8XX_REG_STATUS        0x00
#define CST8XX_REG_XPOS_HIGH     0x03
#define CST8XX_REG_XPOS_LOW      0x04
#define CST8XX_REG_YPOS_HIGH     0x05
#define CST8XX_REG_YPOS_LOW      0x06
#define CST8XX_REG_DIS_AUTOSLEEP 0xFE

static i2c_master_dev_handle_t s_touch = NULL;
static bool s_inited = false;

static esp_err_t touch_write_u8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_touch, buf, sizeof(buf), 1000);
}

static esp_err_t touch_read(uint8_t reg, uint8_t *out, size_t out_len)
{
    return i2c_master_transmit_receive(s_touch, &reg, 1, out, out_len, 1000);
}

esp_err_t rs3_touch_init(void)
{
    if (s_inited) return ESP_OK;

    i2c_master_bus_handle_t bus = rs3_pmu_get_i2c_bus();
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_STATE, TAG, "I2C bus not initialized (PMU init must run first)");

    // RST pin
    gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << RS3_TOUCH_PIN_RST,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rst_cfg), TAG, "gpio rst cfg failed");

    // INT pin (active low)
    gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << RS3_TOUCH_PIN_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_cfg), TAG, "gpio int cfg failed");

    // Hardware reset (per vendor example)
    gpio_set_level(RS3_TOUCH_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(15));
    gpio_set_level(RS3_TOUCH_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(80));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RS3_TOUCH_I2C_ADDR,
        .scl_speed_hz = RS3_PMU_I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_config, &s_touch), TAG, "i2c add touch dev failed");

    // Disable autosleep (helps avoid "dead" touch after some idle time)
    (void)touch_write_u8(CST8XX_REG_DIS_AUTOSLEEP, 0x01);

    s_inited = true;
    ESP_LOGI(TAG, "Touch initialized (addr=0x%02X)", RS3_TOUCH_I2C_ADDR);
    return ESP_OK;
}

bool rs3_touch_get_point(int *out_x, int *out_y)
{
    if (!s_inited || !s_touch || !out_x || !out_y) return false;

    // Optional quick check: INT low means a touch event is pending.
    // If it reads high, we still might be missing the edge; keep it permissive.
    (void)gpio_get_level(RS3_TOUCH_PIN_INT);

    uint8_t buf[13];
    if (touch_read(CST8XX_REG_STATUS, buf, sizeof(buf)) != ESP_OK) return false;

    uint8_t p = buf[2];
    if (p == 0x00 || p == 0xFF) return false;
    uint8_t points = (uint8_t)(p & 0x0F);
    if (points == 0) return false;

    int x = ((buf[CST8XX_REG_XPOS_HIGH] & 0x0F) << 8) | buf[CST8XX_REG_XPOS_LOW];
    int y = ((buf[CST8XX_REG_YPOS_HIGH] & 0x0F) << 8) | buf[CST8XX_REG_YPOS_LOW];

    *out_x = x;
    *out_y = y;
    return true;
}


