#include "pmu_axp2101.h"

#include <stdint.h>

#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "pmu_axp2101";

// Minimal AXP2101 register set (taken from Waveshare XPowersLib headers)
#define AXP2101_REG_DC_ONOFF_DVM_CTRL 0x80
#define AXP2101_REG_DC_VOL0_CTRL      0x82
#define AXP2101_REG_LDO_ONOFF_CTRL0   0x90
#define AXP2101_REG_LDO_VOL0_CTRL     0x92

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

i2c_master_bus_handle_t rs3_pmu_get_i2c_bus(void)
{
    return s_bus;
}

static esp_err_t pmu_i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = RS3_PMU_I2C_PORT,
        .sda_io_num = RS3_PMU_I2C_SDA,
        .scl_io_num = RS3_PMU_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_bus), TAG, "i2c_new_master_bus failed");

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RS3_PMU_I2C_ADDR,
        .scl_speed_hz = RS3_PMU_I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_config, &s_dev), TAG, "i2c add dev failed");
    return ESP_OK;
}

static esp_err_t pmu_reg_read_u8(uint8_t reg, uint8_t *out)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, 1, 1000);
}

static esp_err_t pmu_reg_write_u8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 1000);
}

static esp_err_t pmu_enable_lcd_power_rails(void)
{
    // Mirrors the relevant part of Waveshare's pmu_init():
    // - set DC1=3300mV and enable it
    // - set ALDO1=3300mV and enable it
    ESP_RETURN_ON_ERROR(pmu_reg_write_u8(AXP2101_REG_DC_VOL0_CTRL, 0x12), TAG, "set DC1 voltage failed"); // (3300-1500)/100=0x12

    uint8_t v = 0;
    ESP_RETURN_ON_ERROR(pmu_reg_read_u8(AXP2101_REG_DC_ONOFF_DVM_CTRL, &v), TAG, "read DC ctrl failed");
    v |= (1 << 0); // enable DC1
    ESP_RETURN_ON_ERROR(pmu_reg_write_u8(AXP2101_REG_DC_ONOFF_DVM_CTRL, v), TAG, "enable DC1 failed");

    ESP_RETURN_ON_ERROR(pmu_reg_read_u8(AXP2101_REG_LDO_VOL0_CTRL, &v), TAG, "read LDO vol failed");
    v = (v & 0xE0) | 0x1C; // ALDO1=3300mV => (3300-500)/100=0x1C
    ESP_RETURN_ON_ERROR(pmu_reg_write_u8(AXP2101_REG_LDO_VOL0_CTRL, v), TAG, "set ALDO1 voltage failed");

    ESP_RETURN_ON_ERROR(pmu_reg_read_u8(AXP2101_REG_LDO_ONOFF_CTRL0, &v), TAG, "read LDO onoff failed");
    v |= (1 << 0); // enable ALDO1
    ESP_RETURN_ON_ERROR(pmu_reg_write_u8(AXP2101_REG_LDO_ONOFF_CTRL0, v), TAG, "enable ALDO1 failed");

    ESP_LOGI(TAG, "Enabled DC1 + ALDO1 (3.3V)");
    return ESP_OK;
}

esp_err_t rs3_pmu_init_and_enable_lcd_power(void)
{
    esp_err_t ret = pmu_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C init failed (%s)", esp_err_to_name(ret));
        return ret;
    }
    return pmu_enable_lcd_power_rails();
}


