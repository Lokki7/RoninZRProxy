#include "lcd_st7789.h"

#include "board_config.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"

static const char *TAG = "lcd_st7789";

static esp_lcd_panel_handle_t s_panel = NULL;

static esp_err_t backlight_on(void)
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << RS3_LCD_PIN_BKLT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&bk_gpio_config), TAG, "bk gpio_config failed");
    gpio_set_level(RS3_LCD_PIN_BKLT, 1);
    return ESP_OK;
}

esp_err_t rs3_lcd_init(void)
{
    if (s_panel) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(backlight_on(), TAG, "backlight_on failed");

    spi_bus_config_t buscfg = {
        .sclk_io_num = RS3_LCD_PIN_SCLK,
        .mosi_io_num = RS3_LCD_PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = RS3_LCD_H_RES * 40 * 2,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(RS3_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi_bus_initialize failed");

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = RS3_LCD_PIN_DC,
        .cs_gpio_num = RS3_LCD_PIN_CS,
        .pclk_hz = RS3_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)RS3_LCD_HOST, &io_config, &io_handle), TAG, "new_panel_io_spi failed");

    esp_lcd_panel_dev_config_t panel_config = {
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .reset_gpio_num = RS3_LCD_PIN_RST,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(io_handle, &panel_config, &s_panel), TAG, "new_panel_st7789 failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel_reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel_init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "disp_on failed");
    ESP_LOGI(TAG, "LCD initialized (%dx%d)", RS3_LCD_H_RES, RS3_LCD_V_RES);
    return ESP_OK;
}

rs3_lcd_info_t rs3_lcd_get_info(void)
{
    return (rs3_lcd_info_t){ .w = RS3_LCD_H_RES, .h = RS3_LCD_V_RES };
}

esp_err_t rs3_lcd_draw_full(const uint16_t *fb)
{
    if (!s_panel) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_lcd_panel_draw_bitmap(s_panel, 0, 0, RS3_LCD_H_RES, RS3_LCD_V_RES, fb);
}


