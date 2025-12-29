idf_component_register(
    SRCS
        "main.c"
        "nikon_bt.cpp"
        "pmu_axp2101.c"
        "lcd_st7789.c"
        "font5x7.c"
        "wifi_sta.c"
        "ui_status.c"
        "tcp_server.c"
        "cmd_tcp.c"
        "ota_update.c"
        "log_tcp.c"
        "touch_cst816.c"
        "rec_events.c"
        "usb_ptp_cam.c"
        "usb_ptp_cam_std.c"
        "usb_ptp_proxy.c"
        "ptp_proxy_server.c"
    INCLUDE_DIRS "."
    REQUIRES esp_psram esp_lcd driver esp_wifi esp_event esp_netif nvs_flash bt esp_https_ota esp_http_client esp-tls app_update mbedtls espressif__esp_tinyusb espressif__tinyusb
    PRIV_REQUIRES spi_flash esp_timer
)


