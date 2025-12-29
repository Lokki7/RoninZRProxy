#pragma once

#include "esp_err.h"

/**
 * @brief Start USB device stack with a Still Image (PTP) interface that logs incoming ops via TCP.
 */
esp_err_t rs3_usb_ptp_cam_start(void);


