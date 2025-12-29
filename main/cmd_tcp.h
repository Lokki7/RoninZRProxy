#pragma once

#include "esp_err.h"

/**
 * @brief Register TCP command handler (listens for "ota" commands on the TCP server connection).
 */
esp_err_t rs3_cmd_tcp_start(void);


