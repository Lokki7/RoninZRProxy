#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start Bluetooth (NimBLE) and attempt to connect to the last paired Nikon camera.
 *
 * If Bluetooth/NimBLE is disabled in sdkconfig, this returns ESP_ERR_NOT_SUPPORTED.
 */
esp_err_t rs3_nikon_bt_start(void);

/**
 * Start Nikon remote pairing flow.
 *
 * Behavior:
 * - start scan for Nikon camera in pairing mode
 * - connect to the first matching camera found
 * - perform Nikon remote-mode GATT handshake
 * - persist identifiers for future auto-reconnect
 */
esp_err_t rs3_nikon_bt_pair_start(void);

/**
 * Trigger Nikon shutter (press + release).
 *
 * Requires an active BLE connection to the camera.
 */
esp_err_t rs3_nikon_bt_shutter_click(void);

#ifdef __cplusplus
}
#endif


