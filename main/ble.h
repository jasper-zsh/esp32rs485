#ifndef BLE_H
#define BLE_H

#include "esp_err.h"

/**
 * @brief 初始化BLE功能
 * 
 * @return esp_err_t 
 *         - ESP_OK: 成功
 *         - 其他值: 失败
 */
esp_err_t ble_init(void);

#endif // BLE_H
