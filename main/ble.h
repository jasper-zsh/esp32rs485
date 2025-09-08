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

/**
 * @brief 清空已配对设备列表（恢复出厂设置使用）
 */
void ble_clear_paired_devices(void);

#endif // BLE_H
