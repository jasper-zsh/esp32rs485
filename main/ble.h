#ifndef BLE_H
#define BLE_H

#include "esp_err.h"

// 开关控制引脚定义
#define SWITCH_OUTPUT_GPIO 8

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

/**
 * @brief 初始化开关控制GPIO
 */
void ble_init_switch_control(void);

/**
 * @brief 设置开关状态
 * 
 * @param state 开关状态 (0=关闭, 1=开启)
 */
void ble_set_switch_state(uint8_t state);

#endif // BLE_H
