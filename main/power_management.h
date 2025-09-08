#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// 电源管理相关宏定义
#define POWER_VOLTAGE_GPIO 2
#define POWER_VOLTAGE_ATTEN ADC_ATTEN_DB_12  // 最大量程3.9V (ADC Oneshot模式)
#define ULP_CHECK_INTERVAL_US (1 * 1000 * 1000)

/**
 * @brief 初始化ULP程序
 * 
 * @return esp_err_t ESP_OK on success
 */
void power_ulp_init(void);

/**
 * @brief 进入深度睡眠
 * 根据设计文档，当电压低于13V时进入deep-sleep
 */
void power_enter_deep_sleep_with_ulp(void);

uint16_t power_ulp_get_raw_voltage(void);

void awake(void);


/**
 * @brief 初始化电源管理系统
 * 配置ADC用于电源电压检测
 * 
 * @return esp_err_t ESP_OK on success
 */
 esp_err_t power_management_init(void);

 /**
  * @brief 检查电压是否低于阈值
  * 
  * @param voltage 当前电压值
  * @return true 电压过低需要进入省电模式
  * @return false 电压正常
  */
 bool power_is_voltage_low(uint16_t raw_voltage);
 

#endif // POWER_MANAGEMENT_H
