#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

// 电源管理相关宏定义
#define POWER_VOLTAGE_GPIO 2
#define POWER_VOLTAGE_ATTEN ADC_ATTEN_DB_12  // 最大量程3.9V (ADC Oneshot模式)

// 电源分压电路参数: 220K / (910K + 220K) = 220K / 1130K
#define VOLTAGE_DIVIDER_R1 220000  // 分压电阻R1 (Ohm)
#define VOLTAGE_DIVIDER_R2 910000  // 分压电阻R2 (Ohm)
#define VOLTAGE_DIVIDER_OFFSET 0.2
#define VOLTAGE_DIVIDER_RATIO ((float)VOLTAGE_DIVIDER_R1 / (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2))

// 电源管理参数
#define LOW_VOLTAGE_THRESHOLD_V 13.0f  // 低电压阈值（V）
#define SLEEP_DURATION_US (1 * 1000 * 1000)  // 深度睡眠持续时间（微秒）- 1秒

/**
 * @brief 初始化电源管理系统
 * 配置ADC用于电源电压检测
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t power_management_init(void);

/**
 * @brief 读取当前电源电压值
 * 
 * @return float 电源电压值（伏特），返回 -1.0f 表示读取失败
 */
float power_read_voltage(void);

/**
 * @brief 检查电压是否低于阈值
 * 
 * @param voltage 当前电压值
 * @return true 电压过低需要进入省电模式
 * @return false 电压正常
 */
bool power_is_voltage_low(float voltage);

/**
 * @brief 进入深度睡眠模式
 * 
 * @param sleep_duration_us 睡眠持续时间（微秒）
 */
void power_enter_deep_sleep(uint64_t sleep_duration_us);

void try_to_deep_sleep(void);

#endif // POWER_MANAGEMENT_H
