#include "power_management.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "POWER_MANAGEMENT";

// ADC Oneshot句柄
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;
static adc_unit_t adc_unit;
static adc_channel_t adc_channel;
static bool adc_calibration_init = false;
static bool power_initialized = false;

esp_err_t power_management_init(void)
{
    if (power_initialized) {
        ESP_LOGW(TAG, "电源管理系统已初始化");
        return ESP_OK;
    }

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(POWER_VOLTAGE_GPIO, &adc_unit, &adc_channel));
    ESP_LOGI(TAG, "ADC单元：%d, ADC通道：%d", adc_unit, adc_channel);
    
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = POWER_VOLTAGE_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_channel, &config));

    //-------------ADC1 Calibration Init---------------//
    adc1_cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = adc_unit,
        .chan = adc_channel,
        .atten = POWER_VOLTAGE_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    if (ret == ESP_OK) {
        adc_calibration_init = true;
        ESP_LOGI(TAG, "ADC校准初始化成功 (Curve Fitting)");
    } else {
        ESP_LOGE(TAG, "ADC校准初始化错误: %s", esp_err_to_name(ret));
        adc_calibration_init = false;
    }
    
    ESP_LOGI(TAG, "ADC电源电压检测初始化完成 (GPIO%d -> ADC1_CH%d)", 
             POWER_VOLTAGE_GPIO, adc_channel);

    power_initialized = true;
    return ESP_OK;
}

float power_read_voltage(void)
{
    if (!power_initialized) {
        ESP_LOGE(TAG, "电源管理系统未初始化");
        return -1.0f;
    }

    // 读取ADC原始值
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(adc1_handle, adc_channel, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC读取失败: %s", esp_err_to_name(ret));
        return -1.0f;  // 返回错误值
    }
    
    // 转换为ADC电压值 (mV)
    int adc_voltage_mv = 0;
    if (adc_calibration_init) {
        ret = adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &adc_voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC校准转换失败: %s", esp_err_to_name(ret));
            return -1.0f;  // 返回错误值
        }
    } else {
        // 如果校准未初始化，使用近似计算
        adc_voltage_mv = (adc_raw * 3900) / 4095;  // 假设11dB衰减，满量程3.9V
    }
    
    // 根据分压电路计算实际电源电压 (V)
    float real_voltage_v = ((float)adc_voltage_mv / 1000.0f) / VOLTAGE_DIVIDER_RATIO;
    
    ESP_LOGI(TAG, "电源电压: %.2f V, ADC电压: %d mV, ADC原始值: %d", 
             real_voltage_v, adc_voltage_mv, adc_raw);
    
    return real_voltage_v;
}

bool power_is_voltage_low(float voltage)
{
    return voltage < LOW_VOLTAGE_THRESHOLD_V;
}

void power_enter_deep_sleep(uint64_t sleep_duration_us)
{
    ESP_LOGW(TAG, "进入深度睡眠 %lld 微秒...", sleep_duration_us);
    
    // 配置定时器唤醒源
    esp_sleep_enable_timer_wakeup(sleep_duration_us);
    
    // 进入深度睡眠
    esp_deep_sleep_start();
}

void try_to_deep_sleep(void)
{
    float voltage = power_read_voltage();
    if (power_is_voltage_low(voltage)) {
        ESP_LOGI(TAG, "电压过低，进入深度睡眠");
        power_enter_deep_sleep(1000000);
    }
}