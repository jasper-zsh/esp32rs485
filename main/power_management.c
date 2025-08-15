#include "power_management.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "ulp_power.h"
#include "ulp_adc.h"
#include "ulp_riscv.h"
#include <stdint.h>

extern const uint8_t bin_start[] asm("_binary_ulp_power_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_power_bin_end");

static const char *TAG = "POWER_MANAGEMENT";

void power_ulp_init(void)
{
    ESP_LOGI(TAG, "初始化ULP程序...");
    
    ulp_adc_cfg_t cfg = {
        .adc_n = ADC_UNIT_1,
        .channel = ADC_CHANNEL_1,
        .width = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };
    ESP_ERROR_CHECK(ulp_adc_init(&cfg));

    ESP_ERROR_CHECK(ulp_riscv_load_binary(bin_start, (bin_end - bin_start)));

    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, ULP_CHECK_INTERVAL_US));

    ESP_ERROR_CHECK(ulp_riscv_run());
}

void power_enter_deep_sleep_with_ulp(void)
{
    ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON));
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    esp_deep_sleep_start();
}

uint16_t power_ulp_get_raw_voltage(void)
{
    return ulp_last_result & UINT16_MAX;
}

void awake(void)
{
    esp_sleep_wakeup_cause_t causes = esp_sleep_get_wakeup_cause();
    switch (causes) {
        case ESP_SLEEP_WAKEUP_ULP:
            ESP_LOGI(TAG, "Deep sleep wakeup");
            uint16_t raw_voltage = power_ulp_get_raw_voltage();
            ESP_LOGI(TAG, "Value=%"PRIu16"\n", raw_voltage);
            break;
        default:
            ESP_LOGI(TAG, "Not ULP wakeup (causes = %d)", causes);
            power_ulp_init();
            break;
    }
}

bool power_is_voltage_low(uint16_t raw_voltage)
{
    return raw_voltage != 0 && raw_voltage < ulp_adc_threshold;
}
