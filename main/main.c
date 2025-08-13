#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"

#define WS2812_GPIO 21
#define RESET_BUTTON_GPIO 4
#define LED_STRIP_NUM 1
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)  // 10MHz resolution

static const char *TAG = "ESP32_RS485";

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,        // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_NUM,            // The number of LEDs in the strip
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false,              // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,       // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 0,               // let the driver choose a proper memory block size automatically
        .flags = {
            .with_dma = false,                // Using DMA can improve performance when driving more LEDs
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

void app_main(void)
{
    ESP_LOGI(TAG, "开始初始化ESP32 RS485系统...");
    
    // 初始化LED strip
    led_strip_handle_t led_strip = configure_led();
    
    // 配置恢复出厂设置按键为输入模式
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RESET_BUTTON_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;  // 启用内部上拉电阻
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "开始检测恢复出厂设置按键状态...");
    
    while (1) {
        // 读取按键状态
        int button_state = gpio_get_level(RESET_BUTTON_GPIO);
        
        if (button_state == 0) {  // 按键被按下（低电平）
            ESP_LOGI(TAG, "恢复出厂设置按键被按下，点亮绿色LED");
            
            // 设置LED为绿色 (由于GRB格式，参数顺序为: index, G, R, B)
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            
        } else {  // 按键未被按下（高电平）
            // 关闭LED
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 延时100ms
    }
}