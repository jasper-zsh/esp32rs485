#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "power_management.h"
#include "ble.h"
#include "freertos/task.h"
#include "led_strip.h"

#define WS2812_GPIO 21
#define RESET_BUTTON_GPIO 4
#define LED_STRIP_NUM 1
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)  // 10MHz resolution

// RS485第一路配置 (对应UART1)
#define RS485_1_TX_GPIO 12
#define RS485_1_RX_GPIO 13
#define RS485_1_UART_PORT UART_NUM_1
#define RS485_1_BAUD_RATE 9600
#define RS485_1_BUF_SIZE 1024

static const char *TAG = "ESP32_RS485";

void configure_rs485_uart1(void)
{
    // 配置UART1参数
    uart_config_t uart_config = {
        .baud_rate = RS485_1_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 安装UART驱动
    ESP_ERROR_CHECK(uart_driver_install(RS485_1_UART_PORT, RS485_1_BUF_SIZE, 
                                        RS485_1_BUF_SIZE, 0, NULL, 0));
    
    // 配置UART参数
    ESP_ERROR_CHECK(uart_param_config(RS485_1_UART_PORT, &uart_config));
    
    // 设置UART引脚 (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(RS485_1_UART_PORT, RS485_1_TX_GPIO, 
                                 RS485_1_RX_GPIO, UART_PIN_NO_CHANGE, 
                                 UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "RS485 UART1初始化完成，波特率: %d", RS485_1_BAUD_RATE);
}

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
    
    // 初始化电源管理系统
    ESP_ERROR_CHECK(power_management_init());
    try_to_deep_sleep();
    
    // 初始化BLE功能
    ESP_ERROR_CHECK(ble_init());
    
    // 初始化RS485 UART1
    configure_rs485_uart1();
    
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
    
    ESP_LOGI(TAG, "系统已初始化完成，开始电源监控循环...");
    
    int last_button_state = 1;  // 上次按键状态 (初始为高电平)
    
    while (1) {
        // 每秒读取一次电源电压
        float voltage = power_read_voltage();
        
        if (voltage < 0) {
            ESP_LOGE(TAG, "电压读取失败，等待1秒后重试...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // 检查电压是否低于阈值
        if (power_is_voltage_low(voltage)) {
            ESP_LOGW(TAG, "电压过低 (%.2f V < %.2f V)，进入深度睡眠1秒...", voltage, LOW_VOLTAGE_THRESHOLD_V);
            
            // 清除LED状态
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            
            // 进入深度睡眠
            power_enter_deep_sleep(SLEEP_DURATION_US);
        }
        
        // 电压正常，进入正常工作模式1秒
        ESP_LOGI(TAG, "电压正常 (%.2f V)，开始正常工作1秒...", voltage);
        
        // 在1秒内检测按键状态 (每100ms检测一次，共检测10次)
        for (int i = 0; i < 10; i++) {
            // 读取按键状态
            int button_state = gpio_get_level(RESET_BUTTON_GPIO);
            
            if (button_state == 0) {  // 按键被按下（低电平）
                ESP_LOGI(TAG, "恢复出厂设置按键被按下，点亮绿色LED");
                
                // 设置LED为绿色 (由于GRB格式，参数顺序为: index, G, R, B)
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                
                // 检测按键从高电平变为低电平的瞬间，发送RESET字符串
                if (last_button_state == 1) {
                    const char* reset_msg = "RESET\r\n";
                    uart_write_bytes(RS485_1_UART_PORT, reset_msg, strlen(reset_msg));
                    ESP_LOGI(TAG, "通过RS485发送: %s", reset_msg);
                }
                
            } else {  // 按键未被按下（高电平）
                // 关闭LED
                ESP_ERROR_CHECK(led_strip_clear(led_strip));
            }
            
            last_button_state = button_state;  // 更新上次按键状态
            vTaskDelay(pdMS_TO_TICKS(100));  // 延时100ms
        }
    }
}