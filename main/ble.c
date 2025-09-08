#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "BLE_RS485";

// BLE服务和特性定义
#define DEVICE_NAME             "ESP32-RS485"
#define SERVICE_UUID            0xFF00
#define SETTINGS_CHAR_UUID      0xFFF0
#define RS485_1_CHAR_UUID       0xFFF1
#define RS485_2_CHAR_UUID       0xFFF2

// 开关控制引脚定义
#define SWITCH_OUTPUT_GPIO      8

// 配置项和指令枚举
#define CONFIG_PAIRED_DEVICES   0x01
#define CONFIG_RS485_MODES      0x02
#define CMD_PAIR                0x01
#define CMD_UNPAIR              0x02
#define CMD_CLEAR_PAIR          0x03
#define CMD_READ_CONFIG         0x04
#define CMD_WRITE_CONFIG        0x05
#define CMD_SWITCH_CONTROL      0x06

// RS485模式枚举
#define RS485_MODE_NONE         0x00
#define RS485_MODE_RAW          0x01
#define RS485_MODE_MODBUS_RTU   0x02
#define RS485_MODE_MODBUS_ASCII 0x03

// NVS和硬件配置
#define NVS_NAMESPACE           "ble_config"
#define NVS_KEY_PAIRED_DEVICES  "paired_devs"
#define NVS_KEY_RS485_MODES     "rs485_modes"
#define MAX_PAIRED_DEVICES      10
#define RS485_2_TX_GPIO         10
#define RS485_2_RX_GPIO         11
#define RS485_2_UART_PORT       UART_NUM_2
#define RS485_2_BAUD_RATE       9600
#define RS485_2_BUF_SIZE        1024

// BLE句柄和状态变量
static uint16_t gatts_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;
static uint16_t service_handle = 0;
static uint16_t settings_char_handle = 0;
static uint16_t rs485_1_char_handle = 0;
static uint16_t rs485_2_char_handle = 0;
static bool is_connected = false;
static uint8_t current_client_mac[6] = {0};

// 配置数据结构
typedef struct {
    uint8_t mac_addr[6];
} paired_device_t;

typedef struct {
    paired_device_t devices[MAX_PAIRED_DEVICES];
    uint8_t count;
} paired_devices_config_t;

typedef struct {
    uint8_t channel_0_mode;
    uint8_t channel_1_mode;
} rs485_modes_config_t;

static paired_devices_config_t g_paired_devices = {0};
static rs485_modes_config_t g_rs485_modes = {0};

// BLE广播数据
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// 函数声明
static esp_err_t save_config_to_nvs(void);
static esp_err_t load_config_from_nvs(void);
static bool is_device_paired(uint8_t *mac_addr);
static esp_err_t add_paired_device(uint8_t *mac_addr);
static esp_err_t remove_paired_device(uint8_t *mac_addr);
static void clear_paired_devices(void);
static void send_response(uint8_t cmd, uint8_t *data, uint16_t len);
static void handle_settings_command(uint8_t *data, uint16_t len);
static void handle_rs485_data(uint8_t channel, uint8_t *data, uint16_t len);
static void configure_rs485_uart2(void);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// 开关控制相关函数声明
void ble_init_switch_control(void);
void ble_set_switch_state(uint8_t state);
static void switch_control_task(void *pvParameters);

/**
 * @brief 配置RS485第二路UART
 */
static void configure_rs485_uart2(void)
{
    uart_config_t uart_config = {
        .baud_rate = RS485_2_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(RS485_2_UART_PORT, RS485_2_BUF_SIZE, 
                                        RS485_2_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_2_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS485_2_UART_PORT, RS485_2_TX_GPIO, 
                                 RS485_2_RX_GPIO, UART_PIN_NO_CHANGE, 
                                 UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "RS485 UART2初始化完成，波特率: %d", RS485_2_BAUD_RATE);
}

/**
 * @brief 保存配置到NVS
 */
static esp_err_t save_config_to_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_set_blob(nvs_handle, NVS_KEY_PAIRED_DEVICES, &g_paired_devices, sizeof(paired_devices_config_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存配对设备列表失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    ret = nvs_set_blob(nvs_handle, NVS_KEY_RS485_MODES, &g_rs485_modes, sizeof(rs485_modes_config_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存RS485模式配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return ret;
}

/**
 * @brief 从NVS加载配置
 */
static esp_err_t load_config_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "NVS命名空间不存在，使用默认配置");
        return ESP_OK;
    }
    
    size_t required_size;
    
    required_size = sizeof(paired_devices_config_t);
    ret = nvs_get_blob(nvs_handle, NVS_KEY_PAIRED_DEVICES, &g_paired_devices, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "未找到配对设备列表，使用默认配置");
        memset(&g_paired_devices, 0, sizeof(paired_devices_config_t));
    } else {
        ESP_LOGI(TAG, "已加载%d个配对设备", g_paired_devices.count);
    }
    
    required_size = sizeof(rs485_modes_config_t);
    ret = nvs_get_blob(nvs_handle, NVS_KEY_RS485_MODES, &g_rs485_modes, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "未找到RS485模式配置，使用默认配置");
        memset(&g_rs485_modes, 0, sizeof(rs485_modes_config_t));
    } else {
        ESP_LOGI(TAG, "已加载RS485模式配置: 通道0=%d, 通道1=%d", 
                g_rs485_modes.channel_0_mode, g_rs485_modes.channel_1_mode);
    }
    
    nvs_close(nvs_handle);
    return ESP_OK;
}

/**
 * @brief 检查设备是否已配对
 */
static bool is_device_paired(uint8_t *mac_addr)
{
    return true;
    for (int i = 0; i < g_paired_devices.count; i++) {
        if (memcmp(g_paired_devices.devices[i].mac_addr, mac_addr, 6) == 0) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 添加配对设备
 */
static esp_err_t add_paired_device(uint8_t *mac_addr)
{
    if (g_paired_devices.count >= MAX_PAIRED_DEVICES) {
        ESP_LOGW(TAG, "配对设备列表已满");
        return ESP_ERR_NO_MEM;
    }
    
    if (is_device_paired(mac_addr)) {
        ESP_LOGW(TAG, "设备已存在于配对列表中");
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(g_paired_devices.devices[g_paired_devices.count].mac_addr, mac_addr, 6);
    g_paired_devices.count++;
    
    ESP_LOGI(TAG, "添加配对设备: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    
    return save_config_to_nvs();
}

/**
 * @brief 移除配对设备
 */
static esp_err_t remove_paired_device(uint8_t *mac_addr)
{
    int found_index = -1;
    
    for (int i = 0; i < g_paired_devices.count; i++) {
        if (memcmp(g_paired_devices.devices[i].mac_addr, mac_addr, 6) == 0) {
            found_index = i;
            break;
        }
    }
    
    if (found_index == -1) {
        ESP_LOGW(TAG, "设备不在配对列表中");
        return ESP_ERR_NOT_FOUND;
    }
    
    for (int i = found_index; i < g_paired_devices.count - 1; i++) {
        memcpy(&g_paired_devices.devices[i], &g_paired_devices.devices[i + 1], sizeof(paired_device_t));
    }
    g_paired_devices.count--;
    
    ESP_LOGI(TAG, "移除配对设备: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    
    return save_config_to_nvs();
}

/**
 * @brief 清空配对设备列表
 */
static void clear_paired_devices(void)
{
    memset(&g_paired_devices, 0, sizeof(paired_devices_config_t));
    ESP_LOGI(TAG, "已清空配对设备列表");
    save_config_to_nvs();
}

/**
 * @brief 发送响应数据
 */
static void send_response(uint8_t cmd, uint8_t *data, uint16_t len)
{
    if (!is_connected) {
        return;
    }
    
    uint8_t response[256];
    response[0] = cmd;
    
    if (data && len > 0 && len < 255) {
        memcpy(&response[1], data, len);
        esp_ble_gatts_send_indicate(gatts_if, conn_id, settings_char_handle, len + 1, response, false);
    } else {
        esp_ble_gatts_send_indicate(gatts_if, conn_id, settings_char_handle, 1, response, false);
    }
}

/**
 * @brief 处理设置特性命令
 */
static void handle_settings_command(uint8_t *data, uint16_t len)
{
    if (len < 1) {
        ESP_LOGW(TAG, "命令数据长度不足");
        return;
    }
    
    uint8_t cmd = data[0];
    ESP_LOGI(TAG, "收到命令: 0x%02X, 长度: %d", cmd, len);
    
    switch (cmd) {
        case CMD_PAIR: {
            if (len != 7) {
                ESP_LOGW(TAG, "配对命令参数长度错误");
                send_response(cmd, NULL, 0);
                return;
            }
            
            uint8_t *mac_addr = &data[1];
            
            // 当已配对设备列表为空时，允许任何设备执行配对操作
            if (g_paired_devices.count == 0) {
                ESP_LOGI(TAG, "配对设备列表为空，允许任何设备配对");
                add_paired_device(mac_addr);
                send_response(cmd, NULL, 0);
            } else if (is_device_paired(current_client_mac)) {
                // 仅允许已配对设备添加新的配对设备
                ESP_LOGI(TAG, "已配对设备执行配对操作");
                add_paired_device(mac_addr);
                send_response(cmd, NULL, 0);
            } else {
                ESP_LOGW(TAG, "未配对设备尝试添加配对，操作被拒绝");
                send_response(cmd, NULL, 0);
            }
            break;
        }
        
        case CMD_UNPAIR: {
            if (len != 7) {
                ESP_LOGW(TAG, "取消配对命令参数长度错误");
                send_response(cmd, NULL, 0);
                return;
            }
            
            if (!is_device_paired(current_client_mac)) {
                ESP_LOGW(TAG, "未配对设备尝试取消配对");
                send_response(cmd, NULL, 0);
                return;
            }
            
            uint8_t *mac_addr = &data[1];
            remove_paired_device(mac_addr);
            send_response(cmd, NULL, 0);
            break;
        }
        
        case CMD_CLEAR_PAIR: {
            if (!is_device_paired(current_client_mac)) {
                ESP_LOGW(TAG, "未配对设备尝试清空配对列表");
                send_response(cmd, NULL, 0);
                return;
            }
            
            clear_paired_devices();
            send_response(cmd, NULL, 0);
            break;
        }
        
        case CMD_READ_CONFIG: {
            if (len != 2) {
                ESP_LOGW(TAG, "读取配置命令参数长度错误");
                send_response(cmd, NULL, 0);
                return;
            }
            
            if (!is_device_paired(current_client_mac)) {
                ESP_LOGW(TAG, "未配对设备尝试读取配置");
                send_response(cmd, NULL, 0);
                return;
            }
            
            uint8_t config_type = data[1];
            
            switch (config_type) {
                case CONFIG_PAIRED_DEVICES: {
                    uint8_t response_data[MAX_PAIRED_DEVICES * 6];
                    for (int i = 0; i < g_paired_devices.count; i++) {
                        memcpy(&response_data[i * 6], g_paired_devices.devices[i].mac_addr, 6);
                    }
                    send_response(cmd, response_data, g_paired_devices.count * 6);
                    break;
                }
                
                case CONFIG_RS485_MODES: {
                    uint8_t response_data[4];
                    response_data[0] = 0;
                    response_data[1] = g_rs485_modes.channel_0_mode;
                    response_data[2] = 1;
                    response_data[3] = g_rs485_modes.channel_1_mode;
                    send_response(cmd, response_data, 4);
                    break;
                }
                
                default:
                    ESP_LOGW(TAG, "未知配置类型: 0x%02X", config_type);
                    send_response(cmd, NULL, 0);
                    break;
            }
            break;
        }
        
        case CMD_WRITE_CONFIG: {
            if (len < 3) {
                ESP_LOGW(TAG, "写入配置命令参数长度错误");
                send_response(cmd, NULL, 0);
                return;
            }
            
            if (!is_device_paired(current_client_mac)) {
                ESP_LOGW(TAG, "未配对设备尝试写入配置");
                send_response(cmd, NULL, 0);
                return;
            }
            
            uint8_t config_type = data[1];
            
            switch (config_type) {
                case CONFIG_RS485_MODES: {
                    for (int i = 2; i < len; i += 2) {
                        if (i + 1 >= len) break;
                        
                        uint8_t channel = data[i];
                        uint8_t mode = data[i + 1];
                        
                        if (channel == 0) {
                            g_rs485_modes.channel_0_mode = mode;
                            ESP_LOGI(TAG, "设置RS485通道0模式为: %d", mode);
                        } else if (channel == 1) {
                            g_rs485_modes.channel_1_mode = mode;
                            ESP_LOGI(TAG, "设置RS485通道1模式为: %d", mode);
                        }
                    }
                    save_config_to_nvs();
                    send_response(cmd, NULL, 0);
                    break;
                }
                
                default:
                    ESP_LOGW(TAG, "不支持写入的配置类型: 0x%02X", config_type);
                    send_response(cmd, NULL, 0);
                    break;
            }
            break;
        }
        
        case CMD_SWITCH_CONTROL: {
            if (len < 3) {
                ESP_LOGW(TAG, "开关控制命令参数长度错误");
                send_response(cmd, NULL, 0);
                return;
            }
            
            if (!is_device_paired(current_client_mac)) {
                ESP_LOGW(TAG, "未配对设备尝试控制开关");
                send_response(cmd, NULL, 0);
                return;
            }
            
            // 参数是2字节的开关导通时长（单位毫秒）
            uint16_t duration = (data[1] << 8) | data[2];
            ESP_LOGI(TAG, "开关控制命令，导通时长: %d毫秒", duration);
            
            // 收到指令表示将开关开启（导通），设置为低电平
            ble_set_switch_state(0);
            
            // 如果时长为0，则不自动关闭
            if (duration > 0) {
                // 创建一个任务在指定时间后关闭开关
                // 注意：实际项目中应该使用更高效的方式，比如定时器
                xTaskCreate(switch_control_task, "switch_ctrl", 2048, (void*)(uint32_t)duration, 5, NULL);
            }
            
            send_response(cmd, NULL, 0);
            break;
        }
        
        default:
            ESP_LOGW(TAG, "未知命令: 0x%02X", cmd);
            send_response(cmd, NULL, 0);
            break;
    }
}

/**
 * @brief 处理RS485通道数据
 */
static void handle_rs485_data(uint8_t channel, uint8_t *data, uint16_t len)
{
    if (channel > 1) {
        ESP_LOGW(TAG, "无效的RS485通道: %d", channel);
        return;
    }
    
    uint8_t mode = (channel == 0) ? g_rs485_modes.channel_0_mode : g_rs485_modes.channel_1_mode;
    uart_port_t uart_port = (channel == 0) ? UART_NUM_1 : UART_NUM_2;
    
    if (mode == RS485_MODE_NONE) {
        ESP_LOGW(TAG, "RS485通道%d未设置模式", channel);
        return;
    }
    
    switch (mode) {
        case RS485_MODE_RAW:
            uart_write_bytes(uart_port, (const char*)data, len);
            ESP_LOGI(TAG, "RS485通道%d发送RAW数据，长度: %d", channel, len);
            break;
            
        case RS485_MODE_MODBUS_RTU:
            ESP_LOGI(TAG, "RS485通道%d Modbus RTU模式 - 待实现", channel);
            break;
            
        case RS485_MODE_MODBUS_ASCII:
            ESP_LOGI(TAG, "RS485通道%d Modbus ASCII模式 - 待实现", channel);
            break;
            
        default:
            ESP_LOGW(TAG, "RS485通道%d未知模式: %d", channel, mode);
            break;
    }
}

/**
 * @brief GAP事件处理器
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "广播启动失败");
            } else {
                ESP_LOGI(TAG, "开始BLE广播");
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "广播停止失败");
            } else {
                ESP_LOGI(TAG, "BLE广播已停止");
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief GATTS事件处理器
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if_param, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATTS注册成功，app_id: %d", param->reg.app_id);
            gatts_if = gatts_if_param;
            
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            
            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id.inst_id = 0x00,
                .id.uuid.len = ESP_UUID_LEN_16,
                .id.uuid.uuid.uuid16 = SERVICE_UUID,
            };
            esp_ble_gatts_create_service(gatts_if, &service_id, 10);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "服务创建成功，service_handle: %d", param->create.service_handle);
            service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(service_handle);
            
            esp_bt_uuid_t settings_char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = SETTINGS_CHAR_UUID,
            };
            esp_ble_gatts_add_char(service_handle, &settings_char_uuid,
                                   ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "特性添加成功，char_handle: %d, uuid: 0x%04X", 
                     param->add_char.attr_handle, param->add_char.char_uuid.uuid.uuid16);
            
            if (param->add_char.char_uuid.uuid.uuid16 == SETTINGS_CHAR_UUID) {
                settings_char_handle = param->add_char.attr_handle;
                
                esp_bt_uuid_t rs485_1_char_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid.uuid16 = RS485_1_CHAR_UUID,
                };
                esp_ble_gatts_add_char(service_handle, &rs485_1_char_uuid,
                                       ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ,
                                       ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                       NULL, NULL);
            } else if (param->add_char.char_uuid.uuid.uuid16 == RS485_1_CHAR_UUID) {
                rs485_1_char_handle = param->add_char.attr_handle;
                
                esp_bt_uuid_t rs485_2_char_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid.uuid16 = RS485_2_CHAR_UUID,
                };
                esp_ble_gatts_add_char(service_handle, &rs485_2_char_uuid,
                                       ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ,
                                       ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                       NULL, NULL);
            } else if (param->add_char.char_uuid.uuid.uuid16 == RS485_2_CHAR_UUID) {
                rs485_2_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "所有BLE特性创建完成");
            }
            break;
            
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "服务启动成功");
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "BLE设备连接，conn_id: %d", param->connect.conn_id);
            conn_id = param->connect.conn_id;
            is_connected = true;
            
            memcpy(current_client_mac, param->connect.remote_bda, 6);
            ESP_LOGI(TAG, "连接设备MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     current_client_mac[0], current_client_mac[1], current_client_mac[2],
                     current_client_mac[3], current_client_mac[4], current_client_mac[5]);
            
            esp_ble_gap_stop_advertising();
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "BLE设备断开连接");
            is_connected = false;
            memset(current_client_mac, 0, 6);
            
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "收到写入请求，handle: %d, len: %d", param->write.handle, param->write.len);
            
            if (param->write.handle == settings_char_handle) {
                handle_settings_command(param->write.value, param->write.len);
            } else if (param->write.handle == rs485_1_char_handle) {
                handle_rs485_data(0, param->write.value, param->write.len);
            } else if (param->write.handle == rs485_2_char_handle) {
                handle_rs485_data(1, param->write.value, param->write.len);
            }
            
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                        ESP_GATT_OK, NULL);
            break;
            
        default:
            break;
    }
}

/**
 * @brief RS485数据接收任务
 */
static void rs485_receive_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(1024);
    
    while (1) {
        int len1 = uart_read_bytes(UART_NUM_1, data, 1024, 100 / portTICK_PERIOD_MS);
        if (len1 > 0 && is_connected) {
            if (g_rs485_modes.channel_0_mode != RS485_MODE_NONE) {
                esp_ble_gatts_send_indicate(gatts_if, conn_id, rs485_1_char_handle, len1, data, false);
                ESP_LOGI(TAG, "RS485通道0接收到%d字节数据，已通过BLE发送", len1);
            }
        }
        
        int len2 = uart_read_bytes(UART_NUM_2, data, 1024, 100 / portTICK_PERIOD_MS);
        if (len2 > 0 && is_connected) {
            if (g_rs485_modes.channel_1_mode != RS485_MODE_NONE) {
                esp_ble_gatts_send_indicate(gatts_if, conn_id, rs485_2_char_handle, len2, data, false);
                ESP_LOGI(TAG, "RS485通道1接收到%d字节数据，已通过BLE发送", len2);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    free(data);
}

/**
 * @brief 初始化开关控制GPIO
 */
void ble_init_switch_control(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << SWITCH_OUTPUT_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    // 默认将开关设置为关断状态（高电平）
    gpio_set_level(SWITCH_OUTPUT_GPIO, 1);
    ESP_LOGI(TAG, "开关控制GPIO初始化完成，引脚: %d", SWITCH_OUTPUT_GPIO);
}

/**
 * @brief 设置开关状态
 * 
 * @param state 开关状态 (0=导通/低电平, 1=关断/高电平)
 */
void ble_set_switch_state(uint8_t state)
{
    // 根据硬件设计，低电平导通开关，高电平关断开关
    gpio_set_level(SWITCH_OUTPUT_GPIO, state ? 1 : 0);
    ESP_LOGI(TAG, "设置开关状态: %s", state ? "关断(高电平)" : "导通(低电平)");
}

/**
 * @brief 开关控制任务
 * 
 * @param pvParameters 开关导通时长（毫秒）
 */
static void switch_control_task(void *pvParameters)
{
    uint16_t duration = (uint32_t)pvParameters;
    ESP_LOGI(TAG, "开关控制任务启动，将在%d毫秒后关断开关", duration);
    
    // 延时指定的时间
    vTaskDelay(pdMS_TO_TICKS(duration));
    
    // 关断开关（高电平）
    ble_set_switch_state(1);
    ESP_LOGI(TAG, "开关控制任务完成，开关已关断");
    
    // 删除任务
    vTaskDelete(NULL);
}

/**
 * @brief 清空已配对设备列表（恢复出厂设置使用）
 */
void ble_clear_paired_devices(void)
{
    clear_paired_devices();
}

/**
 * @brief 初始化BLE
 */
esp_err_t ble_init(void)
{
    esp_err_t ret;
    
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    load_config_from_nvs();
    configure_rs485_uart2();
    
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "蓝牙控制器初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "蓝牙控制器启用失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "蓝牙主机栈初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "蓝牙主机栈启用失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0);
    
    esp_ble_gatt_set_local_mtu(500);
    
    xTaskCreate(rs485_receive_task, "rs485_rx", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "BLE初始化完成");
    return ESP_OK;
}
