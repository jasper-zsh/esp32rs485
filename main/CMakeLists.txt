idf_component_register(SRCS "main.c" "power_management.c" "ble.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt nvs_flash driver esp_adc)