set(ulp_app_name ulp_power)
set(ulp_riscv_sources ulp/power.c)
set(ulp_exp_dep_srcs "power_management.c")

idf_component_register(SRCS "main.c" "power_management.c" "ble.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt nvs_flash driver esp_adc ulp soc led_strip)

# 构建ULP应用程序
ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")