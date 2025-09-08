#include <stdint.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_adc_ulp_core.h"

int32_t adc_threshold = 2875;
int32_t wakeup_result;
int32_t last_result;

int main(void)
{
    int32_t l_result, result, d;
    for (;;) {
        result = ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_1);
        d = result - l_result;
        if (d > -10 && d < 10) {
            break;
        }
        l_result = result;
    }
    
    last_result = result;

    if (last_result > adc_threshold) {
        wakeup_result = last_result;
        ulp_riscv_wakeup_main_processor();
    }

    return 0;
}