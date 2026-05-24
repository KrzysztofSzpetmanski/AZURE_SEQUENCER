#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hal_mux_adc_init(void);
uint16_t hal_mux_adc_read_raw(uint8_t channel_0_to_7);
int32_t hal_mux_adc_raw_to_mv(uint16_t raw_12bit);
void hal_mux_adc_read_all(uint16_t raw_out[8], int32_t mv_out[8]);

#ifdef __cplusplus
}
#endif

