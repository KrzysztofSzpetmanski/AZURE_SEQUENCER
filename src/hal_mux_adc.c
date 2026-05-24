#include "hal_mux_adc.h"

#include "hal_pins_azure.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

static void mux_select(uint8_t ch) {
  gpio_put(HRDW_PIN_MUX_S0, (ch & 0x01u) ? 1u : 0u);
  gpio_put(HRDW_PIN_MUX_S1, (ch & 0x02u) ? 1u : 0u);
  gpio_put(HRDW_PIN_MUX_S2, (ch & 0x04u) ? 1u : 0u);
}

void hal_mux_adc_init(void) {
  gpio_init(HRDW_PIN_MUX_S0);
  gpio_set_dir(HRDW_PIN_MUX_S0, GPIO_OUT);
  gpio_put(HRDW_PIN_MUX_S0, 0);

  gpio_init(HRDW_PIN_MUX_S1);
  gpio_set_dir(HRDW_PIN_MUX_S1, GPIO_OUT);
  gpio_put(HRDW_PIN_MUX_S1, 0);

  gpio_init(HRDW_PIN_MUX_S2);
  gpio_set_dir(HRDW_PIN_MUX_S2, GPIO_OUT);
  gpio_put(HRDW_PIN_MUX_S2, 0);

  adc_init();
  adc_gpio_init(HRDW_PIN_CV_MUX_ADC_GPIO);
  adc_select_input(HRDW_PIN_CV_MUX_ADC_INPUT);
}

uint16_t hal_mux_adc_read_raw(uint8_t channel_0_to_7) {
  uint32_t acc = 0;
  uint8_t i;
  uint8_t ch = (uint8_t)(channel_0_to_7 & 0x07u);

  mux_select(ch);
  sleep_us(80);

  for (i = 0; i < 8u; ++i) {
    adc_select_input(HRDW_PIN_CV_MUX_ADC_INPUT);
    acc += adc_read();
  }
  return (uint16_t)(acc / 8u);
}

int32_t hal_mux_adc_raw_to_mv(uint16_t raw_12bit) {
  uint32_t raw = raw_12bit;
  if (raw > 4095u) raw = 4095u;
  return (int32_t)((raw * 3300u + 2047u) / 4095u);
}

void hal_mux_adc_read_all(uint16_t raw_out[8], int32_t mv_out[8]) {
  uint8_t ch;
  for (ch = 0; ch < 8u; ++ch) {
    uint16_t raw = hal_mux_adc_read_raw(ch);
    if (raw_out != NULL) {
      raw_out[ch] = raw;
    }
    if (mv_out != NULL) {
      mv_out[ch] = hal_mux_adc_raw_to_mv(raw);
    }
  }
}

