#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  HAL_IO_BTN_ENC_L = 0,
  HAL_IO_BTN_ENC_R = 1,
  HAL_IO_BTN_SW_A = 2,
  HAL_IO_BTN_SW_B = 3,
  HAL_IO_BTN_COUNT = 4,
} hal_io_button_t;

typedef enum {
  HAL_IO_ENC_L = 0,
  HAL_IO_ENC_R = 1,
} hal_io_encoder_t;

typedef enum {
  HAL_IO_TR1 = 0,
  HAL_IO_TR2 = 1,
  HAL_IO_TR3 = 2,
  HAL_IO_TR4 = 3,
} hal_io_trigger_t;

void hal_io_init(void);
void hal_io_poll(uint64_t now_ms);

int32_t hal_io_encoder_count(hal_io_encoder_t enc);
uint32_t hal_io_encoder_inc_events(hal_io_encoder_t enc);
uint32_t hal_io_encoder_dec_events(hal_io_encoder_t enc);

bool hal_io_button_pressed(hal_io_button_t btn);
bool hal_io_button_edge_pressed(hal_io_button_t btn);
uint32_t hal_io_button_press_count(hal_io_button_t btn);
bool hal_io_trigger_active(hal_io_trigger_t tr);

bool hal_io_dac_set_channels_mv(const int32_t millivolts_4[4]);
bool hal_io_dac_set_all_mv(int32_t millivolts);
bool hal_io_dac_set_channels_code(const uint16_t codes_4[4]);

void hal_io_oled_clear(void);
void hal_io_oled_draw_line(uint8_t row, const char* text, bool inverted);

#ifdef __cplusplus
}
#endif
