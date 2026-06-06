#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  HAL_IO_BTN_ENC_L = 0,
  HAL_IO_BTN_ENC_R = 1,
  HAL_IO_BTN_SW1 = 2,
  HAL_IO_BTN_SW2 = 3,
  HAL_IO_BTN_SW_A = HAL_IO_BTN_SW1,
  HAL_IO_BTN_SW_B = HAL_IO_BTN_SW2,
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

typedef struct {
  uint32_t write_calls;
  uint32_t channel_writes;
  uint32_t skipped_write_calls;
  uint32_t skipped_channel_writes;
} hal_io_dac_diag_t;

void hal_io_dac_get_diag(hal_io_dac_diag_t* diag);

void hal_io_oled_clear(void);
void hal_io_oled_draw_line(uint8_t row, const char* text, bool inverted);
void hal_io_oled_draw_line_color(uint8_t row, const char* text, uint16_t fg, uint16_t bg);
void hal_io_oled_draw_text(uint8_t x, uint8_t y, const char* text, bool inverted);
void hal_io_oled_draw_pixel(uint8_t x, uint8_t y, bool white);
void hal_io_oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool white);
void hal_io_oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool white);
void hal_io_oled_fill_rect_color(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void hal_io_oled_draw_rect_color(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);

#ifdef __cplusplus
}
#endif
