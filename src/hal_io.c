#include "hal_io.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "hal_pins_azure.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define OLED_W 160
#define OLED_H 128

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF

#define MCP4728_I2C_ADDR_DEFAULT 0x60u
#define MCP4728_I2C_ADDR_MAX 0x67u
#define DAC_I2C_BAUDRATE 400000u
#define DAC_CODE_MAX_12BIT 4095u
#define BTN_DEBOUNCE_MS 25u
#define ENCODER_PIN_MASK 0x03u
#define ENCODER_PIN_RISING_EDGE 0x02u

typedef struct {
  char ch;
  uint8_t cols[5];
} glyph_t;

typedef struct {
  uint8_t pin_a;
  uint8_t pin_b;
  bool invert_direction;
  uint8_t pin_state_a;
  uint8_t pin_state_b;
  int32_t count;
  uint32_t inc_events;
  uint32_t dec_events;
} encoder_state_t;

typedef struct {
  uint8_t pin;
  bool level_raw;
  bool level_stable;
  bool edge_pressed;
  uint64_t last_change_ms;
  uint32_t press_count;
} button_state_t;

static const glyph_t k_font[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'+', {0x08, 0x08, 0x3E, 0x08, 0x08}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'_', {0x40, 0x40, 0x40, 0x40, 0x40}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
    {'?', {0x02, 0x01, 0x51, 0x09, 0x06}},
    {'>', {0x08, 0x14, 0x22, 0x41, 0x00}},
    {'(', {0x00, 0x1C, 0x22, 0x41, 0x00}},
    {')', {0x00, 0x41, 0x22, 0x1C, 0x00}},
};

static encoder_state_t g_enc_l;
static encoder_state_t g_enc_r;
static button_state_t g_buttons[HAL_IO_BTN_COUNT];
static struct repeating_timer g_encoder_poll_timer;

static bool g_dac_addr_found = false;
static uint8_t g_dac_i2c_addr = MCP4728_I2C_ADDR_DEFAULT;
static bool g_dac_cache_valid = false;
static uint16_t g_dac_last_codes[4] = {0u, 0u, 0u, 0u};
static hal_io_dac_diag_t g_dac_diag = {0u, 0u, 0u, 0u};

static inline bool is_pressed(uint8_t pin) { return !gpio_get(pin); }

static void poll_encoder(encoder_state_t* enc) {
  uint8_t step = 0;
  uint8_t a;
  uint8_t b;

  enc->pin_state_a = (uint8_t)((enc->pin_state_a << 1u) | (gpio_get(enc->pin_a) ? 1u : 0u));
  enc->pin_state_b = (uint8_t)((enc->pin_state_b << 1u) | (gpio_get(enc->pin_b) ? 1u : 0u));

  a = (uint8_t)(enc->pin_state_a & ENCODER_PIN_MASK);
  b = (uint8_t)(enc->pin_state_b & ENCODER_PIN_MASK);

  // Same strategy as O_C-Phazerville: detect a clean rising edge on one pin
  // while the other is low. This is very robust for these mechanical encoders.
  if (a == ENCODER_PIN_RISING_EDGE && b == 0x00u) {
    step = 1u;
  } else if (b == ENCODER_PIN_RISING_EDGE && a == 0x00u) {
    step = 2u;  // encoded as -1 below
  }

  if (step != 0u) {
    int32_t delta = (step == 1u) ? 1 : -1;
    if (enc->invert_direction) {
      delta = -delta;
    }
    enc->count += delta;
    if (delta > 0) {
      enc->inc_events += 1u;
    } else {
      enc->dec_events += 1u;
    }
  }
}

static bool encoder_poll_timer_cb(struct repeating_timer* timer) {
  (void)timer;
  poll_encoder(&g_enc_l);
  poll_encoder(&g_enc_r);
  return true;
}

static void update_button_state(button_state_t* b, bool raw_level, uint64_t now_ms) {
  b->edge_pressed = false;
  if (raw_level != b->level_raw) {
    b->level_raw = raw_level;
    b->last_change_ms = now_ms;
  }
  if ((now_ms - b->last_change_ms) >= BTN_DEBOUNCE_MS && b->level_stable != b->level_raw) {
    b->level_stable = b->level_raw;
    if (b->level_stable) {
      b->edge_pressed = true;
      b->press_count += 1u;
    }
  }
}

static inline void cs_select(void) { gpio_put(HRDW_PIN_OLED_CS, 0); }
static inline void cs_deselect(void) { gpio_put(HRDW_PIN_OLED_CS, 1); }

static void write_bytes(const uint8_t* data, size_t len) {
  spi_write_blocking(spi0, data, len);
}

static void write_cmd(uint8_t cmd) {
  cs_select();
  gpio_put(HRDW_PIN_OLED_DC, 0);
  write_bytes(&cmd, 1);
  cs_deselect();
}

static void write_data(const uint8_t* data, size_t len) {
  cs_select();
  gpio_put(HRDW_PIN_OLED_DC, 1);
  write_bytes(data, len);
  cs_deselect();
}

static void write_cmd_data(uint8_t cmd, const uint8_t* data, size_t len) {
  cs_select();
  gpio_put(HRDW_PIN_OLED_DC, 0);
  write_bytes(&cmd, 1);
  if (len > 0) {
    gpio_put(HRDW_PIN_OLED_DC, 1);
    write_bytes(data, len);
  }
  cs_deselect();
}

static void tft_reset(void) {
  gpio_put(HRDW_PIN_OLED_RES, 1);
  sleep_ms(20);
  gpio_put(HRDW_PIN_OLED_RES, 0);
  sleep_ms(30);
  gpio_put(HRDW_PIN_OLED_RES, 1);
  sleep_ms(120);
}

static void init_st7735s(void) {
  write_cmd(0x01);
  sleep_ms(120);

  write_cmd(0x11);
  sleep_ms(120);

  {
    const uint8_t frmctr1[] = {0x01, 0x2C, 0x2D};
    write_cmd_data(0xB1, frmctr1, sizeof(frmctr1));
  }
  {
    const uint8_t frmctr2[] = {0x01, 0x2C, 0x2D};
    write_cmd_data(0xB2, frmctr2, sizeof(frmctr2));
  }
  {
    const uint8_t frmctr3[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
    write_cmd_data(0xB3, frmctr3, sizeof(frmctr3));
  }
  {
    const uint8_t invctr = 0x07;
    write_cmd_data(0xB4, &invctr, 1);
  }
  {
    const uint8_t pwctr1[] = {0xA2, 0x02, 0x84};
    write_cmd_data(0xC0, pwctr1, sizeof(pwctr1));
  }
  {
    const uint8_t pwctr2 = 0xC5;
    write_cmd_data(0xC1, &pwctr2, 1);
  }
  {
    const uint8_t pwctr3[] = {0x0A, 0x00};
    write_cmd_data(0xC2, pwctr3, sizeof(pwctr3));
  }
  {
    const uint8_t pwctr4[] = {0x8A, 0x2A};
    write_cmd_data(0xC3, pwctr4, sizeof(pwctr4));
  }
  {
    const uint8_t pwctr5[] = {0x8A, 0xEE};
    write_cmd_data(0xC4, pwctr5, sizeof(pwctr5));
  }
  {
    const uint8_t vmctr1 = 0x0E;
    write_cmd_data(0xC5, &vmctr1, 1);
  }
  {
    const uint8_t colmod = 0x05;
    write_cmd_data(0x3A, &colmod, 1);
  }
  {
    const uint8_t madctl = 0x68;  // rotated 180 deg relative to previous orientation + BGR
    write_cmd_data(0x36, &madctl, 1);
  }

  write_cmd(0x20);
  write_cmd(0x13);
  write_cmd(0x29);
  sleep_ms(100);
}

static void set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  const uint8_t caset[] = {
      (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
      (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF),
  };
  const uint8_t raset[] = {
      (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
      (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF),
  };
  write_cmd_data(0x2A, caset, sizeof(caset));
  write_cmd_data(0x2B, raset, sizeof(raset));
}

static const uint8_t* glyph_for(char c) {
  char glyph_c = c;
  size_t i;

  if (glyph_c >= 'a' && glyph_c <= 'z') {
    glyph_c = (char)(glyph_c - ('a' - 'A'));
  }

  for (i = 0; i < (sizeof(k_font) / sizeof(k_font[0])); ++i) {
    if (k_font[i].ch == glyph_c) return k_font[i].cols;
  }
  for (i = 0; i < (sizeof(k_font) / sizeof(k_font[0])); ++i) {
    if (k_font[i].ch == '?') return k_font[i].cols;
  }
  return k_font[0].cols;
}

static void draw_char(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg) {
  uint8_t px[6 * 8 * 2];
  size_t idx = 0;
  const uint8_t* glyph;
  uint8_t row;
  uint8_t col;

  if (x < 0 || y < 0 || x >= OLED_W || y >= OLED_H) return;

  glyph = glyph_for(c);
  set_addr_window((uint16_t)x, (uint16_t)y, (uint16_t)(x + 5), (uint16_t)(y + 7));
  write_cmd(0x2C);

  for (row = 0; row < 8; ++row) {
    for (col = 0; col < 6; ++col) {
      bool on = false;
      uint16_t color;
      if (row < 7 && col < 5) on = ((glyph[col] >> row) & 0x01u) != 0u;
      color = on ? fg : bg;
      px[idx++] = (uint8_t)(color >> 8);
      px[idx++] = (uint8_t)(color & 0xFF);
    }
  }
  write_data(px, sizeof(px));
}

static void draw_text(int16_t x, int16_t y, const char* text, uint16_t fg, uint16_t bg) {
  int16_t cx = x;
  while (*text != '\0') {
    if ((cx + 6) > OLED_W) break;
    draw_char(cx, y, *text, fg, bg);
    cx += 6;
    ++text;
  }
}

static void display_setup(void) {
  gpio_set_function(HRDW_PIN_OLED_SCK, GPIO_FUNC_SPI);
  gpio_set_function(HRDW_PIN_OLED_MOSI, GPIO_FUNC_SPI);

  gpio_init(HRDW_PIN_OLED_CS);
  gpio_set_dir(HRDW_PIN_OLED_CS, GPIO_OUT);
  gpio_put(HRDW_PIN_OLED_CS, 1);

  gpio_init(HRDW_PIN_OLED_DC);
  gpio_set_dir(HRDW_PIN_OLED_DC, GPIO_OUT);
  gpio_put(HRDW_PIN_OLED_DC, 1);

  gpio_init(HRDW_PIN_OLED_RES);
  gpio_set_dir(HRDW_PIN_OLED_RES, GPIO_OUT);

  gpio_init(HRDW_PIN_OLED_BLK);
  gpio_set_dir(HRDW_PIN_OLED_BLK, GPIO_OUT);
  gpio_put(HRDW_PIN_OLED_BLK, 0);

  spi_init(spi0, 12000000);
  spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  tft_reset();
  init_st7735s();
  gpio_put(HRDW_PIN_OLED_BLK, 1);
}

static void fill_screen(uint16_t color) {
  uint8_t buf[2 * 128];
  uint8_t hi = (uint8_t)(color >> 8);
  uint8_t lo = (uint8_t)(color & 0xFF);
  uint32_t pixels = (uint32_t)OLED_W * (uint32_t)OLED_H;
  size_t i;

  set_addr_window(0, 0, (uint16_t)(OLED_W - 1), (uint16_t)(OLED_H - 1));
  write_cmd(0x2C);

  for (i = 0; i < 128u; ++i) {
    buf[2u * i] = hi;
    buf[2u * i + 1u] = lo;
  }

  while (pixels > 0u) {
    uint32_t chunk = (pixels > 128u) ? 128u : pixels;
    write_data(buf, chunk * 2u);
    pixels -= chunk;
  }
}

static bool detect_mcp4728_address(void) {
  uint8_t addr;
  uint8_t dummy = 0;
  int rc;
  for (addr = MCP4728_I2C_ADDR_DEFAULT; addr <= MCP4728_I2C_ADDR_MAX; ++addr) {
    rc = i2c_read_blocking(i2c0, addr, &dummy, 1, false);
    if (rc == 1) {
      g_dac_i2c_addr = addr;
      return true;
    }
  }
  return false;
}

static bool mcp4728_write_channel(uint8_t channel, uint16_t logical_code_12bit) {
  uint8_t tx[3];
  uint16_t hw_code;
  int written;

  if (channel > 3u) return false;
  if (logical_code_12bit > DAC_CODE_MAX_12BIT) logical_code_12bit = DAC_CODE_MAX_12BIT;

  // Analog output stage on this board is inverted względem kodu logicznego DAC.
  hw_code = (uint16_t)(DAC_CODE_MAX_12BIT - logical_code_12bit);

  tx[0] = (uint8_t)(0x40u | ((channel & 0x03u) << 1u));
  tx[1] = (uint8_t)((hw_code >> 8u) & 0x0Fu);
  tx[2] = (uint8_t)(hw_code & 0xFFu);

  written = i2c_write_blocking(i2c0, g_dac_i2c_addr, tx, 3, false);
  return written == 3;
}

static bool write_all_dac_codes(const uint16_t* codes4) {
  uint8_t ch;
  bool ok = true;
  bool any_changed = false;

  if (codes4 == NULL) return false;

  for (ch = 0; ch < 4u; ++ch) {
    if (!g_dac_cache_valid || codes4[ch] != g_dac_last_codes[ch]) {
      any_changed = true;
      break;
    }
  }

  if (!any_changed) {
    g_dac_diag.skipped_write_calls += 1u;
    g_dac_diag.skipped_channel_writes += 4u;
    return true;
  }

  if (!g_dac_addr_found) {
    g_dac_addr_found = detect_mcp4728_address();
    if (!g_dac_addr_found) g_dac_i2c_addr = MCP4728_I2C_ADDR_DEFAULT;
  }

  for (ch = 0; ch < 4u; ++ch) {
    if (g_dac_cache_valid && codes4[ch] == g_dac_last_codes[ch]) {
      g_dac_diag.skipped_channel_writes += 1u;
      continue;
    }
    ok = mcp4728_write_channel(ch, codes4[ch]) && ok;
    if (ok) {
      g_dac_last_codes[ch] = codes4[ch];
      g_dac_diag.channel_writes += 1u;
    }
  }

  if (ok) g_dac_addr_found = true;
  if (!ok) g_dac_addr_found = false;
  if (ok) {
    g_dac_cache_valid = true;
    g_dac_diag.write_calls += 1u;
  }
  return ok;
}

static uint16_t code_for_output_mv(int32_t mv_out) {
  /* Pre-scaled mapping for PICO_DAC_OPAMP_VOUT path:
   * DAC -> inverting op-amp (Rin=33k, Rf=100k, V_bias on non-inverting input).
   * Values are logical DAC codes (before hw polarity inversion in mcp4728_write_channel).
   */
  static const int32_t kOutMv[5] = {-7000, -3000, 0, 3000, 6000};
  static const uint16_t kCode[5] = {0, 1229, 2168, 3107, 4042};
  uint8_t i;

  if (mv_out <= -3000) mv_out = -3000;
  if (mv_out >= 6000) mv_out = 6000;

  if (mv_out <= kOutMv[0]) return kCode[0];
  if (mv_out >= kOutMv[4]) return kCode[4];

  for (i = 0; i < 4; ++i) {
    int32_t x0 = kOutMv[i];
    int32_t x1 = kOutMv[i + 1];
    uint16_t y0 = kCode[i];
    uint16_t y1 = kCode[i + 1];
    int32_t den = x1 - x0;
    int64_t num;
    int64_t delta;
    int64_t y;

    if (mv_out < x0 || mv_out > x1) continue;

    num = (int64_t)(mv_out - x0) * ((int32_t)y1 - (int32_t)y0);
    if (num >= 0) {
      delta = (num + (den / 2)) / den;
    } else {
      delta = (num - (den / 2)) / den;
    }
    y = (int64_t)y0 + delta;
    if (y < 0) y = 0;
    if (y > 4095) y = 4095;
    return (uint16_t)y;
  }

  return 2168u;
}

void hal_io_init(void) {
  uint64_t now_ms = to_ms_since_boot(get_absolute_time());

  gpio_init(HRDW_PIN_TR1_IN);
  gpio_set_dir(HRDW_PIN_TR1_IN, GPIO_IN);
  gpio_pull_up(HRDW_PIN_TR1_IN);

  gpio_init(HRDW_PIN_TR2_IN);
  gpio_set_dir(HRDW_PIN_TR2_IN, GPIO_IN);
  gpio_pull_up(HRDW_PIN_TR2_IN);

  gpio_init(HRDW_PIN_TR3_IN);
  gpio_set_dir(HRDW_PIN_TR3_IN, GPIO_IN);
  gpio_pull_up(HRDW_PIN_TR3_IN);

  gpio_init(HRDW_PIN_TR4_IN);
  gpio_set_dir(HRDW_PIN_TR4_IN, GPIO_IN);
  gpio_pull_up(HRDW_PIN_TR4_IN);

  g_enc_l.pin_a = HRDW_PIN_ENC_L_A;
  g_enc_l.pin_b = HRDW_PIN_ENC_L_B;
  g_enc_l.invert_direction = true;
  g_enc_r.pin_a = HRDW_PIN_ENC_R_A;
  g_enc_r.pin_b = HRDW_PIN_ENC_R_B;
  g_enc_r.invert_direction = false;

  gpio_init(g_enc_l.pin_a);
  gpio_set_dir(g_enc_l.pin_a, GPIO_IN);
  gpio_pull_up(g_enc_l.pin_a);
  gpio_init(g_enc_l.pin_b);
  gpio_set_dir(g_enc_l.pin_b, GPIO_IN);
  gpio_pull_up(g_enc_l.pin_b);
  gpio_init(g_enc_r.pin_a);
  gpio_set_dir(g_enc_r.pin_a, GPIO_IN);
  gpio_pull_up(g_enc_r.pin_a);
  gpio_init(g_enc_r.pin_b);
  gpio_set_dir(g_enc_r.pin_b, GPIO_IN);
  gpio_pull_up(g_enc_r.pin_b);

  g_enc_l.pin_state_a = 0xFFu;
  g_enc_l.pin_state_b = 0xFFu;
  g_enc_r.pin_state_a = 0xFFu;
  g_enc_r.pin_state_b = 0xFFu;
  add_repeating_timer_us(-1000, encoder_poll_timer_cb, NULL, &g_encoder_poll_timer);

  g_buttons[HAL_IO_BTN_ENC_L].pin = HRDW_PIN_ENC_L_SW;
  g_buttons[HAL_IO_BTN_ENC_R].pin = HRDW_PIN_ENC_R_SW;
  g_buttons[HAL_IO_BTN_SW1].pin = HRDW_PIN_SW1;
  g_buttons[HAL_IO_BTN_SW2].pin = HRDW_PIN_SW2;

  for (size_t i = 0; i < HAL_IO_BTN_COUNT; ++i) {
    gpio_init(g_buttons[i].pin);
    gpio_set_dir(g_buttons[i].pin, GPIO_IN);
    gpio_pull_up(g_buttons[i].pin);
    g_buttons[i].level_raw = is_pressed(g_buttons[i].pin);
    g_buttons[i].level_stable = g_buttons[i].level_raw;
    g_buttons[i].last_change_ms = now_ms;
  }

  // MCP4728/I2C bounds timing accuracy; firmware minimizes jitter with IRQ
  // timestamps, cached output states and batched DAC writes. 400 kHz needs
  // stable MCP4728 operation and appropriate pull-ups; set this back to
  // 100000u if the bus proves marginal on a build.
  i2c_init(i2c0, DAC_I2C_BAUDRATE);
  gpio_set_function(HRDW_PIN_I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(HRDW_PIN_I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(HRDW_PIN_I2C_SDA);
  gpio_pull_up(HRDW_PIN_I2C_SCL);

  display_setup();
  fill_screen(COLOR_BLACK);
}

void hal_io_poll(uint64_t now_ms) {
  for (size_t i = 0; i < HAL_IO_BTN_COUNT; ++i) {
    update_button_state(&g_buttons[i], is_pressed(g_buttons[i].pin), now_ms);
  }
}

int32_t hal_io_encoder_count(hal_io_encoder_t enc) {
  return (enc == HAL_IO_ENC_L) ? g_enc_l.count : g_enc_r.count;
}

uint32_t hal_io_encoder_inc_events(hal_io_encoder_t enc) {
  return (enc == HAL_IO_ENC_L) ? g_enc_l.inc_events : g_enc_r.inc_events;
}

uint32_t hal_io_encoder_dec_events(hal_io_encoder_t enc) {
  return (enc == HAL_IO_ENC_L) ? g_enc_l.dec_events : g_enc_r.dec_events;
}

bool hal_io_button_pressed(hal_io_button_t btn) {
  if ((int)btn < 0 || btn >= HAL_IO_BTN_COUNT) return false;
  return g_buttons[btn].level_stable;
}

bool hal_io_button_edge_pressed(hal_io_button_t btn) {
  bool edge;
  if ((int)btn < 0 || btn >= HAL_IO_BTN_COUNT) return false;
  edge = g_buttons[btn].edge_pressed;
  g_buttons[btn].edge_pressed = false;
  return edge;
}

uint32_t hal_io_button_press_count(hal_io_button_t btn) {
  if ((int)btn < 0 || btn >= HAL_IO_BTN_COUNT) return 0;
  return g_buttons[btn].press_count;
}

bool hal_io_trigger_active(hal_io_trigger_t tr) {
  uint8_t pin;
  switch (tr) {
    case HAL_IO_TR1:
      pin = HRDW_PIN_TR1_IN;
      break;
    case HAL_IO_TR2:
      pin = HRDW_PIN_TR2_IN;
      break;
    case HAL_IO_TR3:
      return false;
    case HAL_IO_TR4:
      return false;
    default:
      return false;
  }
  // Input logic is inverted in hardware: low level means active trigger high.
  return !gpio_get(pin);
}

bool hal_io_dac_set_all_mv(int32_t millivolts) {
  uint16_t code = code_for_output_mv(millivolts);
  uint16_t codes[4] = {code, code, code, code};
  return write_all_dac_codes(codes);
}

bool hal_io_dac_set_channels_mv(const int32_t millivolts_4[4]) {
  uint16_t codes[4];
  if (millivolts_4 == NULL) return false;
  codes[0] = code_for_output_mv(millivolts_4[0]);
  codes[1] = code_for_output_mv(millivolts_4[1]);
  codes[2] = code_for_output_mv(millivolts_4[2]);
  codes[3] = code_for_output_mv(millivolts_4[3]);
  return write_all_dac_codes(codes);
}

bool hal_io_dac_set_channels_code(const uint16_t codes_4[4]) {
  if (codes_4 == NULL) return false;
  return write_all_dac_codes(codes_4);
}

void hal_io_dac_get_diag(hal_io_dac_diag_t* diag) {
  if (diag == NULL) return;
  *diag = g_dac_diag;
}

void hal_io_oled_clear(void) {
  fill_screen(COLOR_BLACK);
}

void hal_io_oled_fill_rect_color(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  uint8_t hi = (uint8_t)(color >> 8);
  uint8_t lo = (uint8_t)(color & 0xFFu);
  uint8_t px[2 * 64];
  uint16_t x0 = x;
  uint16_t y0 = y;
  uint16_t x1;
  uint16_t y1;
  uint16_t row_w;
  uint16_t row_h;
  size_t i;

  if (w == 0u || h == 0u) return;
  if (x0 >= OLED_W || y0 >= OLED_H) return;

  x1 = (uint16_t)(x0 + w - 1u);
  y1 = (uint16_t)(y0 + h - 1u);
  if (x1 >= OLED_W) x1 = OLED_W - 1u;
  if (y1 >= OLED_H) y1 = OLED_H - 1u;

  row_w = (uint16_t)(x1 - x0 + 1u);
  row_h = (uint16_t)(y1 - y0 + 1u);

  for (i = 0; i < 64u; ++i) {
    px[2u * i] = hi;
    px[2u * i + 1u] = lo;
  }

  set_addr_window(x0, y0, x1, y1);
  write_cmd(0x2C);

  for (uint16_t r = 0u; r < row_h; ++r) {
    uint16_t remain = row_w;
    while (remain > 0u) {
      uint16_t chunk = (remain > 64u) ? 64u : remain;
      write_data(px, (size_t)(chunk * 2u));
      remain = (uint16_t)(remain - chunk);
    }
  }
}

void hal_io_oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool white) {
  hal_io_oled_fill_rect_color(x, y, w, h, white ? COLOR_WHITE : COLOR_BLACK);
}

void hal_io_oled_draw_rect_color(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  if (w == 0u || h == 0u) return;

  hal_io_oled_fill_rect_color(x, y, w, 1u, color);
  if (h > 1u) {
    hal_io_oled_fill_rect_color(x, (uint8_t)(y + h - 1u), w, 1u, color);
  }
  if (h > 2u) {
    hal_io_oled_fill_rect_color(x, (uint8_t)(y + 1u), 1u, (uint8_t)(h - 2u), color);
    if (w > 1u) {
      hal_io_oled_fill_rect_color((uint8_t)(x + w - 1u), (uint8_t)(y + 1u), 1u, (uint8_t)(h - 2u), color);
    }
  }
}

void hal_io_oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool white) {
  hal_io_oled_draw_rect_color(x, y, w, h, white ? COLOR_WHITE : COLOR_BLACK);
}

void hal_io_oled_draw_pixel(uint8_t x, uint8_t y, bool white) {
  hal_io_oled_fill_rect(x, y, 1u, 1u, white);
}

void hal_io_oled_draw_line_color(uint8_t row, const char* text, uint16_t fg, uint16_t bg) {
  char line[27];
  int16_t y = (int16_t)(row * 8u);

  if (row >= (OLED_H / 8u)) return;
  snprintf(line, sizeof(line), "%-26.26s", text);
  draw_text(0, y, line, fg, bg);
}

void hal_io_oled_draw_line(uint8_t row, const char* text, bool inverted) {
  uint16_t fg = inverted ? COLOR_BLACK : COLOR_WHITE;
  uint16_t bg = inverted ? COLOR_WHITE : COLOR_BLACK;
  hal_io_oled_draw_line_color(row, text, fg, bg);
}

void hal_io_oled_draw_text(uint8_t x, uint8_t y, const char* text, bool inverted) {
  uint16_t fg = inverted ? COLOR_BLACK : COLOR_WHITE;
  uint16_t bg = inverted ? COLOR_WHITE : COLOR_BLACK;
  draw_text((int16_t)x, (int16_t)y, text, fg, bg);
}
