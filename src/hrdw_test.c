#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_settings.h"
#include "calibration.h"
#include "grids_engine.h"
#include "hal_io.h"
#include "hal_mux_adc.h"
#include "pico/stdlib.h"
#include "trigseq_engine.h"

#define DRAW_PERIOD_MS 120u
#define LOOP_SLEEP_MS 1u
#define DAC_STEP_MV 500
#define DAC_MIN_MV -3000
#define DAC_MAX_MV 6000

#define GRIDS_TRIG_HIGH_MV 5000
#define GRIDS_TRIG_LOW_MV 0
#define GRIDS_TRIG_PULSE_MS 15u
#define GRIDS_BPM_MIN 30
#define GRIDS_BPM_MAX 300
#define TRIGSEQ_TRIG_HIGH_MV 5000
#define TRIGSEQ_TRIG_LOW_MV 0
#define TRIGSEQ_TRIG_PULSE_MS 15u
#define TRIGSEQ_BPM_MIN 30
#define TRIGSEQ_BPM_MAX 300
#define EUCLID_TRIG_HIGH_MV 5000
#define EUCLID_TRIG_LOW_MV 0
#define EUCLID_TRIG_PULSE_MS 15u
#define EUCLID_BPM_MIN 30
#define EUCLID_BPM_MAX 300

typedef enum {
  APP_MENU = 0,
  APP_HRDW_TEST = 1,
  APP_CALIBRATION = 2,
  APP_GRIDS = 3,
  APP_TRIGSEQ = 4,
  APP_EUCLID = 5,
} app_mode_t;

typedef enum {
  CAL_PARAM_CHANNEL = 0,
  CAL_PARAM_POINT = 1,
  CAL_PARAM_CODE = 2,
  CAL_PARAM_COUNT = 3,
} cal_param_t;

typedef enum {
  GRIDS_PARAM_CLOCK = 0,
  GRIDS_PARAM_BPM = 1,
  GRIDS_PARAM_MAP_X = 2,
  GRIDS_PARAM_MAP_Y = 3,
  GRIDS_PARAM_CHAOS = 4,
  GRIDS_PARAM_COUNT = 5,
} grids_param_t;

typedef enum {
  GRIDS_CLOCK_INT = 0,
  GRIDS_CLOCK_EXT = 1,
} grids_clock_t;

typedef enum {
  TRIGSEQ_LEN_4X16 = 0,
  TRIGSEQ_LEN_2X32 = 1,
  TRIGSEQ_LEN_1X64 = 2,
  TRIGSEQ_LEN_COUNT = 3,
} trigseq_len_mode_t;

typedef enum {
  TRIGSEQ_CLOCK_INT = 0,
  TRIGSEQ_CLOCK_EXT = 1,
} trigseq_clock_t;

typedef enum {
  TRIGSEQ_FOCUS_GRID = 0,
  TRIGSEQ_FOCUS_MENU = 1,
} trigseq_focus_t;

typedef enum {
  TRIGSEQ_PARAM_LEN = 0,
  TRIGSEQ_PARAM_CLOCK = 1,
  TRIGSEQ_PARAM_BPM = 2,
  TRIGSEQ_PARAM_RUN = 3,
  TRIGSEQ_PARAM_COUNT = 4,
} trigseq_param_t;

typedef enum {
  EUCLID_CLOCK_INT = 0,
  EUCLID_CLOCK_EXT = 1,
} euclid_clock_t;

typedef enum {
  EUCLID_FOCUS_GRID = 0,
  EUCLID_FOCUS_MENU = 1,
} euclid_focus_t;

typedef enum {
  EUCLID_PARAM_CLOCK = 0,
  EUCLID_PARAM_BPM = 1,
  EUCLID_PARAM_CH1_STEPS = 2,
  EUCLID_PARAM_CH1_HITS = 3,
  EUCLID_PARAM_CH2_STEPS = 4,
  EUCLID_PARAM_CH2_HITS = 5,
  EUCLID_PARAM_CH3_STEPS = 6,
  EUCLID_PARAM_CH3_HITS = 7,
  EUCLID_PARAM_CH4_STEPS = 8,
  EUCLID_PARAM_CH4_HITS = 9,
  EUCLID_PARAM_COUNT = 10,
} euclid_param_t;

typedef struct {
  uint16_t cv_raw[4];
  int32_t cv_mv[4];
  uint16_t mux_raw_manual;
  int32_t mux_mv_manual;
  int32_t dac_mv[4];
  int selected_dac;
  int selected_mux_ch;
  bool mux_manual_mode;
  bool dac_ok;
} hrdw_test_state_t;

typedef struct {
  grids_engine_t engine;
  grids_param_t selected_param;
  grids_clock_t clock;
  int bpm;
  uint8_t map_x;
  uint8_t map_y;
  uint8_t chaos;
  uint16_t fill_raw[4];
  int32_t fill_mv[4];
  uint8_t fill_u8[4];
  bool trig_state[4];
  uint64_t trig_off_ms[4];
  uint64_t next_int_tick_ms;
  uint32_t step_count;
  bool prev_clk_active;
  bool prev_rst_active;
  uint8_t last_out_mask;
  bool preview_cache_valid;
  bool preview_cache_bits[4][32];
  bool preview_cache_progress[4][32];
  bool outputs_ok;
  uint64_t status_until_ms;
  char status[16];
} grids_state_t;

typedef struct {
  trigseq_engine_t engine;
  bool engine_initialized;
  trigseq_len_mode_t len_mode;
  uint8_t cursor_step;
  trigseq_clock_t clock;
  int bpm;
  uint64_t next_int_tick_ms;
  trigseq_focus_t focus;
  trigseq_param_t selected_param;
  bool grid_cache_valid;
  trigseq_len_mode_t grid_cache_mode;
  uint8_t grid_cache_cursor;
  bool grid_cache_bits[64];
  bool grid_cache_progress[64];
  uint8_t step16[4];
  uint8_t step32[2];
  uint8_t step64;
  bool run;
  bool trig_state[4];
  uint64_t trig_off_ms[4];
  bool prev_clk_active;
  bool prev_rst_active;
  uint32_t step_count;
  uint8_t last_out_mask;
  bool outputs_ok;
  uint64_t status_until_ms;
  char status[16];
} trigseq_state_t;

typedef struct {
  euclid_clock_t clock;
  int bpm;
  uint8_t steps[4];
  uint8_t hits[4];
  uint8_t phase[4];
  uint64_t next_int_tick_ms;
  euclid_focus_t focus;
  euclid_param_t selected_param;
  bool grid_cache_valid;
  bool grid_cache_bits[64];
  bool grid_cache_progress[64];
  bool trig_state[4];
  uint64_t trig_off_ms[4];
  bool prev_clk_active;
  bool prev_rst_active;
  uint32_t step_count;
  uint8_t last_out_mask;
  bool outputs_ok;
  uint64_t status_until_ms;
  char status[16];
} euclid_state_t;

static const char* k_menu_items[5] = {"HRDW_TEST", "CALIBRATION", "GRIDS", "TRIGSEQ", "4XEUCLID"};

static app_mode_t g_app_mode = APP_MENU;
static int g_menu_index = 0;

static calibration_data_t g_calibration_data;
static bool g_calibration_loaded = false;
static bool g_calibration_dirty = false;
static app_settings_data_t g_app_settings_data;
static bool g_app_settings_loaded = false;

static cal_param_t g_cal_param = CAL_PARAM_CHANNEL;
static uint8_t g_cal_channel = 0;
static cal_point_t g_cal_point = CAL_POINT_0;

static int32_t g_last_enc_l = 0;
static int32_t g_last_enc_r = 0;

static uint64_t g_cal_status_until_ms = 0;
static char g_cal_status[16] = {0};

static hrdw_test_state_t g_hrdw = {
    .dac_mv = {1000, 1000, 1000, 1000},
    .selected_dac = 0,
    .selected_mux_ch = 0,
    .mux_manual_mode = false,
    .dac_ok = false,
};

static grids_state_t g_grids = {
    .selected_param = GRIDS_PARAM_CLOCK,
    .clock = GRIDS_CLOCK_EXT,
    .bpm = 120,
    .map_x = 128,
    .map_y = 128,
    .chaos = 64,
    .last_out_mask = 0xFFu,
    .preview_cache_valid = false,
    .preview_cache_bits = {{false}},
    .preview_cache_progress = {{false}},
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static trigseq_state_t g_trigseq = {
    .engine_initialized = false,
    .len_mode = TRIGSEQ_LEN_4X16,
    .cursor_step = 0u,
    .clock = TRIGSEQ_CLOCK_EXT,
    .bpm = 120,
    .next_int_tick_ms = 0u,
    .focus = TRIGSEQ_FOCUS_MENU,
    .selected_param = TRIGSEQ_PARAM_LEN,
    .grid_cache_valid = false,
    .grid_cache_mode = TRIGSEQ_LEN_4X16,
    .grid_cache_cursor = 0u,
    .grid_cache_bits = {false},
    .grid_cache_progress = {false},
    .step16 = {0u, 0u, 0u, 0u},
    .step32 = {0u, 0u},
    .step64 = 0u,
    .run = true,
    .last_out_mask = 0xFFu,
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static euclid_state_t g_euclid = {
    .clock = EUCLID_CLOCK_EXT,
    .bpm = 120,
    .steps = {16u, 16u, 16u, 16u},
    .hits = {4u, 6u, 8u, 10u},
    .phase = {0u, 0u, 0u, 0u},
    .next_int_tick_ms = 0u,
    .focus = EUCLID_FOCUS_MENU,
    .selected_param = EUCLID_PARAM_CLOCK,
    .grid_cache_valid = false,
    .grid_cache_bits = {false},
    .grid_cache_progress = {false},
    .last_out_mask = 0xFFu,
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static int32_t clamp_mv(int32_t mv) {
  if (mv < DAC_MIN_MV) return DAC_MIN_MV;
  if (mv > DAC_MAX_MV) return DAC_MAX_MV;
  return mv;
}

static int clamp_i(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint8_t clamp_u8i(int x) {
  if (x < 0) return 0u;
  if (x > 255) return 255u;
  return (uint8_t)x;
}

static uint8_t trigseq_len_for_mode(trigseq_len_mode_t mode) {
  if (mode == TRIGSEQ_LEN_2X32) return 32u;
  if (mode == TRIGSEQ_LEN_1X64) return 64u;
  return 16u;
}

static trigseq_len_mode_t trigseq_mode_from_length(uint8_t length) {
  if (length >= 64u) return TRIGSEQ_LEN_1X64;
  if (length >= 32u) return TRIGSEQ_LEN_2X32;
  return TRIGSEQ_LEN_4X16;
}

static const char* trigseq_mode_label(trigseq_len_mode_t mode) {
  if (mode == TRIGSEQ_LEN_2X32) return "2X32";
  if (mode == TRIGSEQ_LEN_1X64) return "1X64";
  return "4X16";
}

static uint32_t trigseq_int_interval_ms(void) {
  int bpm = clamp_i(g_trigseq.bpm, TRIGSEQ_BPM_MIN, TRIGSEQ_BPM_MAX);
  return (uint32_t)(60000 / bpm / 4);
}

static void trigseq_set_clock_source(trigseq_clock_t source, uint64_t now_ms) {
  g_trigseq.clock = source;
  if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
    g_trigseq.next_int_tick_ms = now_ms + trigseq_int_interval_ms();
  }
}

static void trigseq_invalidate_grid_cache(void) {
  g_trigseq.grid_cache_valid = false;
}

static bool trigseq_cell_get(uint8_t row, uint8_t col) {
  uint64_t mask;
  if (row >= 4u || col >= 16u) return false;
  mask = (uint64_t)1u << col;
  return (g_trigseq.engine.pattern[row] & mask) != 0u;
}

static void trigseq_cell_set(uint8_t row, uint8_t col, bool on) {
  uint64_t mask;
  if (row >= 4u || col >= 16u) return;
  mask = (uint64_t)1u << col;
  if (on) {
    g_trigseq.engine.pattern[row] |= mask;
  } else {
    g_trigseq.engine.pattern[row] &= ~mask;
  }
}

static bool trigseq_grid_get_bit(uint8_t step) {
  uint8_t row = (uint8_t)(step / 16u);
  uint8_t col = (uint8_t)(step % 16u);
  return trigseq_cell_get(row, col);
}

static void trigseq_grid_set_bit(uint8_t step, bool on) {
  uint8_t row = (uint8_t)(step / 16u);
  uint8_t col = (uint8_t)(step % 16u);
  trigseq_cell_set(row, col, on);
}

static bool trigseq_is_progress_step(uint8_t step) {
  uint8_t row = (uint8_t)(step / 16u);
  uint8_t col = (uint8_t)(step % 16u);

  if (g_trigseq.len_mode == TRIGSEQ_LEN_4X16) {
    return col == (uint8_t)((g_trigseq.step16[row] + 15u) & 0x0Fu);
  }

  if (g_trigseq.len_mode == TRIGSEQ_LEN_2X32) {
    if (row < 2u) {
      uint8_t s = (uint8_t)((g_trigseq.step32[0] + 31u) & 0x1Fu);
      if (s < 16u) return row == 0u && col == s;
      return row == 1u && col == (uint8_t)(s - 16u);
    } else {
      uint8_t s = (uint8_t)((g_trigseq.step32[1] + 31u) & 0x1Fu);
      if (s < 16u) return row == 2u && col == s;
      return row == 3u && col == (uint8_t)(s - 16u);
    }
  }

  return row == (uint8_t)(((g_trigseq.step64 + 63u) & 0x3Fu) / 16u) &&
         col == (uint8_t)(((g_trigseq.step64 + 63u) & 0x3Fu) % 16u);
}

static void format_cv_line(char* out, size_t out_sz, uint8_t ch0, int32_t mv0, uint8_t ch1, int32_t mv1) {
  snprintf(out, out_sz, "CV%u %+1ld.%03ld CV%u %+1ld.%03ld", (unsigned)ch0, (long)(mv0 / 1000),
           (long)((mv0 < 0 ? -mv0 : mv0) % 1000), (unsigned)ch1, (long)(mv1 / 1000),
           (long)((mv1 < 0 ? -mv1 : mv1) % 1000));
}

static const char* point_label(cal_point_t p) {
  if (p == CAL_POINT_NEG3) return "-3V";
  if (p == CAL_POINT_0) return "0V";
  if (p == CAL_POINT_POS3) return "+3V";
  return "+6V";
}

static void clear_rows(uint8_t first_row) {
  for (uint8_t row = first_row; row < 16u; ++row) {
    hal_io_oled_draw_line(row, "", false);
  }
}

static void set_cal_status(const char* s) {
  snprintf(g_cal_status, sizeof(g_cal_status), "%s", s);
  g_cal_status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static void set_grids_status(const char* s) {
  snprintf(g_grids.status, sizeof(g_grids.status), "%s", s);
  g_grids.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static void set_trigseq_status(const char* s) {
  snprintf(g_trigseq.status, sizeof(g_trigseq.status), "%s", s);
  g_trigseq.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static void set_euclid_status(const char* s) {
  snprintf(g_euclid.status, sizeof(g_euclid.status), "%s", s);
  g_euclid.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static uint16_t app_code_from_mv(int32_t mv, uint8_t ch) {
  mv = clamp_mv(mv);
  return calibration_code_for_voltage_mv(&g_calibration_data, mv, ch);
}

static bool app_write_outputs_mv(const int32_t* mv4) {
  uint16_t codes[4];

  if (mv4 == NULL) return false;

  for (int ch = 0; ch < 4; ++ch) {
    codes[ch] = app_code_from_mv(mv4[ch], (uint8_t)ch);
  }
  return hal_io_dac_set_channels_code(codes);
}

static bool apply_calibration_preview(void) {
  uint16_t codes[4];
  uint16_t selected = calibration_get_code(&g_calibration_data, g_cal_point, g_cal_channel);

  for (uint8_t ch = 0; ch < 4u; ++ch) {
    if (ch == g_cal_channel) {
      codes[ch] = selected;
    } else {
      codes[ch] = app_code_from_mv(0, ch);
    }
  }

  return hal_io_dac_set_channels_code(codes);
}

static void set_encoder_reference_now(void) {
  g_last_enc_l = hal_io_encoder_count(HAL_IO_ENC_L);
  g_last_enc_r = hal_io_encoder_count(HAL_IO_ENC_R);
}

static uint8_t build_grids_mask(void) {
  uint8_t mask = 0u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    if (g_grids.trig_state[i]) mask |= (uint8_t)(1u << i);
  }
  return mask;
}

static void grids_apply_outputs(bool force) {
  uint8_t mask = build_grids_mask();
  int32_t out_mv[4];

  if (!force && mask == g_grids.last_out_mask) return;

  for (uint8_t i = 0u; i < 4u; ++i) {
    out_mv[i] = g_grids.trig_state[i] ? GRIDS_TRIG_HIGH_MV : GRIDS_TRIG_LOW_MV;
  }

  g_grids.outputs_ok = app_write_outputs_mv(out_mv);
  g_grids.last_out_mask = mask;
}

static uint8_t build_trigseq_mask(void) {
  uint8_t mask = 0u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    if (g_trigseq.trig_state[i]) mask |= (uint8_t)(1u << i);
  }
  return mask;
}

static void trigseq_apply_outputs(bool force) {
  uint8_t mask = build_trigseq_mask();
  int32_t out_mv[4];

  if (!force && mask == g_trigseq.last_out_mask) return;

  for (uint8_t i = 0u; i < 4u; ++i) {
    out_mv[i] = g_trigseq.trig_state[i] ? TRIGSEQ_TRIG_HIGH_MV : TRIGSEQ_TRIG_LOW_MV;
  }

  g_trigseq.outputs_ok = app_write_outputs_mv(out_mv);
  g_trigseq.last_out_mask = mask;
}

static void trigseq_reset_outputs_and_state(void) {
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_trigseq.trig_state[i] = false;
    g_trigseq.trig_off_ms[i] = 0u;
  }
  g_trigseq.last_out_mask = 0xFFu;
  trigseq_apply_outputs(true);
}

static void trigseq_update_pulses(uint64_t now_ms) {
  bool changed = false;
  for (uint8_t i = 0u; i < 4u; ++i) {
    if (g_trigseq.trig_state[i] && now_ms >= g_trigseq.trig_off_ms[i]) {
      g_trigseq.trig_state[i] = false;
      changed = true;
    }
  }
  if (changed) trigseq_apply_outputs(false);
}

static void trigseq_do_step(uint64_t now_ms) {
  bool trig[4] = {false, false, false, false};

  if (g_trigseq.len_mode == TRIGSEQ_LEN_4X16) {
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
      uint8_t s = g_trigseq.step16[ch];
      trig[ch] = trigseq_cell_get(ch, s);
      g_trigseq.step16[ch] = (uint8_t)((s + 1u) & 0x0Fu);
    }
  } else if (g_trigseq.len_mode == TRIGSEQ_LEN_2X32) {
    uint8_t s0 = g_trigseq.step32[0];
    uint8_t s1 = g_trigseq.step32[1];
    bool hit0;
    bool hit1;

    hit0 = (s0 < 16u) ? trigseq_cell_get(0u, s0) : trigseq_cell_get(1u, (uint8_t)(s0 - 16u));
    hit1 = (s1 < 16u) ? trigseq_cell_get(2u, s1) : trigseq_cell_get(3u, (uint8_t)(s1 - 16u));
    trig[0] = hit0;
    trig[2] = hit0;
    trig[1] = hit1;
    trig[3] = hit1;

    g_trigseq.step32[0] = (uint8_t)((s0 + 1u) & 0x1Fu);
    g_trigseq.step32[1] = (uint8_t)((s1 + 1u) & 0x1Fu);
  } else {
    uint8_t s = g_trigseq.step64;
    bool hit = trigseq_cell_get((uint8_t)(s / 16u), (uint8_t)(s % 16u));
    trig[0] = hit;
    trig[1] = hit;
    trig[2] = hit;
    trig[3] = hit;
    g_trigseq.step64 = (uint8_t)((s + 1u) & 0x3Fu);
  }

  for (uint8_t i = 0u; i < 4u; ++i) {
    if (trig[i]) {
      // Keep pulse width constant: do not extend an already active pulse.
      if (!g_trigseq.trig_state[i]) {
        g_trigseq.trig_state[i] = true;
        g_trigseq.trig_off_ms[i] = now_ms + TRIGSEQ_TRIG_PULSE_MS;
      }
    }
  }

  g_trigseq.step_count += 1u;
  trigseq_apply_outputs(false);
}

static void trigseq_reset_engine(void) {
  trigseq_engine_reset(&g_trigseq.engine);
  g_trigseq.step16[0] = 0u;
  g_trigseq.step16[1] = 0u;
  g_trigseq.step16[2] = 0u;
  g_trigseq.step16[3] = 0u;
  g_trigseq.step32[0] = 0u;
  g_trigseq.step32[1] = 0u;
  g_trigseq.step64 = 0u;
  g_trigseq.step_count = 0u;
}

static void trigseq_enter(void) {
  uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  if (!g_trigseq.engine_initialized) {
    trigseq_engine_init(&g_trigseq.engine, (uint32_t)to_us_since_boot(get_absolute_time()));
    g_trigseq.engine_initialized = true;
  }
  g_trigseq.run = true;
  g_trigseq.focus = TRIGSEQ_FOCUS_MENU;
  g_trigseq.selected_param = TRIGSEQ_PARAM_LEN;
  g_trigseq.cursor_step = 0u;
  trigseq_reset_engine();
  if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
    g_trigseq.next_int_tick_ms = now_ms + trigseq_int_interval_ms();
  }
  g_trigseq.prev_clk_active = hal_io_trigger_active(HAL_IO_TR1);
  g_trigseq.prev_rst_active = hal_io_trigger_active(HAL_IO_TR2);
  trigseq_reset_outputs_and_state();
  trigseq_invalidate_grid_cache();
}

static uint32_t euclid_int_interval_ms(void) {
  int bpm = clamp_i(g_euclid.bpm, EUCLID_BPM_MIN, EUCLID_BPM_MAX);
  return (uint32_t)(60000 / bpm / 4);
}

static void euclid_set_clock_source(euclid_clock_t source, uint64_t now_ms) {
  g_euclid.clock = source;
  if (g_euclid.clock == EUCLID_CLOCK_INT) {
    g_euclid.next_int_tick_ms = now_ms + euclid_int_interval_ms();
  }
}

static bool euclid_step_is_hit(uint8_t steps, uint8_t hits, uint8_t step) {
  if (steps == 0u) return false;
  if (hits == 0u) return false;
  if (hits >= steps) return true;
  return (uint8_t)(((uint16_t)step * (uint16_t)hits) % (uint16_t)steps) < hits;
}

static bool euclid_grid_get_bit(uint8_t linear_step) {
  uint8_t row = (uint8_t)(linear_step / 16u);
  uint8_t col = (uint8_t)(linear_step % 16u);
  if (row >= 4u) return false;
  if (col >= g_euclid.steps[row]) return false;
  return euclid_step_is_hit(g_euclid.steps[row], g_euclid.hits[row], col);
}

static bool euclid_is_progress_step(uint8_t linear_step) {
  uint8_t row = (uint8_t)(linear_step / 16u);
  uint8_t col = (uint8_t)(linear_step % 16u);
  uint8_t prev;
  if (row >= 4u) return false;
  if (g_euclid.steps[row] == 0u) return false;
  prev = (uint8_t)((g_euclid.phase[row] + g_euclid.steps[row] - 1u) % g_euclid.steps[row]);
  return col == prev;
}

static uint8_t build_euclid_mask(void) {
  uint8_t mask = 0u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    if (g_euclid.trig_state[i]) mask |= (uint8_t)(1u << i);
  }
  return mask;
}

static void euclid_apply_outputs(bool force) {
  uint8_t mask = build_euclid_mask();
  int32_t out_mv[4];

  if (!force && mask == g_euclid.last_out_mask) return;

  for (uint8_t i = 0u; i < 4u; ++i) {
    out_mv[i] = g_euclid.trig_state[i] ? EUCLID_TRIG_HIGH_MV : EUCLID_TRIG_LOW_MV;
  }

  g_euclid.outputs_ok = app_write_outputs_mv(out_mv);
  g_euclid.last_out_mask = mask;
}

static void euclid_reset_outputs_and_state(void) {
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_euclid.trig_state[i] = false;
    g_euclid.trig_off_ms[i] = 0u;
  }
  g_euclid.last_out_mask = 0xFFu;
  euclid_apply_outputs(true);
}

static void euclid_update_pulses(uint64_t now_ms) {
  bool changed = false;
  for (uint8_t i = 0u; i < 4u; ++i) {
    if (g_euclid.trig_state[i] && now_ms >= g_euclid.trig_off_ms[i]) {
      g_euclid.trig_state[i] = false;
      changed = true;
    }
  }
  if (changed) euclid_apply_outputs(false);
}

static void euclid_do_step(uint64_t now_ms) {
  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    uint8_t step = g_euclid.phase[ch];
    if (euclid_step_is_hit(g_euclid.steps[ch], g_euclid.hits[ch], step)) {
      // Keep pulse width constant: do not extend an already active pulse.
      if (!g_euclid.trig_state[ch]) {
        g_euclid.trig_state[ch] = true;
        g_euclid.trig_off_ms[ch] = now_ms + EUCLID_TRIG_PULSE_MS;
      }
    }
    if (g_euclid.steps[ch] > 0u) {
      g_euclid.phase[ch] = (uint8_t)((step + 1u) % g_euclid.steps[ch]);
    }
  }
  g_euclid.step_count += 1u;
  euclid_apply_outputs(false);
}

static void euclid_reset_engine(void) {
  g_euclid.phase[0] = 0u;
  g_euclid.phase[1] = 0u;
  g_euclid.phase[2] = 0u;
  g_euclid.phase[3] = 0u;
  g_euclid.step_count = 0u;
}

static void euclid_enter(void) {
  uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  g_euclid.focus = EUCLID_FOCUS_MENU;
  g_euclid.selected_param = EUCLID_PARAM_CLOCK;
  euclid_reset_engine();
  if (g_euclid.clock == EUCLID_CLOCK_INT) {
    g_euclid.next_int_tick_ms = now_ms + euclid_int_interval_ms();
  }
  g_euclid.prev_clk_active = hal_io_trigger_active(HAL_IO_TR1);
  g_euclid.prev_rst_active = hal_io_trigger_active(HAL_IO_TR2);
  euclid_reset_outputs_and_state();
  g_euclid.grid_cache_valid = false;
}

static void load_runtime_from_app_settings(void) {
  g_grids.clock = (g_app_settings_data.grids.clock_mode == 0u) ? GRIDS_CLOCK_INT : GRIDS_CLOCK_EXT;
  g_grids.bpm = clamp_i((int)g_app_settings_data.grids.bpm, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
  g_grids.map_x = g_app_settings_data.grids.map_x;
  g_grids.map_y = g_app_settings_data.grids.map_y;
  g_grids.chaos = g_app_settings_data.grids.chaos;

  if (!g_trigseq.engine_initialized) {
    trigseq_engine_init(&g_trigseq.engine, 0x13579BDFu);
    g_trigseq.engine_initialized = true;
  }
  if (g_app_settings_data.trigseq.edit_channel < (uint8_t)TRIGSEQ_LEN_COUNT) {
    g_trigseq.len_mode = (trigseq_len_mode_t)g_app_settings_data.trigseq.edit_channel;
  } else {
    g_trigseq.len_mode = trigseq_mode_from_length(g_app_settings_data.trigseq.length);
  }
  trigseq_engine_set_length(&g_trigseq.engine, trigseq_len_for_mode(g_trigseq.len_mode));
  g_trigseq.clock =
      (g_app_settings_data.trigseq.clock_mode == 0u) ? TRIGSEQ_CLOCK_INT : TRIGSEQ_CLOCK_EXT;
  g_trigseq.bpm = clamp_i((int)g_app_settings_data.trigseq.bpm, TRIGSEQ_BPM_MIN, TRIGSEQ_BPM_MAX);
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_trigseq.engine.pattern[i] = g_app_settings_data.trigseq.pattern[i];
  }
  g_trigseq.cursor_step = (uint8_t)(g_app_settings_data.trigseq.edit_step & 0x3Fu);
  g_trigseq.run = g_app_settings_data.trigseq.run != 0u;
  trigseq_invalidate_grid_cache();

  g_euclid.clock = (g_app_settings_data.euclid.clock_mode == 0u) ? EUCLID_CLOCK_INT : EUCLID_CLOCK_EXT;
  g_euclid.bpm = clamp_i((int)g_app_settings_data.euclid.bpm, EUCLID_BPM_MIN, EUCLID_BPM_MAX);
  for (uint8_t i = 0u; i < 4u; ++i) {
    uint8_t steps = g_app_settings_data.euclid.steps[i];
    uint8_t hits = g_app_settings_data.euclid.hits[i];
    if (steps < 4u) steps = 4u;
    if (steps > 16u) steps = 16u;
    if (hits > steps) hits = steps;
    g_euclid.steps[i] = steps;
    g_euclid.hits[i] = hits;
  }
  g_euclid.grid_cache_valid = false;
}

static void capture_runtime_to_app_settings(void) {
  g_app_settings_data.grids.clock_mode = (g_grids.clock == GRIDS_CLOCK_INT) ? 0u : 1u;
  g_app_settings_data.grids.bpm = (uint16_t)clamp_i(g_grids.bpm, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
  g_app_settings_data.grids.map_x = g_grids.map_x;
  g_app_settings_data.grids.map_y = g_grids.map_y;
  g_app_settings_data.grids.chaos = g_grids.chaos;

  g_app_settings_data.trigseq.length = trigseq_len_for_mode(g_trigseq.len_mode);
  g_app_settings_data.trigseq.edit_channel = (uint8_t)g_trigseq.len_mode;
  g_app_settings_data.trigseq.edit_step = g_trigseq.cursor_step;
  g_app_settings_data.trigseq.clock_mode = (g_trigseq.clock == TRIGSEQ_CLOCK_INT) ? 0u : 1u;
  g_app_settings_data.trigseq.bpm = (uint16_t)clamp_i(g_trigseq.bpm, TRIGSEQ_BPM_MIN, TRIGSEQ_BPM_MAX);
  g_app_settings_data.trigseq.run = g_trigseq.run ? 1u : 0u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.trigseq.pattern[i] = g_trigseq.engine.pattern[i];
  }

  g_app_settings_data.euclid.clock_mode = (g_euclid.clock == EUCLID_CLOCK_INT) ? 0u : 1u;
  g_app_settings_data.euclid.bpm = (uint16_t)clamp_i(g_euclid.bpm, EUCLID_BPM_MIN, EUCLID_BPM_MAX);
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.euclid.steps[i] = g_euclid.steps[i];
    g_app_settings_data.euclid.hits[i] = g_euclid.hits[i];
  }
}

static void save_grids_settings(void) {
  capture_runtime_to_app_settings();
  if (app_settings_save(&g_app_settings_data)) {
    set_grids_status("SAVED");
  } else {
    set_grids_status("SAVE ERR");
  }
}

static void save_trigseq_settings(void) {
  capture_runtime_to_app_settings();
  if (app_settings_save(&g_app_settings_data)) {
    set_trigseq_status("SAVED");
  } else {
    set_trigseq_status("SAVE ERR");
  }
}

static void save_euclid_settings(void) {
  capture_runtime_to_app_settings();
  if (app_settings_save(&g_app_settings_data)) {
    set_euclid_status("SAVED");
  } else {
    set_euclid_status("SAVE ERR");
  }
}

static void grids_sample_fill_from_cv(void) {
  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    uint16_t raw = hal_mux_adc_read_raw(ch);
    int32_t mv = hal_mux_adc_raw_to_mv(raw);
    g_grids.fill_raw[ch] = raw;
    g_grids.fill_mv[ch] = mv;
    g_grids.fill_u8[ch] = (uint8_t)(raw >> 4u);
  }
}

static void grids_reset_outputs_and_state(void) {
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_grids.trig_state[i] = false;
    g_grids.trig_off_ms[i] = 0u;
  }
  g_grids.last_out_mask = 0xFFu;
  grids_apply_outputs(true);
}

static void grids_do_step(uint64_t now_ms) {
  bool trig[4] = {false, false, false, false};

  grids_sample_fill_from_cv();
  grids_engine_step(&g_grids.engine, g_grids.map_x, g_grids.map_y, g_grids.chaos, g_grids.fill_u8, trig);

  for (uint8_t i = 0u; i < 4u; ++i) {
    if (trig[i]) {
      // Keep pulse width constant: do not extend an already active pulse.
      if (!g_grids.trig_state[i]) {
        g_grids.trig_state[i] = true;
        g_grids.trig_off_ms[i] = now_ms + GRIDS_TRIG_PULSE_MS;
      }
    }
  }

  g_grids.step_count += 1u;
  grids_apply_outputs(false);
}

static void grids_update_pulses(uint64_t now_ms) {
  bool changed = false;
  for (uint8_t i = 0u; i < 4u; ++i) {
    if (g_grids.trig_state[i] && now_ms >= g_grids.trig_off_ms[i]) {
      g_grids.trig_state[i] = false;
      changed = true;
    }
  }
  if (changed) grids_apply_outputs(false);
}

static uint32_t grids_int_interval_ms(void) {
  int bpm = clamp_i(g_grids.bpm, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
  return (uint32_t)(60000 / bpm / 4);
}

static void grids_set_clock_source(grids_clock_t source, uint64_t now_ms) {
  g_grids.clock = source;
  if (g_grids.clock == GRIDS_CLOCK_INT) {
    g_grids.next_int_tick_ms = now_ms + grids_int_interval_ms();
  }
}

static void grids_reset_engine(void) {
  grids_engine_reset(&g_grids.engine);
  g_grids.step_count = 0u;
}

static void grids_enter(void) {
  uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  g_grids.preview_cache_valid = false;
  grids_engine_init(&g_grids.engine, (uint32_t)to_us_since_boot(get_absolute_time()));
  grids_reset_engine();
  grids_sample_fill_from_cv();
  g_grids.prev_clk_active = hal_io_trigger_active(HAL_IO_TR1);
  g_grids.prev_rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (g_grids.clock == GRIDS_CLOCK_INT) {
    g_grids.next_int_tick_ms = now_ms + grids_int_interval_ms();
  }
  grids_reset_outputs_and_state();
}

static void app_enter(app_mode_t mode) {
  g_app_mode = mode;
  hal_io_oled_clear();
  set_encoder_reference_now();

  if (mode == APP_MENU) {
    grids_reset_outputs_and_state();
    trigseq_reset_outputs_and_state();
    euclid_reset_outputs_and_state();
  } else if (mode == APP_HRDW_TEST) {
    g_hrdw.dac_ok = app_write_outputs_mv(g_hrdw.dac_mv);
  } else if (mode == APP_CALIBRATION) {
    g_hrdw.dac_ok = apply_calibration_preview();
    set_cal_status("CAL MODE");
  } else if (mode == APP_GRIDS) {
    grids_enter();
  } else if (mode == APP_TRIGSEQ) {
    trigseq_enter();
  } else if (mode == APP_EUCLID) {
    euclid_enter();
  }
}

static void update_menu(int32_t d_l, bool edge_enc_r) {
  const int menu_count = (int)(sizeof(k_menu_items) / sizeof(k_menu_items[0]));

  if (d_l != 0) {
    g_menu_index += (int)d_l;
    while (g_menu_index < 0) g_menu_index += menu_count;
    while (g_menu_index >= menu_count) g_menu_index -= menu_count;
  }

  if (edge_enc_r) {
    if (g_menu_index == 0) {
      app_enter(APP_HRDW_TEST);
    } else if (g_menu_index == 1) {
      app_enter(APP_CALIBRATION);
    } else if (g_menu_index == 2) {
      app_enter(APP_GRIDS);
    } else if (g_menu_index == 3) {
      app_enter(APP_TRIGSEQ);
    } else {
      app_enter(APP_EUCLID);
    }
  }
}

static void update_hrdw_test(int32_t d_l, int32_t d_r, bool edge_sw1) {
  if (edge_sw1) {
    g_hrdw.mux_manual_mode = !g_hrdw.mux_manual_mode;
  }

  if (g_hrdw.mux_manual_mode) {
    g_hrdw.mux_raw_manual = hal_mux_adc_read_raw((uint8_t)g_hrdw.selected_mux_ch);
    g_hrdw.mux_mv_manual = hal_mux_adc_raw_to_mv(g_hrdw.mux_raw_manual);
  } else {
    for (int ch = 0; ch < 4; ++ch) {
      g_hrdw.cv_raw[ch] = hal_mux_adc_read_raw((uint8_t)ch);
      g_hrdw.cv_mv[ch] = hal_mux_adc_raw_to_mv(g_hrdw.cv_raw[ch]);
    }
  }

  if (d_l != 0) {
    if (g_hrdw.mux_manual_mode) {
      g_hrdw.selected_mux_ch += (int)d_l;
      while (g_hrdw.selected_mux_ch < 0) g_hrdw.selected_mux_ch += 4;
      while (g_hrdw.selected_mux_ch > 3) g_hrdw.selected_mux_ch -= 4;
    } else {
      g_hrdw.selected_dac = (g_hrdw.selected_dac + (int)d_l) & 0x03;
    }
  }

  if (d_r != 0) {
    g_hrdw.dac_mv[g_hrdw.selected_dac] =
        clamp_mv(g_hrdw.dac_mv[g_hrdw.selected_dac] + (d_r * DAC_STEP_MV));
    g_hrdw.dac_ok = app_write_outputs_mv(g_hrdw.dac_mv);
  }
}

static void update_calibration(int32_t d_l, int32_t d_r, bool edge_enc_r) {
  if (d_l != 0) {
    int next = (int)g_cal_param + (int)d_l;
    while (next < 0) next += (int)CAL_PARAM_COUNT;
    while (next >= (int)CAL_PARAM_COUNT) next -= (int)CAL_PARAM_COUNT;
    g_cal_param = (cal_param_t)next;
  }

  if (d_r != 0) {
    if (g_cal_param == CAL_PARAM_CHANNEL) {
      int next = (int)g_cal_channel + (int)d_r;
      while (next < 0) next += 4;
      while (next > 3) next -= 4;
      g_cal_channel = (uint8_t)next;
    } else if (g_cal_param == CAL_PARAM_POINT) {
      int next = (int)g_cal_point + (int)d_r;
      while (next < 0) next += (int)CAL_POINT_COUNT;
      while (next >= (int)CAL_POINT_COUNT) next -= (int)CAL_POINT_COUNT;
      g_cal_point = (cal_point_t)next;
    } else {
      int code =
          (int)calibration_get_code(&g_calibration_data, g_cal_point, g_cal_channel) + (int)d_r;
      if (code < 0) code = 0;
      if (code > 4095) code = 4095;
      calibration_set_code(&g_calibration_data, g_cal_point, g_cal_channel, (uint16_t)code);
      g_calibration_dirty = true;
    }
    g_hrdw.dac_ok = apply_calibration_preview();
  }

  if (edge_enc_r) {
    if (calibration_save(&g_calibration_data)) {
      g_calibration_dirty = false;
      set_cal_status("SAVED");
    } else {
      set_cal_status("SAVE ERR");
    }
  }
}

static void update_grids(int32_t d_l, int32_t d_r, bool edge_enc_r, uint64_t now_ms) {
  bool clk_active;
  bool rst_active;

  if (d_l != 0) {
    int next = (int)g_grids.selected_param + (int)d_l;
    while (next < 0) next += (int)GRIDS_PARAM_COUNT;
    while (next >= (int)GRIDS_PARAM_COUNT) next -= (int)GRIDS_PARAM_COUNT;
    g_grids.selected_param = (grids_param_t)next;
  }

  if (d_r != 0) {
    if (g_grids.selected_param == GRIDS_PARAM_CLOCK) {
      if (d_r > 0) {
        grids_set_clock_source(GRIDS_CLOCK_EXT, now_ms);
      } else {
        grids_set_clock_source(GRIDS_CLOCK_INT, now_ms);
      }
    } else if (g_grids.selected_param == GRIDS_PARAM_BPM) {
      if (g_grids.clock == GRIDS_CLOCK_INT) {
        g_grids.bpm = clamp_i(g_grids.bpm + (int)d_r, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
        g_grids.next_int_tick_ms = now_ms + grids_int_interval_ms();
      }
    } else if (g_grids.selected_param == GRIDS_PARAM_MAP_X) {
      g_grids.map_x = clamp_u8i((int)g_grids.map_x + (int)d_r);
    } else if (g_grids.selected_param == GRIDS_PARAM_MAP_Y) {
      g_grids.map_y = clamp_u8i((int)g_grids.map_y + (int)d_r);
    } else {
      g_grids.chaos = clamp_u8i((int)g_grids.chaos + (int)d_r);
    }
    g_grids.preview_cache_valid = false;
  }

  grids_sample_fill_from_cv();

  rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (rst_active && !g_grids.prev_rst_active) {
    grids_reset_engine();
  }
  g_grids.prev_rst_active = rst_active;

  if (g_grids.clock == GRIDS_CLOCK_EXT) {
    clk_active = hal_io_trigger_active(HAL_IO_TR1);
    if (clk_active && !g_grids.prev_clk_active) {
      grids_do_step(now_ms);
    }
    g_grids.prev_clk_active = clk_active;
  } else {
    uint32_t interval = grids_int_interval_ms();
    uint8_t guard = 0;
    while (now_ms >= g_grids.next_int_tick_ms && guard < 8u) {
      grids_do_step(now_ms);
      g_grids.next_int_tick_ms += interval;
      ++guard;
    }
  }

  grids_update_pulses(now_ms);

  if (edge_enc_r) {
    save_grids_settings();
  }
}

static void update_trigseq(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                           uint64_t now_ms) {
  bool clk_active;
  bool rst_active;

  if (edge_sw1) {
    g_trigseq.focus = (g_trigseq.focus == TRIGSEQ_FOCUS_GRID) ? TRIGSEQ_FOCUS_MENU : TRIGSEQ_FOCUS_GRID;
    if (g_trigseq.focus == TRIGSEQ_FOCUS_GRID) {
      g_trigseq.cursor_step = 0u;  // grid starts at 1.1
    }
    trigseq_invalidate_grid_cache();
  }

  if (g_trigseq.focus == TRIGSEQ_FOCUS_GRID) {
    if (d_l != 0) {
      int row = (int)(g_trigseq.cursor_step / 16u);
      int col = (int)(g_trigseq.cursor_step % 16u);
      row += (int)d_l;
      while (row < 0) row += 4;
      while (row >= 4) row -= 4;
      g_trigseq.cursor_step = (uint8_t)(row * 16 + col);
    }

    if (d_r != 0) {
      int row = (int)(g_trigseq.cursor_step / 16u);
      int col = (int)(g_trigseq.cursor_step % 16u);
      col += (int)d_r;
      while (col < 0) col += 16;
      while (col >= 16) col -= 16;
      g_trigseq.cursor_step = (uint8_t)(row * 16 + col);
    }

    if (edge_sw2) {
      bool now_on = trigseq_grid_get_bit(g_trigseq.cursor_step);
      trigseq_grid_set_bit(g_trigseq.cursor_step, !now_on);
    }
  } else {
    if (d_l != 0) {
      int next = (int)g_trigseq.selected_param + (int)d_l;
      while (next < 0) next += (int)TRIGSEQ_PARAM_COUNT;
      while (next >= (int)TRIGSEQ_PARAM_COUNT) next -= (int)TRIGSEQ_PARAM_COUNT;
      g_trigseq.selected_param = (trigseq_param_t)next;
    }

    if (d_r != 0) {
      if (g_trigseq.selected_param == TRIGSEQ_PARAM_LEN) {
        int next = (int)g_trigseq.len_mode + ((d_r > 0) ? 1 : -1);
        while (next < 0) next += (int)TRIGSEQ_LEN_COUNT;
        while (next >= (int)TRIGSEQ_LEN_COUNT) next -= (int)TRIGSEQ_LEN_COUNT;
        g_trigseq.len_mode = (trigseq_len_mode_t)next;
        trigseq_engine_set_length(&g_trigseq.engine, trigseq_len_for_mode(g_trigseq.len_mode));
        trigseq_reset_engine();
        trigseq_reset_outputs_and_state();
      } else if (g_trigseq.selected_param == TRIGSEQ_PARAM_CLOCK) {
        if (d_r > 0) {
          trigseq_set_clock_source(TRIGSEQ_CLOCK_EXT, now_ms);
        } else {
          trigseq_set_clock_source(TRIGSEQ_CLOCK_INT, now_ms);
        }
      } else if (g_trigseq.selected_param == TRIGSEQ_PARAM_BPM) {
        if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
          g_trigseq.bpm = clamp_i(g_trigseq.bpm + (int)d_r, TRIGSEQ_BPM_MIN, TRIGSEQ_BPM_MAX);
          g_trigseq.next_int_tick_ms = now_ms + trigseq_int_interval_ms();
        }
      } else {
        g_trigseq.run = d_r > 0;
        if (!g_trigseq.run) {
          trigseq_reset_outputs_and_state();
        }
      }
    }
  }

  rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (rst_active && !g_trigseq.prev_rst_active) {
    trigseq_reset_engine();
  }
  g_trigseq.prev_rst_active = rst_active;

  if (g_trigseq.clock == TRIGSEQ_CLOCK_EXT) {
    clk_active = hal_io_trigger_active(HAL_IO_TR1);
    if (g_trigseq.run && clk_active && !g_trigseq.prev_clk_active) {
      trigseq_do_step(now_ms);
    }
    g_trigseq.prev_clk_active = clk_active;
  } else {
    uint32_t interval = trigseq_int_interval_ms();
    uint8_t guard = 0;
    while (g_trigseq.run && now_ms >= g_trigseq.next_int_tick_ms && guard < 8u) {
      trigseq_do_step(now_ms);
      g_trigseq.next_int_tick_ms += interval;
      ++guard;
    }
    g_trigseq.prev_clk_active = hal_io_trigger_active(HAL_IO_TR1);
  }

  trigseq_update_pulses(now_ms);

  if (edge_enc_r) {
    save_trigseq_settings();
  }
}

static void update_euclid(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, uint64_t now_ms) {
  bool clk_active;
  bool rst_active;

  (void)edge_sw1;
  g_euclid.focus = EUCLID_FOCUS_MENU;

  if (d_l != 0) {
    int next = (int)g_euclid.selected_param + (int)d_l;
    while (next < 0) next += (int)EUCLID_PARAM_COUNT;
    while (next >= (int)EUCLID_PARAM_COUNT) next -= (int)EUCLID_PARAM_COUNT;
    g_euclid.selected_param = (euclid_param_t)next;
  }

  if (d_r != 0) {
    if (g_euclid.selected_param == EUCLID_PARAM_CLOCK) {
      if (d_r > 0) {
        euclid_set_clock_source(EUCLID_CLOCK_EXT, now_ms);
      } else {
        euclid_set_clock_source(EUCLID_CLOCK_INT, now_ms);
      }
    } else if (g_euclid.selected_param == EUCLID_PARAM_BPM) {
      if (g_euclid.clock == EUCLID_CLOCK_INT) {
        g_euclid.bpm = clamp_i(g_euclid.bpm + (int)d_r, EUCLID_BPM_MIN, EUCLID_BPM_MAX);
        g_euclid.next_int_tick_ms = now_ms + euclid_int_interval_ms();
      }
    } else {
      uint8_t ch = (uint8_t)(((int)g_euclid.selected_param - 2) / 2);
      bool editing_steps = (((int)g_euclid.selected_param - 2) % 2) == 0;
      if (ch < 4u) {
        if (editing_steps) {
          int next_steps = clamp_i((int)g_euclid.steps[ch] + (int)d_r, 4, 16);
          g_euclid.steps[ch] = (uint8_t)next_steps;
          if (g_euclid.hits[ch] > g_euclid.steps[ch]) {
            g_euclid.hits[ch] = g_euclid.steps[ch];
          }
          g_euclid.phase[ch] %= g_euclid.steps[ch];
        } else {
          int next_hits = clamp_i((int)g_euclid.hits[ch] + (int)d_r, 0, (int)g_euclid.steps[ch]);
          g_euclid.hits[ch] = (uint8_t)next_hits;
        }
      }
    }
    g_euclid.grid_cache_valid = false;
  }

  rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (rst_active && !g_euclid.prev_rst_active) {
    euclid_reset_engine();
  }
  g_euclid.prev_rst_active = rst_active;

  if (g_euclid.clock == EUCLID_CLOCK_EXT) {
    clk_active = hal_io_trigger_active(HAL_IO_TR1);
    if (clk_active && !g_euclid.prev_clk_active) {
      euclid_do_step(now_ms);
    }
    g_euclid.prev_clk_active = clk_active;
  } else {
    uint32_t interval = euclid_int_interval_ms();
    uint8_t guard = 0;
    while (now_ms >= g_euclid.next_int_tick_ms && guard < 8u) {
      euclid_do_step(now_ms);
      g_euclid.next_int_tick_ms += interval;
      ++guard;
    }
    g_euclid.prev_clk_active = hal_io_trigger_active(HAL_IO_TR1);
  }

  euclid_update_pulses(now_ms);

  if (edge_enc_r) {
    save_euclid_settings();
  }
}

static void draw_menu(void) {
  char line[32];
  const int menu_count = (int)(sizeof(k_menu_items) / sizeof(k_menu_items[0]));

  hal_io_oled_draw_line(0, "APP MENU  ER_SW OPEN", true);
  for (int i = 0; i < menu_count; ++i) {
    snprintf(line, sizeof(line), "%c %s", (i == g_menu_index) ? '>' : ' ', k_menu_items[i]);
    hal_io_oled_draw_line((uint8_t)(2 + i), line, i == g_menu_index);
  }

  hal_io_oled_draw_line(7, "ENC_L=SELECT", false);
  hal_io_oled_draw_line(8, "ER_SW=ENTER", false);
  hal_io_oled_draw_line(9, "ENC_L_SW=BACK", false);
  clear_rows(10);
}

static void draw_hrdw_test(void) {
  char line[32];

  hal_io_oled_draw_line(0, "HRDW TEST AZURE", true);
  snprintf(line, sizeof(line), "SEL DAC %c  %s", (char)('A' + g_hrdw.selected_dac),
           g_hrdw.dac_ok ? "OK " : "ERR");
  hal_io_oled_draw_line(1, line, false);

  snprintf(line, sizeof(line), "A %+1ld.%03ld B %+1ld.%03ld", (long)(g_hrdw.dac_mv[0] / 1000),
           (long)((g_hrdw.dac_mv[0] < 0 ? -g_hrdw.dac_mv[0] : g_hrdw.dac_mv[0]) % 1000),
           (long)(g_hrdw.dac_mv[1] / 1000),
           (long)((g_hrdw.dac_mv[1] < 0 ? -g_hrdw.dac_mv[1] : g_hrdw.dac_mv[1]) % 1000));
  hal_io_oled_draw_line(2, line, false);

  snprintf(line, sizeof(line), "C %+1ld.%03ld D %+1ld.%03ld", (long)(g_hrdw.dac_mv[2] / 1000),
           (long)((g_hrdw.dac_mv[2] < 0 ? -g_hrdw.dac_mv[2] : g_hrdw.dac_mv[2]) % 1000),
           (long)(g_hrdw.dac_mv[3] / 1000),
           (long)((g_hrdw.dac_mv[3] < 0 ? -g_hrdw.dac_mv[3] : g_hrdw.dac_mv[3]) % 1000));
  hal_io_oled_draw_line(3, line, false);

  if (g_hrdw.mux_manual_mode) {
    snprintf(line, sizeof(line), "MUX CH%d RAW %4u", g_hrdw.selected_mux_ch + 1,
             (unsigned)g_hrdw.mux_raw_manual);
    hal_io_oled_draw_line(4, line, false);
    snprintf(line, sizeof(line), "VIN %+1ld.%03ldV SW1 AUT", (long)(g_hrdw.mux_mv_manual / 1000),
             (long)((g_hrdw.mux_mv_manual < 0 ? -g_hrdw.mux_mv_manual : g_hrdw.mux_mv_manual) %
                    1000));
    hal_io_oled_draw_line(5, line, false);
  } else {
    format_cv_line(line, sizeof(line), 1u, g_hrdw.cv_mv[0], 2u, g_hrdw.cv_mv[1]);
    hal_io_oled_draw_line(4, line, false);
    format_cv_line(line, sizeof(line), 3u, g_hrdw.cv_mv[2], 4u, g_hrdw.cv_mv[3]);
    hal_io_oled_draw_line(5, line, false);
  }

  snprintf(line, sizeof(line), "TR1:%s TR2:%s", hal_io_trigger_active(HAL_IO_TR1) ? "HIGH" : "LOW",
           hal_io_trigger_active(HAL_IO_TR2) ? "HIGH" : "LOW");
  hal_io_oled_draw_line(6, line, false);
  snprintf(line, sizeof(line), "TR3:%s TR4:%s", hal_io_trigger_active(HAL_IO_TR3) ? "HIGH" : "LOW",
           hal_io_trigger_active(HAL_IO_TR4) ? "HIGH" : "LOW");
  hal_io_oled_draw_line(7, line, false);

  snprintf(line, sizeof(line), "EL %ld +%lu -%lu", (long)hal_io_encoder_count(HAL_IO_ENC_L),
           (unsigned long)hal_io_encoder_inc_events(HAL_IO_ENC_L),
           (unsigned long)hal_io_encoder_dec_events(HAL_IO_ENC_L));
  hal_io_oled_draw_line(8, line, false);

  snprintf(line, sizeof(line), "ER %ld +%lu -%lu", (long)hal_io_encoder_count(HAL_IO_ENC_R),
           (unsigned long)hal_io_encoder_inc_events(HAL_IO_ENC_R),
           (unsigned long)hal_io_encoder_dec_events(HAL_IO_ENC_R));
  hal_io_oled_draw_line(9, line, false);

  snprintf(line, sizeof(line), "ELS:%u ERS:%u 1:%u 2:%u", hal_io_button_pressed(HAL_IO_BTN_ENC_L) ? 1u : 0u,
           hal_io_button_pressed(HAL_IO_BTN_ENC_R) ? 1u : 0u,
           hal_io_button_pressed(HAL_IO_BTN_SW1) ? 1u : 0u,
           hal_io_button_pressed(HAL_IO_BTN_SW2) ? 1u : 0u);
  hal_io_oled_draw_line(10, line, false);

  hal_io_oled_draw_line(11, "ENC_L_SW BACK", false);
  clear_rows(12);
}

static void draw_calibration(void) {
  char line[32];
  int32_t target_mv = calibration_point_millivolts(g_cal_point);
  bool show_status =
      (g_cal_status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_cal_status[0] != '\0');

  hal_io_oled_draw_line(0, "CALIBRATION", true);

  snprintf(line, sizeof(line), "%c CH: %c", g_cal_param == CAL_PARAM_CHANNEL ? '>' : ' ',
           (char)('A' + g_cal_channel));
  hal_io_oled_draw_line(2, line, g_cal_param == CAL_PARAM_CHANNEL);

  snprintf(line, sizeof(line), "%c POINT: %s", g_cal_param == CAL_PARAM_POINT ? '>' : ' ',
           point_label(g_cal_point));
  hal_io_oled_draw_line(3, line, g_cal_param == CAL_PARAM_POINT);

  snprintf(line, sizeof(line), "%c CODE: %4u", g_cal_param == CAL_PARAM_CODE ? '>' : ' ',
           (unsigned)calibration_get_code(&g_calibration_data, g_cal_point, g_cal_channel));
  hal_io_oled_draw_line(4, line, g_cal_param == CAL_PARAM_CODE);

  snprintf(line, sizeof(line), "TARGET %ld.%03ldV", (long)(target_mv / 1000),
           (long)(target_mv < 0 ? -(target_mv % 1000) : (target_mv % 1000)));
  hal_io_oled_draw_line(6, line, false);

  hal_io_oled_draw_line(7, "ENC_L PARAM", false);
  hal_io_oled_draw_line(8, "ENC_R VALUE", false);
  hal_io_oled_draw_line(9, "ER_SW SAVE", false);
  hal_io_oled_draw_line(10, "ENC_L_SW BACK", false);

  if (show_status) {
    snprintf(line, sizeof(line), "%s%s", g_cal_status, g_calibration_dirty ? " *" : "");
    hal_io_oled_draw_line(12, line, false);
  } else {
    if (g_calibration_dirty) {
      hal_io_oled_draw_line(12, "DIRTY", false);
    } else {
      hal_io_oled_draw_line(12, "", false);
    }
  }

  clear_rows(13);
}

static void draw_grids(void) {
  char line[32];
  char token[16];
  bool show_status =
      (g_grids.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_grids.status[0] != '\0');
  bool preview[4][32];
  bool progress[4][32];
  const uint8_t grid_x = 14u;
  const uint8_t grid_top = 43u;
  const uint8_t channel_stride = 14u;  // 2 rows * 6px + 2px visual gap between channels
  const uint8_t cell_pitch = 6u;
  const uint8_t cell_size = 6u;

  hal_io_oled_draw_line(0, "GRIDS", true);
  hal_io_oled_draw_line(1, "", false);

  snprintf(line, sizeof(line), "CLK:%s BPM:%3d", g_grids.clock == GRIDS_CLOCK_INT ? "INT" : "EXT", g_grids.bpm);
  hal_io_oled_draw_line(2, line, false);
  if (g_grids.selected_param == GRIDS_PARAM_CLOCK) {
    snprintf(token, sizeof(token), "CLK:%s", g_grids.clock == GRIDS_CLOCK_INT ? "INT" : "EXT");
    hal_io_oled_draw_text(0u, 16u, token, true);
  } else if (g_grids.selected_param == GRIDS_PARAM_BPM) {
    snprintf(token, sizeof(token), "BPM:%3d", g_grids.bpm);
    hal_io_oled_draw_text((uint8_t)(8u * 6u), 16u, token, true);
  }

  snprintf(line, sizeof(line), "MAPX:%3u MAPY:%3u", (unsigned)g_grids.map_x, (unsigned)g_grids.map_y);
  hal_io_oled_draw_line(3, line, false);
  if (g_grids.selected_param == GRIDS_PARAM_MAP_X) {
    snprintf(token, sizeof(token), "MAPX:%3u", (unsigned)g_grids.map_x);
    hal_io_oled_draw_text(0u, 24u, token, true);
  } else if (g_grids.selected_param == GRIDS_PARAM_MAP_Y) {
    snprintf(token, sizeof(token), "MAPY:%3u", (unsigned)g_grids.map_y);
    hal_io_oled_draw_text((uint8_t)(9u * 6u), 24u, token, true);
  }

  snprintf(line, sizeof(line), "CHA:%3u", (unsigned)g_grids.chaos);
  hal_io_oled_draw_line(4, line, false);
  if (g_grids.selected_param == GRIDS_PARAM_CHAOS) {
    snprintf(token, sizeof(token), "CHA:%3u", (unsigned)g_grids.chaos);
    hal_io_oled_draw_text(0u, 32u, token, true);
  }

  {
    grids_engine_t sim = g_grids.engine;
    uint8_t fill[4] = {g_grids.fill_u8[0], g_grids.fill_u8[1], g_grids.fill_u8[2], g_grids.fill_u8[3]};

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
      for (uint8_t s = 0u; s < 32u; ++s) {
        preview[ch][s] = false;
        progress[ch][s] = false;
      }
    }

    for (uint8_t s = 0u; s < 32u; ++s) {
      bool trig[4] = {false, false, false, false};
      uint8_t idx = sim.step;
      grids_engine_step(&sim, g_grids.map_x, g_grids.map_y, g_grids.chaos, fill, trig);
      for (uint8_t ch = 0u; ch < 4u; ++ch) {
        preview[ch][idx] = trig[ch];
      }
    }

    {
      uint8_t current = (uint8_t)((g_grids.engine.step + 31u) & 0x1Fu);
      for (uint8_t ch = 0u; ch < 4u; ++ch) {
        progress[ch][current] = true;
      }
    }
  }

  if (!g_grids.preview_cache_valid) {
    hal_io_oled_fill_rect(0u, 43u, 160u, 56u, false);
  }

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    for (uint8_t s = 0u; s < 32u; ++s) {
      uint8_t col = (uint8_t)(s & 0x0Fu);
      uint8_t row_y = (uint8_t)(grid_top + (uint8_t)(ch * channel_stride) + (s < 16u ? 0u : 6u));
      uint8_t x = (uint8_t)(grid_x + col * cell_pitch);
      uint8_t y = row_y;
      bool on = preview[ch][s];
      bool prog = progress[ch][s];
      bool was_on = g_grids.preview_cache_bits[ch][s];
      bool was_prog = g_grids.preview_cache_progress[ch][s];

      if (!g_grids.preview_cache_valid || on != was_on || prog != was_prog) {
        hal_io_oled_fill_rect(x, y, cell_pitch, cell_pitch, false);
        hal_io_oled_draw_rect(x, y, cell_size, cell_size, true);
        hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 1u), 4u, 4u, false);
        if (on) {
          hal_io_oled_fill_rect((uint8_t)(x + 2u), (uint8_t)(y + 2u), 2u, 2u, true);
        }
        if (prog) {
          hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 4u), 4u, 1u, true);
        }
      }

      g_grids.preview_cache_bits[ch][s] = on;
      g_grids.preview_cache_progress[ch][s] = prog;
    }
  }
  g_grids.preview_cache_valid = true;

  if (show_status) {
    hal_io_oled_draw_line(13, g_grids.status, false);
  } else {
    hal_io_oled_draw_line(13, "", false);
  }
  hal_io_oled_draw_line(14, "ENC1 PARAM ENC2 VALUE", false);
  hal_io_oled_draw_line(15, "ENC_R SAVE ENC_L_SW BK", false);
}

static void trigseq_draw_grid_cell(uint8_t step, bool on, bool selected, bool progress) {
  const uint8_t grid_x = 10u;
  const uint8_t grid_y = 42u;
  const uint8_t cell = 7u;
  uint8_t row = (uint8_t)(step / 16u);
  uint8_t col = (uint8_t)(step % 16u);
  uint8_t x = (uint8_t)(grid_x + col * cell);
  uint8_t y = (uint8_t)(grid_y + row * cell);

  hal_io_oled_draw_rect(x, y, 7u, 7u, true);
  hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 1u), 5u, 5u, false);

  if (on) {
    hal_io_oled_fill_rect((uint8_t)(x + 2u), (uint8_t)(y + 2u), 3u, 3u, true);
  }
  if (selected) {
    hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 1u), 1u, 1u, true);
    hal_io_oled_fill_rect((uint8_t)(x + 5u), (uint8_t)(y + 1u), 1u, 1u, true);
    hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 5u), 1u, 1u, true);
    hal_io_oled_fill_rect((uint8_t)(x + 5u), (uint8_t)(y + 5u), 1u, 1u, true);
  }
  if (progress) {
    hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 5u), 5u, 1u, true);
  }
}

static void euclid_draw_grid_cell(uint8_t step, bool visible, bool on, bool progress) {
  const uint8_t grid_x = 10u;
  const uint8_t grid_y = 57u;
  const uint8_t cell = 7u;
  uint8_t row = (uint8_t)(step / 16u);
  uint8_t col = (uint8_t)(step % 16u);
  uint8_t x = (uint8_t)(grid_x + col * cell);
  uint8_t y = (uint8_t)(grid_y + row * cell);

  if (!visible) {
    hal_io_oled_fill_rect(x, y, 7u, 7u, false);
    return;
  }

  hal_io_oled_draw_rect(x, y, 7u, 7u, true);
  hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 1u), 5u, 5u, false);

  if (on) {
    hal_io_oled_fill_rect((uint8_t)(x + 2u), (uint8_t)(y + 2u), 3u, 3u, true);
  }
  if (progress) {
    hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 5u), 5u, 1u, true);
  }
}

static void oled_draw_text_26(uint8_t y, const char* text, bool inverted) {
  char padded[27];
  snprintf(padded, sizeof(padded), "%-26.26s", text);
  hal_io_oled_draw_text(0u, y, padded, inverted);
}

static void draw_trigseq(void) {
  char line[32];
  bool force_grid = !g_trigseq.grid_cache_valid || (g_trigseq.grid_cache_mode != g_trigseq.len_mode);
  bool show_status =
      (g_trigseq.status_until_ms > to_ms_since_boot(get_absolute_time())) &&
      (g_trigseq.status[0] != '\0');

  hal_io_oled_draw_line(0, "TRIGSEQ GRID", true);

  snprintf(line, sizeof(line), "%c LEN   %s", g_trigseq.selected_param == TRIGSEQ_PARAM_LEN ? '>' : ' ',
           trigseq_mode_label(g_trigseq.len_mode));
  hal_io_oled_draw_line(1, line,
                        g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_LEN);
  snprintf(line, sizeof(line), "%c CLOCK %s", g_trigseq.selected_param == TRIGSEQ_PARAM_CLOCK ? '>' : ' ',
           g_trigseq.clock == TRIGSEQ_CLOCK_INT ? "INT" : "EXT");
  hal_io_oled_draw_line(2, line,
                        g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_CLOCK);
  if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
    snprintf(line, sizeof(line), "%c BPM   %3d", g_trigseq.selected_param == TRIGSEQ_PARAM_BPM ? '>' : ' ',
             g_trigseq.bpm);
  } else {
    snprintf(line, sizeof(line), "%c BPM   ---", g_trigseq.selected_param == TRIGSEQ_PARAM_BPM ? '>' : ' ');
  }
  hal_io_oled_draw_line(3, line,
                        g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_BPM);
  snprintf(line, sizeof(line), "%c RUN   %s", g_trigseq.selected_param == TRIGSEQ_PARAM_RUN ? '>' : ' ',
           g_trigseq.run ? "ON" : "OFF");
  hal_io_oled_draw_line(4, line,
                        g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_RUN);

  if (force_grid) {
    // Clear thin gaps around the pixel-grid area so stale glyph rows don't remain visible.
    hal_io_oled_fill_rect(0u, 40u, 160u, 2u, false);
    hal_io_oled_fill_rect(0u, 70u, 160u, 2u, false);
    // Clear left/right gutters around the grid to remove leftovers from old full-width text rows.
    hal_io_oled_fill_rect(0u, 42u, 10u, 28u, false);
    hal_io_oled_fill_rect(122u, 42u, 38u, 28u, false);
    hal_io_oled_fill_rect(10u, 42u, 112u, 28u, false);
    // Closing bars around the grid area.
    hal_io_oled_fill_rect(10u, 70u, 112u, 1u, true);  // bottom bar
    hal_io_oled_fill_rect(122u, 42u, 1u, 28u, true);  // right-side bar
  }
  for (uint8_t step = 0u; step < 64u; ++step) {
    bool on = trigseq_grid_get_bit(step);
    bool selected = (step == g_trigseq.cursor_step);
    bool progress = trigseq_is_progress_step(step);
    bool was_on = g_trigseq.grid_cache_bits[step];
    bool was_selected = (step == g_trigseq.grid_cache_cursor);
    bool was_progress = g_trigseq.grid_cache_progress[step];
    if (force_grid || on != was_on || selected != was_selected || progress != was_progress) {
      trigseq_draw_grid_cell(step, on, selected, progress);
    }
    g_trigseq.grid_cache_bits[step] = on;
    g_trigseq.grid_cache_progress[step] = progress;
  }
  g_trigseq.grid_cache_valid = true;
  g_trigseq.grid_cache_mode = g_trigseq.len_mode;
  g_trigseq.grid_cache_cursor = g_trigseq.cursor_step;

  snprintf(line, sizeof(line), "CLK:TR1 RST:TR2 %s", g_trigseq.clock == TRIGSEQ_CLOCK_INT ? "INT" : "EXT");
  hal_io_oled_draw_line(9, line, false);
  hal_io_oled_draw_line(10, "", false);

  if (show_status) {
    hal_io_oled_draw_line(11, g_trigseq.status, false);
  } else {
    hal_io_oled_draw_line(11, "", false);
  }
  hal_io_oled_draw_line(12, "SW1 GRID/MENU", false);
  hal_io_oled_draw_line(13, "SW2 ON/OFF", false);
  hal_io_oled_draw_line(14, "ENC_R_SW SAVE", false);
  hal_io_oled_draw_line(15, "ENC_L_SW BACK", false);
}

static void draw_euclid(void) {
  char line[32];
  char token[16];
  const uint8_t y_off = 7u;
  const uint8_t y_clock = (uint8_t)(8u + y_off);
  const uint8_t y_ch1 = (uint8_t)(16u + y_off);
  const uint8_t y_ch2 = (uint8_t)(24u + y_off);
  const uint8_t y_ch3 = (uint8_t)(32u + y_off);
  const uint8_t y_ch4 = (uint8_t)(40u + y_off);
  const uint8_t y_clk_src = (uint8_t)(85u + y_off);
  bool force_grid = !g_euclid.grid_cache_valid;
  bool show_status =
      (g_euclid.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_euclid.status[0] != '\0');
  bool sel_clock = g_euclid.selected_param == EUCLID_PARAM_CLOCK;
  bool sel_bpm = g_euclid.selected_param == EUCLID_PARAM_BPM;
  bool sel_c1s = g_euclid.selected_param == EUCLID_PARAM_CH1_STEPS;
  bool sel_c1h = g_euclid.selected_param == EUCLID_PARAM_CH1_HITS;
  bool sel_c2s = g_euclid.selected_param == EUCLID_PARAM_CH2_STEPS;
  bool sel_c2h = g_euclid.selected_param == EUCLID_PARAM_CH2_HITS;
  bool sel_c3s = g_euclid.selected_param == EUCLID_PARAM_CH3_STEPS;
  bool sel_c3h = g_euclid.selected_param == EUCLID_PARAM_CH3_HITS;
  bool sel_c4s = g_euclid.selected_param == EUCLID_PARAM_CH4_STEPS;
  bool sel_c4h = g_euclid.selected_param == EUCLID_PARAM_CH4_HITS;

  hal_io_oled_draw_line(0, "4X EUCLID", true);

  if (g_euclid.clock == EUCLID_CLOCK_INT) {
    snprintf(line, sizeof(line), "CLOCK:%s BPM:%3d", g_euclid.clock == EUCLID_CLOCK_INT ? "INT" : "EXT",
             g_euclid.bpm);
  } else {
    snprintf(line, sizeof(line), "CLOCK:%s BPM:---", g_euclid.clock == EUCLID_CLOCK_INT ? "INT" : "EXT");
  }
  oled_draw_text_26(y_clock, line, false);
  if (sel_clock) {
    snprintf(token, sizeof(token), "CLOCK:%s", g_euclid.clock == EUCLID_CLOCK_INT ? "INT" : "EXT");
    hal_io_oled_draw_text(0u, y_clock, token, true);
  } else if (sel_bpm) {
    if (g_euclid.clock == EUCLID_CLOCK_INT) {
      snprintf(token, sizeof(token), "BPM:%3d", g_euclid.bpm);
    } else {
      snprintf(token, sizeof(token), "BPM:---");
    }
    hal_io_oled_draw_text((uint8_t)(10u * 6u), y_clock, token, true);
  }

  snprintf(line, sizeof(line), "CH1 STEPS:%2u HITS:%2u", g_euclid.steps[0], g_euclid.hits[0]);
  oled_draw_text_26(y_ch1, line, false);
  if (sel_c1s) {
    snprintf(token, sizeof(token), "STEPS:%2u", g_euclid.steps[0]);
    hal_io_oled_draw_text((uint8_t)(4u * 6u), y_ch1, token, true);
  } else if (sel_c1h) {
    snprintf(token, sizeof(token), "HITS:%2u", g_euclid.hits[0]);
    hal_io_oled_draw_text((uint8_t)(13u * 6u), y_ch1, token, true);
  }
  snprintf(line, sizeof(line), "CH2 STEPS:%2u HITS:%2u", g_euclid.steps[1], g_euclid.hits[1]);
  oled_draw_text_26(y_ch2, line, false);
  if (sel_c2s) {
    snprintf(token, sizeof(token), "STEPS:%2u", g_euclid.steps[1]);
    hal_io_oled_draw_text((uint8_t)(4u * 6u), y_ch2, token, true);
  } else if (sel_c2h) {
    snprintf(token, sizeof(token), "HITS:%2u", g_euclid.hits[1]);
    hal_io_oled_draw_text((uint8_t)(13u * 6u), y_ch2, token, true);
  }
  snprintf(line, sizeof(line), "CH3 STEPS:%2u HITS:%2u", g_euclid.steps[2], g_euclid.hits[2]);
  oled_draw_text_26(y_ch3, line, false);
  if (sel_c3s) {
    snprintf(token, sizeof(token), "STEPS:%2u", g_euclid.steps[2]);
    hal_io_oled_draw_text((uint8_t)(4u * 6u), y_ch3, token, true);
  } else if (sel_c3h) {
    snprintf(token, sizeof(token), "HITS:%2u", g_euclid.hits[2]);
    hal_io_oled_draw_text((uint8_t)(13u * 6u), y_ch3, token, true);
  }
  snprintf(line, sizeof(line), "CH4 STEPS:%2u HITS:%2u", g_euclid.steps[3], g_euclid.hits[3]);
  oled_draw_text_26(y_ch4, line, false);
  if (sel_c4s) {
    snprintf(token, sizeof(token), "STEPS:%2u", g_euclid.steps[3]);
    hal_io_oled_draw_text((uint8_t)(4u * 6u), y_ch4, token, true);
  } else if (sel_c4h) {
    snprintf(token, sizeof(token), "HITS:%2u", g_euclid.hits[3]);
    hal_io_oled_draw_text((uint8_t)(13u * 6u), y_ch4, token, true);
  }

  if (force_grid) {
    hal_io_oled_fill_rect(0u, 55u, 160u, 2u, false);
    hal_io_oled_fill_rect(0u, 85u, 160u, 2u, false);
    hal_io_oled_fill_rect(0u, 57u, 10u, 28u, false);
    hal_io_oled_fill_rect(122u, 57u, 38u, 28u, false);
    hal_io_oled_fill_rect(10u, 57u, 112u, 28u, false);
  }

  for (uint8_t step = 0u; step < 64u; ++step) {
    uint8_t row = (uint8_t)(step / 16u);
    uint8_t col = (uint8_t)(step % 16u);
    bool visible = col < g_euclid.steps[row];
    bool on = euclid_grid_get_bit(step);
    bool progress = euclid_is_progress_step(step);
    bool was_on = g_euclid.grid_cache_bits[step];
    bool was_progress = g_euclid.grid_cache_progress[step];
    if (force_grid || on != was_on || progress != was_progress) {
      euclid_draw_grid_cell(step, visible, on, progress);
    }
    g_euclid.grid_cache_bits[step] = on;
    g_euclid.grid_cache_progress[step] = progress;
  }
  g_euclid.grid_cache_valid = true;

  hal_io_oled_draw_line(12, "", false);
  snprintf(line, sizeof(line), "CLK:TR1 RST:TR2 %s", g_euclid.clock == EUCLID_CLOCK_INT ? "INT" : "EXT");
  oled_draw_text_26(y_clk_src, line, false);

  if (show_status) {
    hal_io_oled_draw_line(13, g_euclid.status, false);
  } else {
    hal_io_oled_draw_line(13, "", false);
  }
  hal_io_oled_draw_line(14, "ENC_R_SW SAVE", false);
  hal_io_oled_draw_line(15, "ENC_L_SW BACK", false);
}

static void service_active_app_pulses(uint64_t now_ms) {
  if (g_app_mode == APP_GRIDS) {
    grids_update_pulses(now_ms);
  } else if (g_app_mode == APP_TRIGSEQ) {
    trigseq_update_pulses(now_ms);
  } else if (g_app_mode == APP_EUCLID) {
    euclid_update_pulses(now_ms);
  }
}

static bool active_app_has_live_pulse(void) {
  if (g_app_mode == APP_GRIDS) {
    return g_grids.trig_state[0] || g_grids.trig_state[1] || g_grids.trig_state[2] || g_grids.trig_state[3];
  }
  if (g_app_mode == APP_TRIGSEQ) {
    return g_trigseq.trig_state[0] || g_trigseq.trig_state[1] || g_trigseq.trig_state[2] || g_trigseq.trig_state[3];
  }
  if (g_app_mode == APP_EUCLID) {
    return g_euclid.trig_state[0] || g_euclid.trig_state[1] || g_euclid.trig_state[2] || g_euclid.trig_state[3];
  }
  return false;
}

int main(void) {
  uint64_t now_ms;
  uint64_t last_draw_ms = 0;

  stdio_init_all();
  sleep_ms(1200);

  hal_io_init();
  hal_mux_adc_init();
  hal_io_oled_clear();

  g_calibration_loaded = calibration_init(&g_calibration_data);
  g_app_settings_loaded = app_settings_init(&g_app_settings_data);
  load_runtime_from_app_settings();
  g_hrdw.dac_ok = app_write_outputs_mv(g_hrdw.dac_mv);

  set_encoder_reference_now();

  printf("hrdw_test start, calib source: %s, app settings: %s\n",
         g_calibration_loaded ? "flash" : "defaults",
         g_app_settings_loaded ? "flash" : "defaults");

  while (true) {
    int32_t enc_l_now;
    int32_t enc_r_now;
    int32_t d_l;
    int32_t d_r;
    bool edge_enc_l;
    bool edge_enc_r;
    bool edge_sw1;
    bool edge_sw2;

    now_ms = to_ms_since_boot(get_absolute_time());
    hal_io_poll(now_ms);

    edge_enc_l = hal_io_button_edge_pressed(HAL_IO_BTN_ENC_L);
    edge_enc_r = hal_io_button_edge_pressed(HAL_IO_BTN_ENC_R);
    edge_sw1 = hal_io_button_edge_pressed(HAL_IO_BTN_SW1);
    edge_sw2 = hal_io_button_edge_pressed(HAL_IO_BTN_SW2);

    enc_l_now = hal_io_encoder_count(HAL_IO_ENC_L);
    enc_r_now = hal_io_encoder_count(HAL_IO_ENC_R);
    d_l = enc_l_now - g_last_enc_l;
    d_r = enc_r_now - g_last_enc_r;
    g_last_enc_l = enc_l_now;
    g_last_enc_r = enc_r_now;

    if (g_app_mode != APP_MENU && edge_enc_l) {
      app_enter(APP_MENU);
    } else if (g_app_mode == APP_MENU) {
      update_menu(d_l, edge_enc_r);
    } else if (g_app_mode == APP_HRDW_TEST) {
      update_hrdw_test(d_l, d_r, edge_sw1);
      (void)edge_enc_r;
      (void)edge_sw2;
    } else if (g_app_mode == APP_CALIBRATION) {
      update_calibration(d_l, d_r, edge_enc_r);
      (void)edge_sw1;
    } else if (g_app_mode == APP_GRIDS) {
      update_grids(d_l, d_r, edge_enc_r, now_ms);
      (void)edge_sw1;
      (void)edge_sw2;
    } else if (g_app_mode == APP_TRIGSEQ) {
      update_trigseq(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms);
    } else {
      update_euclid(d_l, d_r, edge_enc_r, edge_sw1, now_ms);
      (void)edge_sw2;
    }

    if ((now_ms - last_draw_ms) >= DRAW_PERIOD_MS) {
      // Drawing can block long enough to stretch output pulses. Skip this frame
      // while any pulse is active; render right after pulses fall low.
      if (active_app_has_live_pulse()) {
        service_active_app_pulses(to_ms_since_boot(get_absolute_time()));
        sleep_ms(LOOP_SLEEP_MS);
        tight_loop_contents();
        continue;
      }

      if (g_app_mode == APP_MENU) {
        draw_menu();
      } else if (g_app_mode == APP_HRDW_TEST) {
        draw_hrdw_test();
      } else if (g_app_mode == APP_CALIBRATION) {
        draw_calibration();
      } else if (g_app_mode == APP_GRIDS) {
        draw_grids();
      } else if (g_app_mode == APP_TRIGSEQ) {
        draw_trigseq();
      } else {
        draw_euclid();
      }
      last_draw_ms = now_ms;

      // Rendering can take noticeable time. Service pulse timeouts again right after draw
      // so output pulse width is not stretched by OLED refresh time.
      service_active_app_pulses(to_ms_since_boot(get_absolute_time()));
    }

    sleep_ms(LOOP_SLEEP_MS);
    tight_loop_contents();
  }
}
