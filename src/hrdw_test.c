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
#define LOOP_SLEEP_MS 2u
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

typedef enum {
  APP_MENU = 0,
  APP_HRDW_TEST = 1,
  APP_CALIBRATION = 2,
  APP_GRIDS = 3,
  APP_TRIGSEQ = 4,
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
  TRIGSEQ_PARAM_LEN = 0,
  TRIGSEQ_PARAM_CH = 1,
  TRIGSEQ_PARAM_STEP = 2,
  TRIGSEQ_PARAM_VAL = 3,
  TRIGSEQ_PARAM_RUN = 4,
  TRIGSEQ_PARAM_COUNT = 5,
} trigseq_param_t;

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
  bool outputs_ok;
  uint64_t status_until_ms;
  char status[16];
} grids_state_t;

typedef struct {
  trigseq_engine_t engine;
  bool engine_initialized;
  trigseq_param_t selected_param;
  uint8_t edit_channel;
  uint8_t edit_step;
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

static const char* k_menu_items[4] = {"HRDW_TEST", "CALIBRATION", "GRIDS", "TRIGSEQ"};

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
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static trigseq_state_t g_trigseq = {
    .engine_initialized = false,
    .selected_param = TRIGSEQ_PARAM_LEN,
    .edit_channel = 0u,
    .edit_step = 0u,
    .run = true,
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

  trigseq_engine_clock(&g_trigseq.engine, trig);
  for (uint8_t i = 0u; i < 4u; ++i) {
    if (trig[i]) {
      g_trigseq.trig_state[i] = true;
      g_trigseq.trig_off_ms[i] = now_ms + TRIGSEQ_TRIG_PULSE_MS;
    }
  }

  g_trigseq.step_count += 1u;
  trigseq_apply_outputs(false);
}

static void trigseq_reset_engine(void) {
  trigseq_engine_reset(&g_trigseq.engine);
  g_trigseq.step_count = 0u;
}

static void trigseq_enter(void) {
  if (!g_trigseq.engine_initialized) {
    trigseq_engine_init(&g_trigseq.engine, (uint32_t)to_us_since_boot(get_absolute_time()));
    g_trigseq.engine_initialized = true;
  }
  trigseq_reset_engine();
  g_trigseq.prev_clk_active = hal_io_trigger_active(HAL_IO_TR1);
  g_trigseq.prev_rst_active = hal_io_trigger_active(HAL_IO_TR2);
  trigseq_reset_outputs_and_state();
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
  trigseq_engine_set_length(&g_trigseq.engine, g_app_settings_data.trigseq.length);
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_trigseq.engine.pattern[i] = g_app_settings_data.trigseq.pattern[i];
  }
  g_trigseq.edit_channel = (uint8_t)(g_app_settings_data.trigseq.edit_channel & 0x03u);
  g_trigseq.edit_step = g_app_settings_data.trigseq.edit_step;
  if (g_trigseq.edit_step >= trigseq_engine_get_length(&g_trigseq.engine)) {
    g_trigseq.edit_step = (uint8_t)(trigseq_engine_get_length(&g_trigseq.engine) - 1u);
  }
  g_trigseq.run = g_app_settings_data.trigseq.run != 0u;
}

static void capture_runtime_to_app_settings(void) {
  g_app_settings_data.grids.clock_mode = (g_grids.clock == GRIDS_CLOCK_INT) ? 0u : 1u;
  g_app_settings_data.grids.bpm = (uint16_t)clamp_i(g_grids.bpm, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
  g_app_settings_data.grids.map_x = g_grids.map_x;
  g_app_settings_data.grids.map_y = g_grids.map_y;
  g_app_settings_data.grids.chaos = g_grids.chaos;

  g_app_settings_data.trigseq.length = trigseq_engine_get_length(&g_trigseq.engine);
  g_app_settings_data.trigseq.edit_channel = g_trigseq.edit_channel;
  g_app_settings_data.trigseq.edit_step = g_trigseq.edit_step;
  g_app_settings_data.trigseq.run = g_trigseq.run ? 1u : 0u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.trigseq.pattern[i] = g_trigseq.engine.pattern[i];
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
      g_grids.trig_state[i] = true;
      g_grids.trig_off_ms[i] = now_ms + GRIDS_TRIG_PULSE_MS;
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
  set_encoder_reference_now();

  if (mode == APP_HRDW_TEST) {
    g_hrdw.dac_ok = app_write_outputs_mv(g_hrdw.dac_mv);
  } else if (mode == APP_CALIBRATION) {
    g_hrdw.dac_ok = apply_calibration_preview();
    set_cal_status("CAL MODE");
  } else if (mode == APP_GRIDS) {
    grids_enter();
  } else if (mode == APP_TRIGSEQ) {
    trigseq_enter();
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
    } else {
      app_enter(APP_TRIGSEQ);
    }
  }
}

static void update_hrdw_test(int32_t d_l, int32_t d_r, bool edge_sw_a) {
  if (edge_sw_a) {
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

static void update_trigseq(int32_t d_l, int32_t d_r, bool edge_enc_r, uint64_t now_ms) {
  bool clk_active;
  bool rst_active;

  if (d_l != 0) {
    int next = (int)g_trigseq.selected_param + (int)d_l;
    while (next < 0) next += (int)TRIGSEQ_PARAM_COUNT;
    while (next >= (int)TRIGSEQ_PARAM_COUNT) next -= (int)TRIGSEQ_PARAM_COUNT;
    g_trigseq.selected_param = (trigseq_param_t)next;
  }

  if (d_r != 0) {
    if (g_trigseq.selected_param == TRIGSEQ_PARAM_LEN) {
      int len = (int)trigseq_engine_get_length(&g_trigseq.engine) + (int)d_r;
      len = clamp_i(len, 4, 32);
      trigseq_engine_set_length(&g_trigseq.engine, (uint8_t)len);
      if (g_trigseq.edit_step >= (uint8_t)len) g_trigseq.edit_step = (uint8_t)(len - 1);
    } else if (g_trigseq.selected_param == TRIGSEQ_PARAM_CH) {
      int ch = (int)g_trigseq.edit_channel + (int)d_r;
      while (ch < 0) ch += 4;
      while (ch >= 4) ch -= 4;
      g_trigseq.edit_channel = (uint8_t)ch;
    } else if (g_trigseq.selected_param == TRIGSEQ_PARAM_STEP) {
      int len = (int)trigseq_engine_get_length(&g_trigseq.engine);
      int step = (int)g_trigseq.edit_step + (int)d_r;
      while (step < 0) step += len;
      while (step >= len) step -= len;
      g_trigseq.edit_step = (uint8_t)step;
    } else if (g_trigseq.selected_param == TRIGSEQ_PARAM_VAL) {
      bool cur =
          trigseq_engine_get_step_bit(&g_trigseq.engine, g_trigseq.edit_channel, g_trigseq.edit_step);
      if (d_r > 0) {
        cur = true;
      } else {
        cur = false;
      }
      trigseq_engine_set_step_bit(&g_trigseq.engine, g_trigseq.edit_channel, g_trigseq.edit_step, cur);
    } else if (g_trigseq.selected_param == TRIGSEQ_PARAM_RUN) {
      g_trigseq.run = d_r > 0;
      if (!g_trigseq.run) {
        trigseq_reset_outputs_and_state();
      }
    }
  }

  rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (rst_active && !g_trigseq.prev_rst_active) {
    trigseq_reset_engine();
  }
  g_trigseq.prev_rst_active = rst_active;

  clk_active = hal_io_trigger_active(HAL_IO_TR1);
  if (g_trigseq.run && clk_active && !g_trigseq.prev_clk_active) {
    trigseq_do_step(now_ms);
  }
  g_trigseq.prev_clk_active = clk_active;

  trigseq_update_pulses(now_ms);

  if (edge_enc_r) {
    save_trigseq_settings();
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
    snprintf(line, sizeof(line), "VIN %+1ld.%03ldV SW_A AUT", (long)(g_hrdw.mux_mv_manual / 1000),
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

  snprintf(line, sizeof(line), "ELS:%u ERS:%u A:%u", hal_io_button_pressed(HAL_IO_BTN_ENC_L) ? 1u : 0u,
           hal_io_button_pressed(HAL_IO_BTN_ENC_R) ? 1u : 0u,
           hal_io_button_pressed(HAL_IO_BTN_SW_A) ? 1u : 0u);
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
  bool show_status =
      (g_grids.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_grids.status[0] != '\0');

  hal_io_oled_draw_line(0, "GRIDS", true);
  snprintf(line, sizeof(line), "CLK %s BPM %3d", g_grids.clock == GRIDS_CLOCK_INT ? "INT" : "EXT", g_grids.bpm);
  hal_io_oled_draw_line(1, line, false);

  snprintf(line, sizeof(line), "%c CLOCK %s", g_grids.selected_param == GRIDS_PARAM_CLOCK ? '>' : ' ',
           g_grids.clock == GRIDS_CLOCK_INT ? "INT" : "EXT");
  hal_io_oled_draw_line(3, line, g_grids.selected_param == GRIDS_PARAM_CLOCK);

  snprintf(line, sizeof(line), "%c BPM %3d %s", g_grids.selected_param == GRIDS_PARAM_BPM ? '>' : ' ',
           g_grids.bpm, g_grids.clock == GRIDS_CLOCK_INT ? "" : "(EXT)");
  hal_io_oled_draw_line(4, line, g_grids.selected_param == GRIDS_PARAM_BPM);

  snprintf(line, sizeof(line), "%c MAPX %3u", g_grids.selected_param == GRIDS_PARAM_MAP_X ? '>' : ' ',
           (unsigned)g_grids.map_x);
  hal_io_oled_draw_line(5, line, g_grids.selected_param == GRIDS_PARAM_MAP_X);

  snprintf(line, sizeof(line), "%c MAPY %3u", g_grids.selected_param == GRIDS_PARAM_MAP_Y ? '>' : ' ',
           (unsigned)g_grids.map_y);
  hal_io_oled_draw_line(6, line, g_grids.selected_param == GRIDS_PARAM_MAP_Y);

  snprintf(line, sizeof(line), "%c CHAOS %3u", g_grids.selected_param == GRIDS_PARAM_CHAOS ? '>' : ' ',
           (unsigned)g_grids.chaos);
  hal_io_oled_draw_line(7, line, g_grids.selected_param == GRIDS_PARAM_CHAOS);

  snprintf(line, sizeof(line), "F %3u %3u %3u %3u", (unsigned)g_grids.fill_u8[0], (unsigned)g_grids.fill_u8[1],
           (unsigned)g_grids.fill_u8[2], (unsigned)g_grids.fill_u8[3]);
  hal_io_oled_draw_line(9, line, false);

  snprintf(line, sizeof(line), "T %u %u %u %u ST%3lu", g_grids.trig_state[0] ? 1u : 0u,
           g_grids.trig_state[1] ? 1u : 0u, g_grids.trig_state[2] ? 1u : 0u,
           g_grids.trig_state[3] ? 1u : 0u, (unsigned long)(g_grids.step_count & 0x1FFu));
  hal_io_oled_draw_line(10, line, false);

  snprintf(line, sizeof(line), "TR1:%s TR2:%s", hal_io_trigger_active(HAL_IO_TR1) ? "HIGH" : "LOW",
           hal_io_trigger_active(HAL_IO_TR2) ? "HIGH" : "LOW");
  hal_io_oled_draw_line(11, line, false);

  snprintf(line, sizeof(line), "%s", g_grids.outputs_ok ? "OUT OK" : "OUT ERR");
  hal_io_oled_draw_line(12, line, false);
  if (show_status) {
    hal_io_oled_draw_line(13, g_grids.status, false);
  } else {
    hal_io_oled_draw_line(13, "", false);
  }
  hal_io_oled_draw_line(14, "E1 PARAM E2 VALUE", false);
  hal_io_oled_draw_line(15, "ER_SW SAVE E1_SW BK", false);
}

static void draw_trigseq(void) {
  char line[32];
  uint8_t len = trigseq_engine_get_length(&g_trigseq.engine);
  bool bit = trigseq_engine_get_step_bit(&g_trigseq.engine, g_trigseq.edit_channel, g_trigseq.edit_step);
  int step_pos = (int)g_trigseq.engine.step + 1;
  bool show_status =
      (g_trigseq.status_until_ms > to_ms_since_boot(get_absolute_time())) &&
      (g_trigseq.status[0] != '\0');

  hal_io_oled_draw_line(0, "TRIGSEQ 4CH", true);
  snprintf(line, sizeof(line), "CLK:TR1 RST:TR2 ST:%2d/%2u", step_pos, (unsigned)len);
  hal_io_oled_draw_line(1, line, false);

  snprintf(line, sizeof(line), "%c LEN %2u", g_trigseq.selected_param == TRIGSEQ_PARAM_LEN ? '>' : ' ',
           (unsigned)len);
  hal_io_oled_draw_line(3, line, g_trigseq.selected_param == TRIGSEQ_PARAM_LEN);

  snprintf(line, sizeof(line), "%c CH  %c", g_trigseq.selected_param == TRIGSEQ_PARAM_CH ? '>' : ' ',
           (char)('A' + g_trigseq.edit_channel));
  hal_io_oled_draw_line(4, line, g_trigseq.selected_param == TRIGSEQ_PARAM_CH);

  snprintf(line, sizeof(line), "%c STEP %2u", g_trigseq.selected_param == TRIGSEQ_PARAM_STEP ? '>' : ' ',
           (unsigned)(g_trigseq.edit_step + 1u));
  hal_io_oled_draw_line(5, line, g_trigseq.selected_param == TRIGSEQ_PARAM_STEP);

  snprintf(line, sizeof(line), "%c VAL  %u", g_trigseq.selected_param == TRIGSEQ_PARAM_VAL ? '>' : ' ',
           bit ? 1u : 0u);
  hal_io_oled_draw_line(6, line, g_trigseq.selected_param == TRIGSEQ_PARAM_VAL);

  snprintf(line, sizeof(line), "%c RUN  %s", g_trigseq.selected_param == TRIGSEQ_PARAM_RUN ? '>' : ' ',
           g_trigseq.run ? "ON" : "OFF");
  hal_io_oled_draw_line(7, line, g_trigseq.selected_param == TRIGSEQ_PARAM_RUN);

  snprintf(line, sizeof(line), "OUT %u %u %u %u", g_trigseq.trig_state[0] ? 1u : 0u,
           g_trigseq.trig_state[1] ? 1u : 0u, g_trigseq.trig_state[2] ? 1u : 0u,
           g_trigseq.trig_state[3] ? 1u : 0u);
  hal_io_oled_draw_line(9, line, false);

  snprintf(line, sizeof(line), "TR1:%s TR2:%s", hal_io_trigger_active(HAL_IO_TR1) ? "HIGH" : "LOW",
           hal_io_trigger_active(HAL_IO_TR2) ? "HIGH" : "LOW");
  hal_io_oled_draw_line(10, line, false);

  snprintf(line, sizeof(line), "%s", g_trigseq.outputs_ok ? "OUT OK" : "OUT ERR");
  hal_io_oled_draw_line(11, line, false);
  if (show_status) {
    hal_io_oled_draw_line(12, g_trigseq.status, false);
  } else {
    hal_io_oled_draw_line(12, "", false);
  }
  hal_io_oled_draw_line(13, "E1 PARAM  E2 VALUE", false);
  hal_io_oled_draw_line(14, "LEN 4..32 / 4CH", false);
  hal_io_oled_draw_line(15, "ER_SW SAVE E1_SW BK", false);
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
    bool edge_sw_a;
    bool edge_sw_b_unused;

    now_ms = to_ms_since_boot(get_absolute_time());
    hal_io_poll(now_ms);

    edge_enc_l = hal_io_button_edge_pressed(HAL_IO_BTN_ENC_L);
    edge_enc_r = hal_io_button_edge_pressed(HAL_IO_BTN_ENC_R);
    edge_sw_a = hal_io_button_edge_pressed(HAL_IO_BTN_SW_A);
    edge_sw_b_unused = hal_io_button_edge_pressed(HAL_IO_BTN_SW_B);

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
      update_hrdw_test(d_l, d_r, edge_sw_a);
      (void)edge_enc_r;
      (void)edge_sw_b_unused;
    } else if (g_app_mode == APP_CALIBRATION) {
      update_calibration(d_l, d_r, edge_enc_r);
      (void)edge_sw_a;
    } else if (g_app_mode == APP_GRIDS) {
      update_grids(d_l, d_r, edge_enc_r, now_ms);
      (void)edge_sw_a;
      (void)edge_sw_b_unused;
    } else {
      update_trigseq(d_l, d_r, edge_enc_r, now_ms);
      (void)edge_sw_a;
      (void)edge_sw_b_unused;
    }

    if ((now_ms - last_draw_ms) >= DRAW_PERIOD_MS) {
      if (g_app_mode == APP_MENU) {
        draw_menu();
      } else if (g_app_mode == APP_HRDW_TEST) {
        draw_hrdw_test();
      } else if (g_app_mode == APP_CALIBRATION) {
        draw_calibration();
      } else if (g_app_mode == APP_GRIDS) {
        draw_grids();
      } else {
        draw_trigseq();
      }
      last_draw_ms = now_ms;
    }

    sleep_ms(LOOP_SLEEP_MS);
    tight_loop_contents();
  }
}
