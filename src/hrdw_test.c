#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_presets.h"
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
#define TR2GATE_GATE_MIN_CS 1u
#define TR2GATE_GATE_MAX_CS 1000u
#define TR2ADSR_TIME_MIN_DS 1u
#define TR2ADSR_TIME_MAX_DS 100u
#define BURSTGEN_BPM_MIN 40
#define BURSTGEN_BPM_MAX 220
#define BURSTGEN_SWING_MAX 40u
#define BURSTGEN_PULSE_MS 35u
#define CVGEN_RATE_MIN_DHZ 1u
#define CVGEN_RATE_MAX_DHZ 100u
#define CVGEN_VOLT_MIN_MV (-10000)
#define CVGEN_VOLT_MAX_MV 10000
#define CVGEN_VOLT_STEP_MV 100
#define CVGEN_MIN_SPAN_MV 100
#define CVGEN_SAMPLE_RATE_HZ 200.0f
#define CVGEN_TWO_PI 6.28318530718f

typedef enum {
  APP_MENU = 0,
  APP_HRDW_TEST = 1,
  APP_CALIBRATION = 2,
  APP_GRIDS = 3,
  APP_TRIGSEQ = 4,
  APP_EUCLID = 5,
  APP_TR2GATE = 6,
  APP_TR2ADSR = 7,
  APP_BURSTGEN = 8,
  APP_CVGEN = 9,
} app_mode_t;

typedef enum {
  APP_SCREEN_MAIN = 0,
  APP_SCREEN_TRIGSEQ_GRID = 1,
  APP_SCREEN_MAX = 4,
} app_screen_t;

typedef struct {
  app_screen_t screen;
  bool menu_open;
  bool popup_dirty;
  uint8_t menu_sel;
  uint8_t slot_sel;
} preset_ui_state_t;

typedef enum {
  CAL_PARAM_MODE = 0,
  CAL_PARAM_CHANNEL = 1,
  CAL_PARAM_POINT = 2,
  CAL_PARAM_CODE = 3,
  CAL_PARAM_COUNT = 4,
} cal_param_t;

typedef enum {
  GRIDS_PARAM_CLOCK = 0,
  GRIDS_PARAM_BPM = 1,
  GRIDS_PARAM_MAP_X = 2,
  GRIDS_PARAM_MAP_Y = 3,
  GRIDS_PARAM_CHAOS = 4,
  GRIDS_PARAM_PROB1 = 5,
  GRIDS_PARAM_PROB2 = 6,
  GRIDS_PARAM_PROB3 = 7,
  GRIDS_PARAM_PROB4 = 8,
  GRIDS_PARAM_COUNT = 9,
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
  TRIGSEQ_PARAM_PROB1 = 3,
  TRIGSEQ_PARAM_PROB2 = 4,
  TRIGSEQ_PARAM_PROB3 = 5,
  TRIGSEQ_PARAM_PROB4 = 6,
  TRIGSEQ_PARAM_COUNT = 7,
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
  EUCLID_PARAM_CH1_PRB = 4,
  EUCLID_PARAM_CH2_STEPS = 5,
  EUCLID_PARAM_CH2_HITS = 6,
  EUCLID_PARAM_CH2_PRB = 7,
  EUCLID_PARAM_CH3_STEPS = 8,
  EUCLID_PARAM_CH3_HITS = 9,
  EUCLID_PARAM_CH3_PRB = 10,
  EUCLID_PARAM_CH4_STEPS = 11,
  EUCLID_PARAM_CH4_HITS = 12,
  EUCLID_PARAM_CH4_PRB = 13,
  EUCLID_PARAM_COUNT = 14,
} euclid_param_t;

typedef enum {
  TR2GATE_PARAM_MODE = 0,
  TR2GATE_PARAM_CHANNEL = 1,
  TR2GATE_PARAM_SOURCE = 2,
  TR2GATE_PARAM_TIME = 3,
  TR2GATE_PARAM_PROB = 4,
  TR2GATE_PARAM_LEVEL = 5,
  TR2GATE_PARAM_COUNT = 6,
} tr2gate_param_t;

typedef enum {
  TR2GATE_MODE_DIRECT = 0,
  TR2GATE_MODE_ROUND_ROBIN = 1,
} tr2gate_mode_t;

typedef enum {
  TR2ADSR_ENV_AR = 0,
  TR2ADSR_ENV_ASR = 1,
  TR2ADSR_ENV_ADSR = 2,
} tr2adsr_env_type_t;

typedef enum {
  TR2ADSR_PARAM_CHANNEL = 0,
  TR2ADSR_PARAM_SOURCE = 1,
  TR2ADSR_PARAM_TYPE = 2,
  TR2ADSR_PARAM_TIME = 3,
  TR2ADSR_PARAM_PROB = 4,
  TR2ADSR_PARAM_LEVEL = 5,
  TR2ADSR_PARAM_ATTACK = 6,
  TR2ADSR_PARAM_DECAY = 7,
  TR2ADSR_PARAM_SUSTAIN = 8,
  TR2ADSR_PARAM_RELEASE = 9,
  TR2ADSR_PARAM_COUNT = 10,
} tr2adsr_param_t;

typedef enum {
  TR2ADSR_STATE_IDLE = 0,
  TR2ADSR_STATE_ATTACK = 1,
  TR2ADSR_STATE_DECAY = 2,
  TR2ADSR_STATE_SUSTAIN = 3,
  TR2ADSR_STATE_RELEASE = 4,
} tr2adsr_env_state_t;

typedef enum {
  BURSTGEN_PARAM_CHANNEL = 0,
  BURSTGEN_PARAM_SIGNATURE = 1,
  BURSTGEN_PARAM_BPM = 2,
  BURSTGEN_PARAM_SWING = 3,
  BURSTGEN_PARAM_PROB = 4,
  BURSTGEN_PARAM_LEVEL = 5,
  BURSTGEN_PARAM_COUNT = 6,
} burstgen_param_t;

typedef enum {
  CVGEN_ALGO_STEP = 0,
  CVGEN_ALGO_BEZIER = 1,
  CVGEN_ALGO_OCEAN = 2,
  CVGEN_ALGO_WALK = 3,
} cvgen_algo_t;

typedef enum {
  CVGEN_CLOCK_INT = 0,
  CVGEN_CLOCK_EXT = 1,
} cvgen_clock_t;

typedef enum {
  CVGEN_WALK_MODE_WALK = 0,
  CVGEN_WALK_MODE_SNH = 1,
  CVGEN_WALK_MODE_TNH = 2,
} cvgen_walk_mode_t;

typedef enum {
  CVGEN_PARAM_ALGO = 0,
  CVGEN_PARAM_CLOCK = 1,
  CVGEN_PARAM_SOURCE = 2,
  CVGEN_PARAM_RATE = 3,
  CVGEN_PARAM_MIN = 4,
  CVGEN_PARAM_MAX = 5,
  CVGEN_PARAM_A = 6,
  CVGEN_PARAM_B = 7,
  CVGEN_PARAM_COUNT = 8,
} cvgen_param_t;

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
  uint8_t prob[4];
  uint16_t fill_raw[4];
  int32_t fill_mv[4];
  uint8_t fill_u8[4];
  uint32_t prob_rng_state;
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
  uint8_t prob[4];
  uint32_t prob_rng_state;
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
  uint8_t prob[4];
  uint32_t prob_rng_state;
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

typedef struct {
  tr2gate_param_t selected_param;
  tr2gate_mode_t mode;
  uint8_t selected_channel;
  uint8_t source[4];
  uint16_t gate_time_cs[4];
  uint8_t prob[4];
  bool level_10v;
  bool src_active[4];
  bool gate_out[4];
  uint64_t gate_off_ms[4];
  uint8_t rr_channel;
  bool outputs_ok;
  uint64_t status_until_ms;
  char status[16];
} tr2gate_state_t;

typedef struct {
  tr2adsr_param_t selected_param;
  uint8_t selected_channel;
  uint8_t source[4];
  tr2adsr_env_type_t type[4];
  uint16_t total_time_ds[4];
  uint8_t prob[4];
  bool level_10v;
  uint8_t attack_pct[4];
  uint8_t decay_pct[4];
  uint8_t sustain_pct[4];
  uint8_t release_pct[4];
  bool src_active[4];
  tr2adsr_env_state_t env_state[4];
  float level[4];
  float release_start[4];
  float latched_sustain[4];
  uint8_t latched_attack_pct[4];
  uint8_t latched_decay_pct[4];
  uint8_t latched_release_pct[4];
  int32_t cv_mv[4];
  float cv_norm[4];
  uint64_t gate_hold_until_ms[4];
  uint64_t state_start_ms[4];
  uint16_t latched_total_ms[4];
  bool outputs_ok;
  uint64_t status_until_ms;
  char status[16];
} tr2adsr_state_t;

typedef struct {
  burstgen_param_t selected_param;
  uint8_t selected_channel;
  uint8_t signature_mode;
  uint16_t bpm;
  uint8_t swing_pct;
  uint8_t probability;
  bool level_10v;
  bool src_active[4];
  bool running[4];
  bool gate_out[4];
  uint8_t pattern_mask[4];
  uint8_t current_step[4];
  uint64_t next_step_at_ms[4];
  uint64_t gate_off_at_ms[4];
  uint32_t rng_state;
  bool outputs_ok;
  uint64_t status_until_ms;
  char status[16];
} burstgen_state_t;

typedef struct {
  cvgen_param_t selected_param;
  cvgen_algo_t algo;
  cvgen_clock_t clock_mode;
  uint8_t source;
  uint16_t rate_dhz;
  int16_t min_mv;
  int16_t max_mv;
  uint8_t param1;
  uint8_t param2;
  bool prev_src_active;
  uint64_t last_update_ms;
  uint64_t last_edge_ms;
  uint64_t segment_start_ms;
  uint32_t segment_duration_ms;
  float output_norm;
  float step_target_norm;
  float bez_y0;
  float bez_y1;
  float bez_c1;
  float bez_c2;
  float ocean_t;
  float ocean_phase_a;
  float ocean_phase_b;
  float ocean_rate_wobble;
  float ocean_mix;
  float ocean_mix_target;
  float walk_last;
  float walk_last_out;
  float walk_filter_state;
  float walk_filter_alpha;
  float walk_damp;
  float walk_noise_step;
  float hold_norm;
} cvgen_channel_state_t;

typedef struct {
  cvgen_channel_state_t ch[4];
  uint32_t rng_state;
  bool outputs_ok;
  uint64_t status_until_ms;
  char status[16];
} cvgen_state_t;

static const char* k_menu_items[9] = {
    "HRDW_TEST", "CALIBRATION", "GRIDS", "TRIG SEQ", "4XEUCLID", "TR2GATE", "TR2ADSR", "BURST GEN", "CV GEN"};

static app_mode_t g_app_mode = APP_MENU;
static int g_menu_index = 0;
static preset_ui_state_t g_preset_ui = {
    .screen = APP_SCREEN_MAIN,
    .menu_open = false,
    .popup_dirty = false,
    .menu_sel = 0u,
    .slot_sel = 0u,
};

static calibration_data_t g_calibration_data;
static bool g_calibration_loaded = false;
static bool g_calibration_dirty = false;
static app_settings_data_t g_app_settings_data;
static bool g_app_settings_loaded = false;

static cal_param_t g_cal_param = CAL_PARAM_MODE;
static uint8_t g_cal_channel = 0;
static cal_point_t g_cal_point = CAL_POINT_MID_LOW;

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
    .prob = {100u, 100u, 100u, 100u},
    .prob_rng_state = 0x4D595DF4u,
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
    .prob = {100u, 100u, 100u, 100u},
    .prob_rng_state = 0x2A1F6C3Du,
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
  .prob = {100u, 100u, 100u, 100u},
  .prob_rng_state = 0x7A31C9E5u,
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

static tr2gate_state_t g_tr2gate = {
    .selected_param = TR2GATE_PARAM_MODE,
    .mode = TR2GATE_MODE_DIRECT,
    .selected_channel = 0u,
    .source = {0u, 1u, 2u, 3u},
    .gate_time_cs = {50u, 50u, 50u, 50u},
    .prob = {100u, 100u, 100u, 100u},
    .level_10v = true,
    .src_active = {false, false, false, false},
    .gate_out = {false, false, false, false},
    .gate_off_ms = {0u, 0u, 0u, 0u},
    .rr_channel = 0u,
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static tr2adsr_state_t g_tr2adsr = {
    .selected_param = TR2ADSR_PARAM_CHANNEL,
    .selected_channel = 0u,
    .source = {0u, 1u, 2u, 3u},
    .type = {TR2ADSR_ENV_ADSR, TR2ADSR_ENV_ADSR, TR2ADSR_ENV_ADSR, TR2ADSR_ENV_ADSR},
    .total_time_ds = {10u, 10u, 10u, 10u},
    .prob = {100u, 100u, 100u, 100u},
    .level_10v = true,
    .attack_pct = {25u, 25u, 25u, 25u},
    .decay_pct = {25u, 25u, 25u, 25u},
    .sustain_pct = {70u, 70u, 70u, 70u},
    .release_pct = {25u, 25u, 25u, 25u},
    .src_active = {false, false, false, false},
    .env_state = {TR2ADSR_STATE_IDLE, TR2ADSR_STATE_IDLE, TR2ADSR_STATE_IDLE, TR2ADSR_STATE_IDLE},
    .level = {0.0f, 0.0f, 0.0f, 0.0f},
    .release_start = {0.0f, 0.0f, 0.0f, 0.0f},
    .latched_sustain = {0.7f, 0.7f, 0.7f, 0.7f},
    .gate_hold_until_ms = {0u, 0u, 0u, 0u},
    .state_start_ms = {0u, 0u, 0u, 0u},
    .latched_total_ms = {1000u, 1000u, 1000u, 1000u},
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static burstgen_state_t g_burstgen = {
    .selected_param = BURSTGEN_PARAM_CHANNEL,
    .selected_channel = 0u,
    .signature_mode = 0u,
    .bpm = 120u,
    .swing_pct = 15u,
    .probability = 100u,
    .level_10v = true,
    .src_active = {false, false, false, false},
    .running = {false, false, false, false},
    .gate_out = {false, false, false, false},
    .pattern_mask = {0u, 0u, 0u, 0u},
    .current_step = {0u, 0u, 0u, 0u},
    .next_step_at_ms = {0u, 0u, 0u, 0u},
    .gate_off_at_ms = {0u, 0u, 0u, 0u},
    .rng_state = 0x53A91C27u,
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static cvgen_state_t g_cvgen = {
    .rng_state = 0x6C4E1D29u,
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static int32_t clamp_mv(int32_t mv) {
  return calibration_clamp_voltage_mv(&g_calibration_data, mv);
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

static uint8_t clamp_u8i_100(int x) {
  if (x < 0) return 0u;
  if (x > 100) return 100u;
  return (uint8_t)x;
}

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint8_t prob_rng8(uint32_t* state);
static bool prob_pass(uint8_t prob_percent, uint32_t* rng_state);

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

static void set_tr2gate_status(const char* s) {
  snprintf(g_tr2gate.status, sizeof(g_tr2gate.status), "%s", s);
  g_tr2gate.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static void set_tr2adsr_status(const char* s) {
  snprintf(g_tr2adsr.status, sizeof(g_tr2adsr.status), "%s", s);
  g_tr2adsr.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static void set_burstgen_status(const char* s) {
  snprintf(g_burstgen.status, sizeof(g_burstgen.status), "%s", s);
  g_burstgen.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static uint8_t wrap_u8_delta(uint8_t value, int32_t delta, uint8_t count) {
  int next = (int)value + (int)delta;
  while (next < 0) next += (int)count;
  while (next >= (int)count) next -= (int)count;
  return (uint8_t)next;
}

static const char* tr2gate_mode_label(tr2gate_mode_t mode) {
  return (mode == TR2GATE_MODE_ROUND_ROBIN) ? "ROUND" : "DIRECT";
}

static const char* tr2adsr_type_label(tr2adsr_env_type_t type) {
  if (type == TR2ADSR_ENV_AR) return "AR";
  if (type == TR2ADSR_ENV_ASR) return "ASR";
  return "ADSR";
}

static const char* tr2adsr_state_label(tr2adsr_env_state_t state) {
  if (state == TR2ADSR_STATE_ATTACK) return "ATK";
  if (state == TR2ADSR_STATE_DECAY) return "DEC";
  if (state == TR2ADSR_STATE_SUSTAIN) return "SUS";
  if (state == TR2ADSR_STATE_RELEASE) return "REL";
  return "IDL";
}

static const char* burstgen_signature_label(uint8_t mode) {
  if (mode == 3u) return "3/3";
  if (mode == 1u) return "6/6";
  if (mode == 2u) return "8/8";
  return "4/4";
}

static uint8_t burstgen_steps_for_mode(uint8_t mode) {
  if (mode == 3u) return 3u;
  if (mode == 1u) return 6u;
  if (mode == 2u) return 8u;
  return 4u;
}

static uint8_t burstgen_max_hits_for_mode(uint8_t mode) {
  if (mode == 3u) return 3u;
  if (mode == 1u) return 5u;
  if (mode == 2u) return 6u;
  return 4u;
}

static uint8_t cvgen_walk_mode_from_param(uint8_t param) {
  if (param < 34u) return CVGEN_WALK_MODE_WALK;
  if (param < 67u) return CVGEN_WALK_MODE_SNH;
  return CVGEN_WALK_MODE_TNH;
}

static uint8_t cvgen_walk_mode_to_param(uint8_t mode) {
  if (mode == CVGEN_WALK_MODE_SNH) return 50u;
  if (mode == CVGEN_WALK_MODE_TNH) return 100u;
  return 0u;
}

static uint8_t burstgen_rng8(void) {
  return prob_rng8(&g_burstgen.rng_state);
}

static uint8_t burstgen_popcount(uint8_t value) {
  uint8_t bits = 0u;
  while (value != 0u) {
    bits = (uint8_t)(bits + (value & 1u));
    value >>= 1u;
  }
  return bits;
}

static uint8_t burstgen_pick_pattern_mask(uint8_t steps, uint8_t max_hits) {
  uint8_t max_mask = (uint8_t)((1u << steps) - 1u);
  for (;;) {
    uint8_t mask = (uint8_t)(1u + (burstgen_rng8() % max_mask));
    if (burstgen_popcount(mask) <= max_hits) return mask;
  }
}

static bool burstgen_pattern_hit(uint8_t pattern_mask, uint8_t steps, uint8_t step_index) {
  uint8_t bit = (uint8_t)(steps - 1u - step_index);
  return ((pattern_mask >> bit) & 1u) != 0u;
}

static uint32_t burstgen_step_interval_ms(uint8_t step_index, uint32_t base_step_ms) {
  uint32_t swing = (uint32_t)clamp_i((int)g_burstgen.swing_pct, 0, (int)BURSTGEN_SWING_MAX);
  uint32_t swing_amount = (swing == 0u) ? 0u : (uint32_t)(burstgen_rng8() % (swing + 1u));
  uint32_t factor_pct = (step_index & 1u) == 0u ? (100u - swing_amount) : (100u + swing_amount);
  uint32_t interval = (base_step_ms * factor_pct) / 100u;
  return interval == 0u ? 1u : interval;
}

static void burstgen_format_pattern(char* out, size_t out_sz, uint8_t pattern_mask, uint8_t steps) {
  size_t idx = 0u;
  if (out_sz == 0u) return;
  while (idx < (size_t)steps && (idx + 1u) < out_sz) {
    out[idx] = burstgen_pattern_hit(pattern_mask, steps, (uint8_t)idx) ? 'x' : '-';
    ++idx;
  }
  out[idx] = '\0';
}

static void set_cvgen_status(const char* s) {
  snprintf(g_cvgen.status, sizeof(g_cvgen.status), "%s", s);
  g_cvgen.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static bool read_trigger_active_src(uint8_t src);

static const char* cvgen_algo_label(cvgen_algo_t algo) {
  if (algo == CVGEN_ALGO_BEZIER) return "BEZIER";
  if (algo == CVGEN_ALGO_OCEAN) return "OCEAN";
  if (algo == CVGEN_ALGO_WALK) return "WALK";
  return "STEP";
}

static const char* cvgen_clock_label(cvgen_clock_t mode) {
  return mode == CVGEN_CLOCK_EXT ? "EXT" : "INT";
}

static const char* cvgen_walk_mode_label(uint8_t mode) {
  if (mode == CVGEN_WALK_MODE_SNH) return "S&H";
  if (mode == CVGEN_WALK_MODE_TNH) return "T&H";
  return "WALK";
}

static const char* cvgen_param_a_label(cvgen_algo_t algo) {
  if (algo == CVGEN_ALGO_BEZIER) return "BEND";
  if (algo == CVGEN_ALGO_OCEAN) return "SWELL";
  if (algo == CVGEN_ALGO_WALK) return "CHG";
  return "SLEW";
}

static const char* cvgen_param_b_label(cvgen_algo_t algo) {
  if (algo == CVGEN_ALGO_BEZIER) return "SPAN";
  if (algo == CVGEN_ALGO_OCEAN) return "AGIT";
  if (algo == CVGEN_ALGO_WALK) return "MODE";
  return "SPAN";
}

static void cvgen_format_mv(char* out, size_t out_sz, int32_t mv) {
  int32_t abs_mv = mv < 0 ? -mv : mv;
  snprintf(out, out_sz, "%s%ld.%01ldV", mv < 0 ? "-" : "", (long)(abs_mv / 1000),
           (long)((abs_mv % 1000) / 100));
}

static float cvgen_rand_unit(uint32_t* state) {
  return (float)prob_rng8(state) / 255.0f;
}

static float cvgen_rand_signed(uint32_t* state) {
  return (cvgen_rand_unit(state) * 2.0f) - 1.0f;
}

static float cvgen_norm_from_mv(int32_t mv, int32_t min_mv, int32_t max_mv) {
  int32_t span = max_mv - min_mv;
  if (span <= 0) return 0.0f;
  return clampf((float)(mv - min_mv) / (float)span, 0.0f, 1.0f);
}

static int32_t cvgen_map_norm_to_mv(const cvgen_channel_state_t* channel, float norm) {
  float clamped = clampf(norm, 0.0f, 1.0f);
  if (channel->max_mv <= channel->min_mv) return channel->min_mv;
  return (int32_t)((float)channel->min_mv + clamped * (float)(channel->max_mv - channel->min_mv));
}

static float cvgen_bezier_value(float y0, float c1, float c2, float y1, float t) {
  float u = 1.0f - t;
  return (u * u * u * y0) + (3.0f * u * u * t * c1) + (3.0f * u * t * t * c2) + (t * t * t * y1);
}

static void cvgen_walk_set_params(cvgen_channel_state_t* channel) {
  float change = (float)channel->param1 / 100.0f;
  float cutoff = 0.8f + (change * 25.0f);
  float dt = 1.0f / CVGEN_SAMPLE_RATE_HZ;
  float rc = 1.0f / (2.0f * 3.14159265f * cutoff);
  channel->walk_filter_alpha = dt / (rc + dt);
  channel->walk_damp = 0.985f + (1.0f - change) * 0.014f;
  channel->walk_noise_step = 0.01f + (change * 0.28f);
}

static float cvgen_walk_next(cvgen_channel_state_t* channel, uint32_t* rng_state) {
  float delta = cvgen_rand_signed(rng_state) * channel->walk_noise_step;
  if ((channel->walk_last_out >= 1.0f && delta > 0.0f) || (channel->walk_last_out <= -1.0f && delta < 0.0f)) {
    delta = -delta;
  }
  channel->walk_last = clampf((channel->walk_damp * channel->walk_last) + delta, -1.0f, 1.0f);
  channel->walk_filter_state += channel->walk_filter_alpha * (channel->walk_last - channel->walk_filter_state);
  channel->walk_last_out = clampf(channel->walk_filter_state, -1.0f, 1.0f);
  return channel->walk_last_out;
}

static void cvgen_step_pick_target(cvgen_channel_state_t* channel, uint32_t* rng_state) {
  float spread = 0.05f + 0.95f * ((float)channel->param2 / 100.0f);
  channel->step_target_norm = clampf(0.5f + cvgen_rand_signed(rng_state) * 0.5f * spread, 0.0f, 1.0f);
}

static void cvgen_bezier_start_segment(cvgen_channel_state_t* channel, uint64_t now_ms, uint32_t* rng_state) {
  float bend = ((float)channel->param1 / 50.0f) - 1.0f;
  float span = 0.05f + 0.95f * ((float)channel->param2 / 100.0f);
  float target = clampf(0.5f + cvgen_rand_signed(rng_state) * 0.5f * span, 0.0f, 1.0f);
  float dy;

  channel->bez_y0 = channel->bez_y1;
  channel->bez_y1 = target;
  dy = channel->bez_y1 - channel->bez_y0;
  channel->bez_c1 = clampf(channel->bez_y0 + (dy * (0.2f + 0.6f * (1.0f - bend))), 0.0f, 1.0f);
  channel->bez_c2 = clampf(channel->bez_y1 - (dy * (0.2f + 0.6f * (1.0f + bend))), 0.0f, 1.0f);
  channel->segment_start_ms = now_ms;
}

static void cvgen_init_channel(cvgen_channel_state_t* channel, uint64_t now_ms, uint32_t* rng_state) {
  channel->prev_src_active = read_trigger_active_src(channel->source);
  channel->last_update_ms = now_ms;
  channel->last_edge_ms = now_ms;
  channel->segment_start_ms = now_ms;
  channel->segment_duration_ms = 1000u;
  channel->output_norm = 0.5f;
  channel->step_target_norm = 0.5f;
  channel->bez_y0 = 0.5f;
  channel->bez_y1 = 0.5f;
  channel->bez_c1 = 0.5f;
  channel->bez_c2 = 0.5f;
  channel->ocean_t = 0.0f;
  channel->ocean_phase_a = cvgen_rand_unit(rng_state) * CVGEN_TWO_PI;
  channel->ocean_phase_b = cvgen_rand_unit(rng_state) * CVGEN_TWO_PI;
  channel->ocean_rate_wobble = 0.0f;
  channel->ocean_mix = 0.25f;
  channel->ocean_mix_target = 0.25f;
  channel->walk_last = cvgen_rand_signed(rng_state) * 0.2f;
  channel->walk_last_out = channel->walk_last;
  channel->walk_filter_state = channel->walk_last;
  channel->hold_norm = 0.5f;
  cvgen_walk_set_params(channel);
  cvgen_step_pick_target(channel, rng_state);
  channel->bez_y0 = channel->step_target_norm;
  channel->bez_y1 = channel->step_target_norm;
}

static bool read_trigger_active_src(uint8_t src) {
  if (src == 1u) return hal_io_trigger_active(HAL_IO_TR2);
  if (src == 2u) return hal_io_trigger_active(HAL_IO_TR3);
  if (src == 3u) return hal_io_trigger_active(HAL_IO_TR4);
  return hal_io_trigger_active(HAL_IO_TR1);
}

static void sample_trigger_edges(bool* prev_active, bool* edge_out) {
  bool active[4];
  active[0] = hal_io_trigger_active(HAL_IO_TR1);
  active[1] = hal_io_trigger_active(HAL_IO_TR2);
  active[2] = hal_io_trigger_active(HAL_IO_TR3);
  active[3] = hal_io_trigger_active(HAL_IO_TR4);
  for (uint8_t i = 0u; i < 4u; ++i) {
    edge_out[i] = active[i] && !prev_active[i];
    prev_active[i] = active[i];
  }
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
    if (trig[i] && prob_pass(g_trigseq.prob[i], &g_trigseq.prob_rng_state)) {
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
  uint32_t now_us = (uint32_t)to_us_since_boot(get_absolute_time());
  if (!g_trigseq.engine_initialized) {
    trigseq_engine_init(&g_trigseq.engine, now_us);
    g_trigseq.engine_initialized = true;
  }
  g_trigseq.prob_rng_state = now_us ^ 0x5EED1234u;
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
    if (euclid_step_is_hit(g_euclid.steps[ch], g_euclid.hits[ch], step) &&
        prob_pass(g_euclid.prob[ch], &g_euclid.prob_rng_state)) {
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
  uint32_t now_us = (uint32_t)to_us_since_boot(get_absolute_time());
  g_euclid.focus = EUCLID_FOCUS_MENU;
  g_euclid.selected_param = EUCLID_PARAM_CLOCK;
  g_euclid.prob_rng_state = now_us ^ 0x1E7A4D99u;
  euclid_reset_engine();
  if (g_euclid.clock == EUCLID_CLOCK_INT) {
    g_euclid.next_int_tick_ms = now_ms + euclid_int_interval_ms();
  }
  g_euclid.prev_clk_active = hal_io_trigger_active(HAL_IO_TR1);
  g_euclid.prev_rst_active = hal_io_trigger_active(HAL_IO_TR2);
  euclid_reset_outputs_and_state();
  g_euclid.grid_cache_valid = false;
}

static void tr2gate_apply_outputs(void) {
  int32_t out_mv[4];
  int32_t high_mv = g_tr2gate.level_10v ? 10000 : 5000;
  for (uint8_t i = 0u; i < 4u; ++i) {
    out_mv[i] = g_tr2gate.gate_out[i] ? high_mv : 0;
  }
  g_tr2gate.outputs_ok = app_write_outputs_mv(out_mv);
}

static void tr2gate_reset_outputs_and_state(void) {
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_tr2gate.src_active[i] = read_trigger_active_src(i);
    g_tr2gate.gate_out[i] = false;
    g_tr2gate.gate_off_ms[i] = 0u;
  }
  g_tr2gate.rr_channel = 0u;
  tr2gate_apply_outputs();
}

static void tr2gate_enter(void) {
  tr2gate_reset_outputs_and_state();
}

static bool tr2adsr_param_visible(uint8_t ch, tr2adsr_param_t param) {
  if (param == TR2ADSR_PARAM_DECAY) {
    return g_tr2adsr.type[ch] == TR2ADSR_ENV_ADSR;
  }
  if (param == TR2ADSR_PARAM_SUSTAIN) {
    return g_tr2adsr.type[ch] != TR2ADSR_ENV_AR;
  }
  return true;
}

static tr2adsr_param_t tr2adsr_next_visible_param(uint8_t ch, tr2adsr_param_t current, int32_t delta) {
  int next = (int)current;
  do {
    next += (delta > 0) ? 1 : -1;
    while (next < 0) next += (int)TR2ADSR_PARAM_COUNT;
    while (next >= (int)TR2ADSR_PARAM_COUNT) next -= (int)TR2ADSR_PARAM_COUNT;
  } while (!tr2adsr_param_visible(ch, (tr2adsr_param_t)next));
  return (tr2adsr_param_t)next;
}

static void tr2adsr_refresh_cv_inputs(void) {
  for (uint8_t i = 0u; i < 4u; ++i) {
    uint16_t raw = hal_mux_adc_read_raw(i);
    int32_t mv = hal_mux_adc_raw_to_mv(raw);
    if (mv < 0) mv = 0;
    if (mv > 3300) mv = 3300;
    g_tr2adsr.cv_mv[i] = mv;
    g_tr2adsr.cv_norm[i] = (float)mv / 3300.0f;
  }
}

static uint8_t tr2adsr_cv_pct(uint8_t cv_index) {
  int pct = (g_tr2adsr.cv_mv[cv_index] * 100 + 1650) / 3300;
  return (uint8_t)clamp_i(pct, 0, 100);
}

static uint8_t tr2adsr_add_cv_pct(uint8_t base_pct, uint8_t cv_index, uint8_t max_value) {
  int scaled = (int)base_pct + (int)tr2adsr_cv_pct(cv_index);
  return (uint8_t)clamp_i(scaled, 0, (int)max_value);
}

static uint8_t tr2adsr_effective_attack_pct(uint8_t ch) {
  return tr2adsr_add_cv_pct(g_tr2adsr.attack_pct[ch], 0u, 100u);
}

static uint8_t tr2adsr_effective_decay_pct(uint8_t ch) {
  return tr2adsr_add_cv_pct(g_tr2adsr.decay_pct[ch], 1u, 100u);
}

static uint8_t tr2adsr_effective_sustain_pct(uint8_t ch) {
  return tr2adsr_add_cv_pct(g_tr2adsr.sustain_pct[ch], 2u, 100u);
}

static uint8_t tr2adsr_effective_release_pct(uint8_t ch) {
  return tr2adsr_add_cv_pct(g_tr2adsr.release_pct[ch], 3u, 100u);
}

static void tr2adsr_compute_durations_ms(uint8_t ch, uint32_t* attack_ms, uint32_t* decay_ms,
                                         uint32_t* release_ms) {
  uint32_t total_ms = (uint32_t)g_tr2adsr.latched_total_ms[ch];
  uint32_t attack;
  uint32_t decay;
  uint32_t release;
  uint32_t attack_pct = (uint32_t)g_tr2adsr.latched_attack_pct[ch];
  uint32_t decay_pct = (uint32_t)g_tr2adsr.latched_decay_pct[ch];
  uint32_t release_pct = (uint32_t)g_tr2adsr.latched_release_pct[ch];
  if (total_ms < 10u) total_ms = 10u;

  if (g_tr2adsr.type[ch] == TR2ADSR_ENV_AR || g_tr2adsr.type[ch] == TR2ADSR_ENV_ASR) {
    uint32_t sum = attack_pct + release_pct;
    if (sum < 1u) sum = 1u;
    attack = (total_ms * attack_pct) / sum;
    release = total_ms - attack;
    if (attack < 1u) attack = 1u;
    if (release < 1u) release = 1u;
    decay = 1u;
  } else {
    uint32_t sum = attack_pct + decay_pct + release_pct;
    if (sum < 1u) sum = 1u;
    attack = (total_ms * attack_pct) / sum;
    decay = (total_ms * decay_pct) / sum;
    if (attack < 1u) attack = 1u;
    if (decay < 1u) decay = 1u;
    release = total_ms - attack - decay;
    if (release < 1u) release = 1u;
  }

  *attack_ms = attack;
  *decay_ms = decay;
  *release_ms = release;
}

static void tr2adsr_set_state(uint8_t ch, tr2adsr_env_state_t state, uint64_t now_ms) {
  g_tr2adsr.env_state[ch] = state;
  g_tr2adsr.state_start_ms[ch] = now_ms;
}

static void tr2adsr_on_trigger(uint8_t ch, uint64_t now_ms) {
  uint32_t total_ms = (uint32_t)g_tr2adsr.total_time_ds[ch] * 100u;
  uint8_t sustain_pct;
  if (total_ms < 100u) total_ms = 100u;
  if (total_ms > 10000u) total_ms = 10000u;
  g_tr2adsr.latched_total_ms[ch] = (uint16_t)total_ms;
  g_tr2adsr.latched_attack_pct[ch] = tr2adsr_effective_attack_pct(ch);
  g_tr2adsr.latched_decay_pct[ch] = tr2adsr_effective_decay_pct(ch);
  g_tr2adsr.latched_release_pct[ch] = tr2adsr_effective_release_pct(ch);
  sustain_pct = tr2adsr_effective_sustain_pct(ch);
  g_tr2adsr.gate_hold_until_ms[ch] = now_ms + total_ms;
  g_tr2adsr.latched_sustain[ch] =
      (g_tr2adsr.type[ch] == TR2ADSR_ENV_ASR) ? 1.0f : ((float)sustain_pct / 100.0f);
  tr2adsr_set_state(ch, TR2ADSR_STATE_ATTACK, now_ms);
}

static void tr2adsr_advance_env(uint8_t ch, uint64_t now_ms, bool gate_high) {
  uint32_t attack_ms;
  uint32_t decay_ms;
  uint32_t release_ms;
  float t;
  tr2adsr_compute_durations_ms(ch, &attack_ms, &decay_ms, &release_ms);

  if (g_tr2adsr.type[ch] != TR2ADSR_ENV_AR && !gate_high &&
      g_tr2adsr.env_state[ch] != TR2ADSR_STATE_IDLE && g_tr2adsr.env_state[ch] != TR2ADSR_STATE_RELEASE) {
    g_tr2adsr.release_start[ch] = g_tr2adsr.level[ch];
    tr2adsr_set_state(ch, TR2ADSR_STATE_RELEASE, now_ms);
  }

  switch (g_tr2adsr.env_state[ch]) {
    case TR2ADSR_STATE_IDLE:
      g_tr2adsr.level[ch] = 0.0f;
      break;

    case TR2ADSR_STATE_ATTACK:
      t = (float)(now_ms - g_tr2adsr.state_start_ms[ch]) / (float)attack_ms;
      if (t >= 1.0f) {
        g_tr2adsr.level[ch] = 1.0f;
        if (g_tr2adsr.type[ch] == TR2ADSR_ENV_AR) {
          g_tr2adsr.release_start[ch] = 1.0f;
          tr2adsr_set_state(ch, TR2ADSR_STATE_RELEASE, now_ms);
        } else if (g_tr2adsr.type[ch] == TR2ADSR_ENV_ASR) {
          tr2adsr_set_state(ch, TR2ADSR_STATE_SUSTAIN, now_ms);
        } else {
          tr2adsr_set_state(ch, TR2ADSR_STATE_DECAY, now_ms);
        }
      } else {
        g_tr2adsr.level[ch] = t;
      }
      break;

    case TR2ADSR_STATE_DECAY:
      t = (float)(now_ms - g_tr2adsr.state_start_ms[ch]) / (float)decay_ms;
      if (t >= 1.0f) {
        g_tr2adsr.level[ch] = g_tr2adsr.latched_sustain[ch];
        tr2adsr_set_state(ch, TR2ADSR_STATE_SUSTAIN, now_ms);
      } else {
        g_tr2adsr.level[ch] = 1.0f + (g_tr2adsr.latched_sustain[ch] - 1.0f) * t;
      }
      break;

    case TR2ADSR_STATE_SUSTAIN:
      g_tr2adsr.level[ch] = g_tr2adsr.latched_sustain[ch];
      break;

    case TR2ADSR_STATE_RELEASE:
      t = (float)(now_ms - g_tr2adsr.state_start_ms[ch]) / (float)release_ms;
      if (t >= 1.0f) {
        g_tr2adsr.level[ch] = 0.0f;
        tr2adsr_set_state(ch, TR2ADSR_STATE_IDLE, now_ms);
      } else {
        g_tr2adsr.level[ch] = g_tr2adsr.release_start[ch] * (1.0f - t);
      }
      break;
  }

  if (g_tr2adsr.level[ch] < 0.0f) g_tr2adsr.level[ch] = 0.0f;
  if (g_tr2adsr.level[ch] > 1.0f) g_tr2adsr.level[ch] = 1.0f;
}

static void tr2adsr_apply_outputs(void) {
  int32_t out_mv[4];
  float max_mv = g_tr2adsr.level_10v ? 10000.0f : 5000.0f;
  for (uint8_t i = 0u; i < 4u; ++i) {
    out_mv[i] = (int32_t)(g_tr2adsr.level[i] * max_mv);
  }
  g_tr2adsr.outputs_ok = app_write_outputs_mv(out_mv);
}

static void tr2adsr_reset_outputs_and_state(void) {
  uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  tr2adsr_refresh_cv_inputs();
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_tr2adsr.src_active[i] = read_trigger_active_src(i);
    g_tr2adsr.env_state[i] = TR2ADSR_STATE_IDLE;
    g_tr2adsr.level[i] = 0.0f;
    g_tr2adsr.release_start[i] = 0.0f;
    g_tr2adsr.latched_sustain[i] = (float)g_tr2adsr.sustain_pct[i] / 100.0f;
    g_tr2adsr.latched_attack_pct[i] = g_tr2adsr.attack_pct[i];
    g_tr2adsr.latched_decay_pct[i] = g_tr2adsr.decay_pct[i];
    g_tr2adsr.latched_release_pct[i] = g_tr2adsr.release_pct[i];
    g_tr2adsr.gate_hold_until_ms[i] = now_ms;
    g_tr2adsr.state_start_ms[i] = now_ms;
    g_tr2adsr.latched_total_ms[i] = (uint16_t)((uint32_t)g_tr2adsr.total_time_ds[i] * 100u);
  }
  tr2adsr_apply_outputs();
}

static void tr2adsr_enter(void) {
  tr2adsr_reset_outputs_and_state();
}

static void burstgen_apply_outputs(void) {
  int32_t out_mv[4];
  int32_t high_mv = g_burstgen.level_10v ? 10000 : 5000;
  for (uint8_t i = 0u; i < 4u; ++i) {
    out_mv[i] = g_burstgen.gate_out[i] ? high_mv : 0;
  }
  g_burstgen.outputs_ok = app_write_outputs_mv(out_mv);
}

static void burstgen_reset_outputs_and_state(void) {
  uint32_t now_us = (uint32_t)to_us_since_boot(get_absolute_time());
  uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  g_burstgen.rng_state = now_us ^ 0x53A91C27u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_burstgen.src_active[i] = read_trigger_active_src(i);
    g_burstgen.running[i] = false;
    g_burstgen.gate_out[i] = false;
    g_burstgen.pattern_mask[i] = 0u;
    g_burstgen.current_step[i] = 0u;
    g_burstgen.next_step_at_ms[i] = now_ms;
    g_burstgen.gate_off_at_ms[i] = 0u;
  }
  burstgen_apply_outputs();
}

static void burstgen_start_channel(uint8_t ch, uint64_t now_ms) {
  uint8_t steps = burstgen_steps_for_mode(g_burstgen.signature_mode);
  uint8_t max_hits = burstgen_max_hits_for_mode(g_burstgen.signature_mode);
  g_burstgen.pattern_mask[ch] = burstgen_pick_pattern_mask(steps, max_hits);
  g_burstgen.current_step[ch] = 0u;
  g_burstgen.next_step_at_ms[ch] = now_ms;
  g_burstgen.gate_off_at_ms[ch] = 0u;
  g_burstgen.running[ch] = true;
}

static void burstgen_update_channel(uint8_t ch, uint64_t now_ms) {
  uint8_t steps = burstgen_steps_for_mode(g_burstgen.signature_mode);
  uint32_t quarter_ms;
  uint32_t base_step_ms;

  if (g_burstgen.gate_out[ch] && now_ms >= g_burstgen.gate_off_at_ms[ch]) {
    g_burstgen.gate_out[ch] = false;
  }

  if (!g_burstgen.running[ch] || now_ms < g_burstgen.next_step_at_ms[ch]) return;

  quarter_ms = (uint32_t)(60000u / (uint32_t)clamp_i((int)g_burstgen.bpm, BURSTGEN_BPM_MIN, BURSTGEN_BPM_MAX));
  base_step_ms = (quarter_ms * 4u) / steps;
  if (base_step_ms == 0u) base_step_ms = 1u;

  if (burstgen_pattern_hit(g_burstgen.pattern_mask[ch], steps, g_burstgen.current_step[ch]) &&
      prob_pass(g_burstgen.probability, &g_burstgen.rng_state)) {
    g_burstgen.gate_out[ch] = true;
    g_burstgen.gate_off_at_ms[ch] = now_ms + BURSTGEN_PULSE_MS;
  } else {
    g_burstgen.gate_out[ch] = false;
    g_burstgen.gate_off_at_ms[ch] = 0u;
  }

  {
    uint32_t interval_ms = burstgen_step_interval_ms(g_burstgen.current_step[ch], base_step_ms);
    g_burstgen.current_step[ch] = (uint8_t)(g_burstgen.current_step[ch] + 1u);
    if (g_burstgen.current_step[ch] >= steps) {
      g_burstgen.running[ch] = false;
      g_burstgen.current_step[ch] = 0u;
    } else {
      g_burstgen.next_step_at_ms[ch] += interval_ms;
    }
  }
}

static void burstgen_enter(void) {
  burstgen_reset_outputs_and_state();
}

static uint32_t cvgen_period_ms(const cvgen_channel_state_t* channel) {
  uint16_t rate_dhz = (uint16_t)clamp_i((int)channel->rate_dhz, CVGEN_RATE_MIN_DHZ, CVGEN_RATE_MAX_DHZ);
  uint32_t period = 10000u / rate_dhz;
  return period < 20u ? 20u : period;
}

static void cvgen_apply_outputs(void) {
  int32_t out_mv[4];
  for (uint8_t i = 0u; i < 4u; ++i) {
    out_mv[i] = cvgen_map_norm_to_mv(&g_cvgen.ch[i], g_cvgen.ch[i].output_norm);
  }
  g_cvgen.outputs_ok = app_write_outputs_mv(out_mv);
}

static void cvgen_reset_outputs_and_state(void) {
  uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  g_cvgen.rng_state ^= (uint32_t)to_us_since_boot(get_absolute_time());
  for (uint8_t i = 0u; i < 4u; ++i) {
    cvgen_init_channel(&g_cvgen.ch[i], now_ms, &g_cvgen.rng_state);
  }
  cvgen_apply_outputs();
}

static void cvgen_enter(void) {
  cvgen_reset_outputs_and_state();
}

static void cvgen_step_update(cvgen_channel_state_t* channel, uint64_t now_ms, bool rising_edge) {
  uint32_t period_ms = cvgen_period_ms(channel);
  float alpha = 1.0f - 0.95f * ((float)channel->param1 / 100.0f);

  if (channel->clock_mode == CVGEN_CLOCK_INT) {
    while ((now_ms - channel->segment_start_ms) >= period_ms) {
      channel->segment_start_ms += period_ms;
      cvgen_step_pick_target(channel, &g_cvgen.rng_state);
    }
  } else if (rising_edge) {
    cvgen_step_pick_target(channel, &g_cvgen.rng_state);
  }

  channel->output_norm += alpha * (channel->step_target_norm - channel->output_norm);
  channel->output_norm = clampf(channel->output_norm, 0.0f, 1.0f);
}

static void cvgen_bezier_update(cvgen_channel_state_t* channel, uint64_t now_ms, bool rising_edge) {
  uint32_t period_ms = cvgen_period_ms(channel);
  float t;

  if (channel->clock_mode == CVGEN_CLOCK_INT) {
    channel->segment_duration_ms = period_ms;
    while ((now_ms - channel->segment_start_ms) >= channel->segment_duration_ms) {
      channel->segment_start_ms += channel->segment_duration_ms;
      cvgen_bezier_start_segment(channel, channel->segment_start_ms, &g_cvgen.rng_state);
    }
  } else if (rising_edge) {
    if (channel->last_edge_ms != 0u && now_ms > channel->last_edge_ms) {
      channel->segment_duration_ms =
          (uint32_t)clamp_i((int)(now_ms - channel->last_edge_ms), 20, 10000);
    } else if (channel->segment_duration_ms == 0u) {
      channel->segment_duration_ms = period_ms;
    }
    channel->last_edge_ms = now_ms;
    cvgen_bezier_start_segment(channel, now_ms, &g_cvgen.rng_state);
  } else if (channel->segment_duration_ms == 0u) {
    channel->segment_duration_ms = period_ms;
  }

  t = (float)(now_ms - channel->segment_start_ms) / (float)(channel->segment_duration_ms == 0u ? 1u : channel->segment_duration_ms);
  channel->output_norm = clampf(cvgen_bezier_value(channel->bez_y0, channel->bez_c1, channel->bez_c2, channel->bez_y1,
                                                   clampf(t, 0.0f, 1.0f)),
                                0.0f, 1.0f);
}

static void cvgen_ocean_update(cvgen_channel_state_t* channel, float dt_s, uint64_t now_ms, bool rising_edge) {
  float freq_hz = (float)channel->rate_dhz / 10.0f;
  float swell = (float)channel->param1 / 100.0f;
  float agitation = (float)channel->param2 / 100.0f;
  float speed_mod;
  float spread_dyn;
  float radius;
  float wavelength;
  float base;
  float overtone;
  float sub;
  float y;

  if (channel->clock_mode == CVGEN_CLOCK_EXT && rising_edge) {
    if (channel->last_edge_ms != 0u && now_ms > channel->last_edge_ms) {
      channel->segment_duration_ms =
          (uint32_t)clamp_i((int)(now_ms - channel->last_edge_ms), 20, 10000);
    }
    channel->last_edge_ms = now_ms;
    channel->ocean_t = 0.0f;
  }
  if (channel->clock_mode == CVGEN_CLOCK_EXT && channel->segment_duration_ms > 0u) {
    freq_hz = 1000.0f / (float)channel->segment_duration_ms;
  }

  channel->ocean_rate_wobble =
      clampf(channel->ocean_rate_wobble + cvgen_rand_signed(&g_cvgen.rng_state) * 0.03f, -0.5f, 0.5f);
  speed_mod = freq_hz * (1.0f + 0.18f * channel->ocean_rate_wobble);
  channel->ocean_t = fmodf(channel->ocean_t + speed_mod * dt_s * CVGEN_TWO_PI, CVGEN_TWO_PI);
  channel->ocean_phase_a =
      fmodf(channel->ocean_phase_a + (0.07f + 0.08f * swell) * dt_s * (1.0f + 0.40f * channel->ocean_rate_wobble),
            CVGEN_TWO_PI);
  channel->ocean_phase_b =
      fmodf(channel->ocean_phase_b + (0.11f + 0.12f * agitation) * dt_s * (1.0f - 0.35f * channel->ocean_rate_wobble),
            CVGEN_TWO_PI);
  if ((g_cvgen.rng_state & 0x1Fu) == 0u) {
    channel->ocean_mix_target = 0.12f + 0.38f * cvgen_rand_unit(&g_cvgen.rng_state);
  }
  channel->ocean_mix += 0.02f * (channel->ocean_mix_target - channel->ocean_mix);

  spread_dyn = clampf(0.5f + 0.25f * sinf(0.37f * channel->ocean_phase_a + 0.23f * channel->ocean_phase_b), 0.0f, 1.0f);
  radius = 0.01f + agitation * 0.99f;
  wavelength = 1.0f + swell * 19.0f;
  base = radius * cosf(channel->ocean_t - CVGEN_TWO_PI * (10.0f * spread_dyn) / wavelength);
  overtone = channel->ocean_mix * radius * 0.7f * sinf((channel->ocean_t * 2.11f) + channel->ocean_phase_b);
  sub = 0.12f * sinf((channel->ocean_t * 0.73f) + 0.5f * channel->ocean_phase_a);
  y = clampf(base + overtone + sub, -1.0f, 1.0f);
  channel->output_norm = 0.5f + 0.5f * y;
}

static void cvgen_walk_update(cvgen_channel_state_t* channel, float dt_s, uint64_t now_ms, bool rising_edge,
                              bool gate_high) {
  uint32_t period_ms = cvgen_period_ms(channel);
  uint8_t mode = cvgen_walk_mode_from_param(channel->param2);
  float v;

  (void)dt_s;
  cvgen_walk_set_params(channel);

  if (channel->clock_mode == CVGEN_CLOCK_INT) {
    if (mode == CVGEN_WALK_MODE_WALK) {
      while ((now_ms - channel->segment_start_ms) >= period_ms) {
        channel->segment_start_ms += period_ms;
      }
      v = cvgen_walk_next(channel, &g_cvgen.rng_state);
      channel->output_norm = 0.5f + 0.5f * v;
    } else if (mode == CVGEN_WALK_MODE_SNH) {
      while ((now_ms - channel->segment_start_ms) >= period_ms) {
        channel->segment_start_ms += period_ms;
        v = cvgen_walk_next(channel, &g_cvgen.rng_state);
        channel->hold_norm = 0.5f + 0.5f * v;
      }
      channel->output_norm = channel->hold_norm;
    } else {
      while ((now_ms - channel->segment_start_ms) >= period_ms) {
        channel->segment_start_ms += period_ms;
      }
      if ((now_ms - channel->segment_start_ms) < (period_ms / 2u)) {
        v = cvgen_walk_next(channel, &g_cvgen.rng_state);
        channel->hold_norm = 0.5f + 0.5f * v;
      }
      channel->output_norm = channel->hold_norm;
    }
  } else {
    v = cvgen_walk_next(channel, &g_cvgen.rng_state);
    if (mode == CVGEN_WALK_MODE_WALK) {
      if (rising_edge) {
        channel->segment_start_ms = now_ms;
      }
      channel->output_norm = 0.5f + 0.5f * v;
    } else if (mode == CVGEN_WALK_MODE_SNH) {
      if (rising_edge) {
        channel->hold_norm = 0.5f + 0.5f * v;
      }
      channel->output_norm = channel->hold_norm;
    } else {
      if (gate_high) {
        channel->hold_norm = 0.5f + 0.5f * v;
      }
      channel->output_norm = channel->hold_norm;
    }
  }
}

static void cvgen_update_channel(cvgen_channel_state_t* channel, uint64_t now_ms) {
  bool src_active = read_trigger_active_src(channel->source);
  bool rising_edge = src_active && !channel->prev_src_active;
  float dt_s = (float)(now_ms - channel->last_update_ms) / 1000.0f;
  if (dt_s < 0.0f) dt_s = 0.0f;

  if (channel->algo == CVGEN_ALGO_STEP) {
    cvgen_step_update(channel, now_ms, rising_edge);
  } else if (channel->algo == CVGEN_ALGO_BEZIER) {
    cvgen_bezier_update(channel, now_ms, rising_edge);
  } else if (channel->algo == CVGEN_ALGO_OCEAN) {
    cvgen_ocean_update(channel, dt_s, now_ms, rising_edge);
  } else {
    cvgen_walk_update(channel, dt_s, now_ms, rising_edge, src_active);
  }

  channel->prev_src_active = src_active;
  channel->last_update_ms = now_ms;
}

static void load_runtime_from_app_settings(void) {
  g_grids.clock = (g_app_settings_data.grids.clock_mode == 0u) ? GRIDS_CLOCK_INT : GRIDS_CLOCK_EXT;
  g_grids.bpm = clamp_i((int)g_app_settings_data.grids.bpm, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
  g_grids.map_x = g_app_settings_data.grids.map_x;
  g_grids.map_y = g_app_settings_data.grids.map_y;
  g_grids.chaos = g_app_settings_data.grids.chaos;
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_grids.prob[i] = clamp_u8i_100((int)g_app_settings_data.grids.prob[i]);
  }

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
    g_trigseq.prob[i] = clamp_u8i_100((int)g_app_settings_data.trigseq.prob[i]);
  }
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_trigseq.engine.pattern[i] = g_app_settings_data.trigseq.pattern[i];
  }
  g_trigseq.cursor_step = (uint8_t)(g_app_settings_data.trigseq.edit_step & 0x3Fu);
  g_trigseq.run = true;
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
    g_euclid.prob[i] = clamp_u8i_100((int)g_app_settings_data.euclid.prob[i]);
  }
  g_euclid.grid_cache_valid = false;

  for (uint8_t i = 0u; i < 4u; ++i) {
    g_tr2gate.source[i] = g_app_settings_data.tr2gate.src[i] & 0x03u;
    g_tr2gate.gate_time_cs[i] =
        (uint16_t)clamp_i((int)g_app_settings_data.tr2gate.gate_time_cs[i], TR2GATE_GATE_MIN_CS,
                          TR2GATE_GATE_MAX_CS);
    g_tr2gate.prob[i] = clamp_u8i_100((int)g_app_settings_data.tr2gate.prob[i]);
  }
  g_tr2gate.level_10v = g_app_settings_data.tr2gate.level_10v != 0u;

  for (uint8_t i = 0u; i < 4u; ++i) {
    g_tr2adsr.source[i] = g_app_settings_data.tr2adsr.src[i] & 0x03u;
    g_tr2adsr.type[i] =
        (tr2adsr_env_type_t)clamp_i((int)g_app_settings_data.tr2adsr.type[i], 0, 2);
    g_tr2adsr.total_time_ds[i] =
        (uint16_t)clamp_i((int)g_app_settings_data.tr2adsr.total_time_ds[i], TR2ADSR_TIME_MIN_DS,
                          TR2ADSR_TIME_MAX_DS);
    g_tr2adsr.prob[i] = clamp_u8i_100((int)g_app_settings_data.tr2adsr.prob[i]);
    g_tr2adsr.attack_pct[i] = (uint8_t)clamp_i((int)g_app_settings_data.tr2adsr.a_pct[i], 1, 99);
    g_tr2adsr.decay_pct[i] = (uint8_t)clamp_i((int)g_app_settings_data.tr2adsr.d_pct[i], 1, 99);
    g_tr2adsr.sustain_pct[i] = clamp_u8i_100((int)g_app_settings_data.tr2adsr.s_pct[i]);
    g_tr2adsr.release_pct[i] = (uint8_t)clamp_i((int)g_app_settings_data.tr2adsr.r_pct[i], 1, 99);
  }
  g_tr2adsr.level_10v = g_app_settings_data.tr2adsr.level_10v != 0u;

  g_burstgen.selected_channel = g_app_settings_data.burstgen.selected_channel & 0x03u;
  g_burstgen.signature_mode = (uint8_t)clamp_i((int)g_app_settings_data.burstgen.signature_mode, 0, 2);
  g_burstgen.bpm =
      (uint16_t)clamp_i((int)g_app_settings_data.burstgen.bpm, BURSTGEN_BPM_MIN, BURSTGEN_BPM_MAX);
  g_burstgen.swing_pct =
      (uint8_t)clamp_i((int)g_app_settings_data.burstgen.swing_pct, 0, (int)BURSTGEN_SWING_MAX);
  g_burstgen.probability = clamp_u8i_100((int)g_app_settings_data.burstgen.probability);
  g_burstgen.level_10v = g_app_settings_data.burstgen.level_10v != 0u;

  for (uint8_t i = 0u; i < 4u; ++i) {
    g_cvgen.ch[i].algo = (cvgen_algo_t)clamp_i((int)g_app_settings_data.cvgen.algo[i], 0, 3);
    g_cvgen.ch[i].clock_mode =
        (g_app_settings_data.cvgen.clock_mode[i] == 0u) ? CVGEN_CLOCK_INT : CVGEN_CLOCK_EXT;
    g_cvgen.ch[i].source = g_app_settings_data.cvgen.src[i] & 0x03u;
    g_cvgen.ch[i].rate_dhz =
        (uint16_t)clamp_i((int)g_app_settings_data.cvgen.rate_dhz[i], CVGEN_RATE_MIN_DHZ, CVGEN_RATE_MAX_DHZ);
    g_cvgen.ch[i].min_mv =
        (int16_t)clamp_i((int)g_app_settings_data.cvgen.min_mv[i], CVGEN_VOLT_MIN_MV, CVGEN_VOLT_MAX_MV);
    g_cvgen.ch[i].max_mv =
        (int16_t)clamp_i((int)g_app_settings_data.cvgen.max_mv[i], CVGEN_VOLT_MIN_MV, CVGEN_VOLT_MAX_MV);
    if (g_cvgen.ch[i].max_mv <= g_cvgen.ch[i].min_mv) {
      g_cvgen.ch[i].max_mv =
          (int16_t)clamp_i(g_cvgen.ch[i].min_mv + CVGEN_MIN_SPAN_MV, CVGEN_VOLT_MIN_MV, CVGEN_VOLT_MAX_MV);
    }
    g_cvgen.ch[i].param1 = clamp_u8i_100((int)g_app_settings_data.cvgen.param1[i]);
    g_cvgen.ch[i].param2 = clamp_u8i_100((int)g_app_settings_data.cvgen.param2[i]);
  }
}

static void capture_runtime_to_app_settings(void) {
  g_app_settings_data.grids.clock_mode = (g_grids.clock == GRIDS_CLOCK_INT) ? 0u : 1u;
  g_app_settings_data.grids.bpm = (uint16_t)clamp_i(g_grids.bpm, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
  g_app_settings_data.grids.map_x = g_grids.map_x;
  g_app_settings_data.grids.map_y = g_grids.map_y;
  g_app_settings_data.grids.chaos = g_grids.chaos;
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.grids.prob[i] = clamp_u8i_100((int)g_grids.prob[i]);
  }

  g_app_settings_data.trigseq.length = trigseq_len_for_mode(g_trigseq.len_mode);
  g_app_settings_data.trigseq.edit_channel = (uint8_t)g_trigseq.len_mode;
  g_app_settings_data.trigseq.edit_step = g_trigseq.cursor_step;
  g_app_settings_data.trigseq.clock_mode = (g_trigseq.clock == TRIGSEQ_CLOCK_INT) ? 0u : 1u;
  g_app_settings_data.trigseq.bpm = (uint16_t)clamp_i(g_trigseq.bpm, TRIGSEQ_BPM_MIN, TRIGSEQ_BPM_MAX);
  g_app_settings_data.trigseq.run = 1u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.trigseq.prob[i] = clamp_u8i_100((int)g_trigseq.prob[i]);
  }
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.trigseq.pattern[i] = g_trigseq.engine.pattern[i];
  }

  g_app_settings_data.euclid.clock_mode = (g_euclid.clock == EUCLID_CLOCK_INT) ? 0u : 1u;
  g_app_settings_data.euclid.bpm = (uint16_t)clamp_i(g_euclid.bpm, EUCLID_BPM_MIN, EUCLID_BPM_MAX);
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.euclid.steps[i] = g_euclid.steps[i];
    g_app_settings_data.euclid.hits[i] = g_euclid.hits[i];
    g_app_settings_data.euclid.prob[i] = clamp_u8i_100((int)g_euclid.prob[i]);
  }

  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.tr2gate.src[i] = g_tr2gate.source[i] & 0x03u;
    g_app_settings_data.tr2gate.gate_time_cs[i] =
        (uint16_t)clamp_i((int)g_tr2gate.gate_time_cs[i], TR2GATE_GATE_MIN_CS, TR2GATE_GATE_MAX_CS);
    g_app_settings_data.tr2gate.prob[i] = clamp_u8i_100((int)g_tr2gate.prob[i]);
  }
  g_app_settings_data.tr2gate.level_10v = g_tr2gate.level_10v ? 1u : 0u;

  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.tr2adsr.src[i] = g_tr2adsr.source[i] & 0x03u;
    g_app_settings_data.tr2adsr.type[i] = (uint8_t)g_tr2adsr.type[i];
    g_app_settings_data.tr2adsr.total_time_ds[i] =
        (uint16_t)clamp_i((int)g_tr2adsr.total_time_ds[i], TR2ADSR_TIME_MIN_DS, TR2ADSR_TIME_MAX_DS);
    g_app_settings_data.tr2adsr.prob[i] = clamp_u8i_100((int)g_tr2adsr.prob[i]);
    g_app_settings_data.tr2adsr.a_pct[i] = (uint8_t)clamp_i((int)g_tr2adsr.attack_pct[i], 1, 99);
    g_app_settings_data.tr2adsr.d_pct[i] = (uint8_t)clamp_i((int)g_tr2adsr.decay_pct[i], 1, 99);
    g_app_settings_data.tr2adsr.s_pct[i] = clamp_u8i_100((int)g_tr2adsr.sustain_pct[i]);
    g_app_settings_data.tr2adsr.r_pct[i] = (uint8_t)clamp_i((int)g_tr2adsr.release_pct[i], 1, 99);
  }
  g_app_settings_data.tr2adsr.level_10v = g_tr2adsr.level_10v ? 1u : 0u;

  g_app_settings_data.burstgen.selected_channel = g_burstgen.selected_channel & 0x03u;
  g_app_settings_data.burstgen.signature_mode = (uint8_t)clamp_i((int)g_burstgen.signature_mode, 0, 2);
  g_app_settings_data.burstgen.bpm =
      (uint16_t)clamp_i((int)g_burstgen.bpm, BURSTGEN_BPM_MIN, BURSTGEN_BPM_MAX);
  g_app_settings_data.burstgen.swing_pct =
      (uint8_t)clamp_i((int)g_burstgen.swing_pct, 0, (int)BURSTGEN_SWING_MAX);
  g_app_settings_data.burstgen.probability = clamp_u8i_100((int)g_burstgen.probability);
  g_app_settings_data.burstgen.level_10v = g_burstgen.level_10v ? 1u : 0u;

  for (uint8_t i = 0u; i < 4u; ++i) {
    g_app_settings_data.cvgen.algo[i] = (uint8_t)g_cvgen.ch[i].algo;
    g_app_settings_data.cvgen.clock_mode[i] = (g_cvgen.ch[i].clock_mode == CVGEN_CLOCK_INT) ? 0u : 1u;
    g_app_settings_data.cvgen.src[i] = g_cvgen.ch[i].source & 0x03u;
    g_app_settings_data.cvgen.rate_dhz[i] =
        (uint16_t)clamp_i((int)g_cvgen.ch[i].rate_dhz, CVGEN_RATE_MIN_DHZ, CVGEN_RATE_MAX_DHZ);
    g_app_settings_data.cvgen.min_mv[i] =
        (int16_t)clamp_i((int)g_cvgen.ch[i].min_mv, CVGEN_VOLT_MIN_MV, CVGEN_VOLT_MAX_MV);
    g_app_settings_data.cvgen.max_mv[i] =
        (int16_t)clamp_i((int)g_cvgen.ch[i].max_mv, CVGEN_VOLT_MIN_MV, CVGEN_VOLT_MAX_MV);
    g_app_settings_data.cvgen.param1[i] = clamp_u8i_100((int)g_cvgen.ch[i].param1);
    g_app_settings_data.cvgen.param2[i] = clamp_u8i_100((int)g_cvgen.ch[i].param2);
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

static void save_tr2gate_settings(void) {
  capture_runtime_to_app_settings();
  if (app_settings_save(&g_app_settings_data)) {
    set_tr2gate_status("SAVED");
  } else {
    set_tr2gate_status("SAVE ERR");
  }
}

static void save_tr2adsr_settings(void) {
  capture_runtime_to_app_settings();
  if (app_settings_save(&g_app_settings_data)) {
    set_tr2adsr_status("SAVED");
  } else {
    set_tr2adsr_status("SAVE ERR");
  }
}

static void save_burstgen_settings(void) {
  capture_runtime_to_app_settings();
  if (app_settings_save(&g_app_settings_data)) {
    set_burstgen_status("SAVED");
  } else {
    set_burstgen_status("SAVE ERR");
  }
}

static void save_cvgen_settings(void) {
  capture_runtime_to_app_settings();
  if (app_settings_save(&g_app_settings_data)) {
    set_cvgen_status("SAVED");
  } else {
    set_cvgen_status("SAVE ERR");
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

static uint8_t prob_rng8(uint32_t* state) {
  uint32_t x = *state;
  x ^= x << 13u;
  x ^= x >> 17u;
  x ^= x << 5u;
  *state = x;
  return (uint8_t)(x >> 24u);
}

static bool prob_pass(uint8_t prob_percent, uint32_t* rng_state) {
  if (prob_percent >= 100u) return true;
  if (prob_percent == 0u) return false;
  return (prob_rng8(rng_state) % 100u) < prob_percent;
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
    if (trig[i] && prob_pass(g_grids.prob[i], &g_grids.prob_rng_state)) {
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
  uint32_t now_us = (uint32_t)to_us_since_boot(get_absolute_time());
  g_grids.preview_cache_valid = false;
  grids_engine_init(&g_grids.engine, now_us);
  g_grids.prob_rng_state = now_us ^ 0xA5A55A5Au;
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
  g_preset_ui.screen = APP_SCREEN_MAIN;
  g_preset_ui.menu_open = false;
  g_preset_ui.popup_dirty = false;
  hal_io_oled_clear();
  set_encoder_reference_now();

  if (mode == APP_MENU) {
    grids_reset_outputs_and_state();
    trigseq_reset_outputs_and_state();
    euclid_reset_outputs_and_state();
    tr2gate_reset_outputs_and_state();
    tr2adsr_reset_outputs_and_state();
    burstgen_reset_outputs_and_state();
    /* CV GEN has no pulse state, but we still zero its outputs on menu exit. */
    for (uint8_t i = 0u; i < 4u; ++i) {
      g_cvgen.ch[i].output_norm = 0.0f;
    }
    g_cvgen.outputs_ok = app_write_outputs_mv((int32_t[4]){0, 0, 0, 0});
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
  } else if (mode == APP_TR2GATE) {
    tr2gate_enter();
  } else if (mode == APP_TR2ADSR) {
    tr2adsr_enter();
  } else if (mode == APP_BURSTGEN) {
    burstgen_enter();
  } else if (mode == APP_CVGEN) {
    cvgen_enter();
  }
}

static bool app_supports_presets(app_mode_t mode) {
  return mode == APP_GRIDS || mode == APP_TRIGSEQ || mode == APP_EUCLID || mode == APP_TR2GATE ||
         mode == APP_TR2ADSR || mode == APP_BURSTGEN || mode == APP_CVGEN;
}

static const char* app_display_name(app_mode_t mode) {
  if (mode == APP_GRIDS) return "GRIDS";
  if (mode == APP_TRIGSEQ) return "TRIG SEQ";
  if (mode == APP_EUCLID) return "4XEUCLID";
  if (mode == APP_TR2GATE) return "TR2GATE";
  if (mode == APP_TR2ADSR) return "TR2ADSR";
  if (mode == APP_BURSTGEN) return "BURST GEN";
  if (mode == APP_CVGEN) return "CV GEN";
  return "APP";
}

static void active_app_preset_pick_initial_slot(void);

static void set_active_app_status(const char* s) {
  if (g_app_mode == APP_GRIDS) {
    set_grids_status(s);
  } else if (g_app_mode == APP_TRIGSEQ) {
    set_trigseq_status(s);
  } else if (g_app_mode == APP_EUCLID) {
    set_euclid_status(s);
  } else if (g_app_mode == APP_TR2GATE) {
    set_tr2gate_status(s);
  } else if (g_app_mode == APP_TR2ADSR) {
    set_tr2adsr_status(s);
  } else if (g_app_mode == APP_BURSTGEN) {
    set_burstgen_status(s);
  } else if (g_app_mode == APP_CVGEN) {
    set_cvgen_status(s);
  }
}

static const char* active_app_status_text(uint64_t now_ms) {
  if (g_app_mode == APP_GRIDS && g_grids.status_until_ms > now_ms && g_grids.status[0] != '\0') {
    return g_grids.status;
  }
  if (g_app_mode == APP_TRIGSEQ && g_trigseq.status_until_ms > now_ms && g_trigseq.status[0] != '\0') {
    return g_trigseq.status;
  }
  if (g_app_mode == APP_EUCLID && g_euclid.status_until_ms > now_ms && g_euclid.status[0] != '\0') {
    return g_euclid.status;
  }
  if (g_app_mode == APP_TR2GATE && g_tr2gate.status_until_ms > now_ms && g_tr2gate.status[0] != '\0') {
    return g_tr2gate.status;
  }
  if (g_app_mode == APP_TR2ADSR && g_tr2adsr.status_until_ms > now_ms && g_tr2adsr.status[0] != '\0') {
    return g_tr2adsr.status;
  }
  if (g_app_mode == APP_BURSTGEN && g_burstgen.status_until_ms > now_ms && g_burstgen.status[0] != '\0') {
    return g_burstgen.status;
  }
  if (g_app_mode == APP_CVGEN && g_cvgen.status_until_ms > now_ms && g_cvgen.status[0] != '\0') {
    return g_cvgen.status;
  }
  return "";
}

static void invalidate_active_app_draw_cache(void) {
  if (g_app_mode == APP_GRIDS) {
    g_grids.preview_cache_valid = false;
  } else if (g_app_mode == APP_TRIGSEQ) {
    trigseq_invalidate_grid_cache();
  } else if (g_app_mode == APP_EUCLID) {
    g_euclid.grid_cache_valid = false;
  }
}

static void preset_ui_full_redraw(void) {
  hal_io_oled_clear();
  invalidate_active_app_draw_cache();
}

static void preset_ui_close_menu(void) {
  if (!g_preset_ui.menu_open) return;
  g_preset_ui.menu_open = false;
  g_preset_ui.popup_dirty = false;
  preset_ui_full_redraw();
}

static uint8_t active_app_screen_count(void) {
  if (g_app_mode == APP_TRIGSEQ) return 4u;
  if (g_app_mode == APP_CVGEN) return 6u;
  return 3u;
}

static uint8_t active_app_preset_load_screen(void) {
  return (uint8_t)(active_app_screen_count() - 2u);
}

static uint8_t active_app_preset_save_screen(void) {
  return (uint8_t)(active_app_screen_count() - 1u);
}

static void preset_ui_set_screen(uint8_t screen) {
  uint8_t screen_count = active_app_screen_count();
  if (screen >= screen_count) screen = APP_SCREEN_MAIN;
  if (g_preset_ui.screen == screen) return;
  g_preset_ui.screen = screen;
  if (g_app_mode == APP_TRIGSEQ) {
    g_trigseq.focus = (screen == APP_SCREEN_TRIGSEQ_GRID) ? TRIGSEQ_FOCUS_GRID : TRIGSEQ_FOCUS_MENU;
    if (g_trigseq.focus == TRIGSEQ_FOCUS_GRID) {
      g_trigseq.cursor_step = 0u;
    }
  }
  if (screen == active_app_preset_load_screen() || screen == active_app_preset_save_screen()) {
    active_app_preset_pick_initial_slot();
  }
  preset_ui_full_redraw();
}

static void save_active_app_runtime_settings_quiet(void) {
  if (!app_supports_presets(g_app_mode)) return;
  capture_runtime_to_app_settings();
  (void)app_settings_save(&g_app_settings_data);
}

static bool active_app_preset_slot_used(uint8_t slot) {
  if (g_app_mode == APP_GRIDS) return app_presets_grids_slot_used(slot);
  if (g_app_mode == APP_TRIGSEQ) return app_presets_trigseq_slot_used(slot);
  if (g_app_mode == APP_EUCLID) return app_presets_euclid_slot_used(slot);
  if (g_app_mode == APP_TR2GATE) return app_presets_tr2gate_slot_used(slot);
  if (g_app_mode == APP_TR2ADSR) return app_presets_tr2adsr_slot_used(slot);
  if (g_app_mode == APP_BURSTGEN) return app_presets_burstgen_slot_used(slot);
  if (g_app_mode == APP_CVGEN) return app_presets_cvgen_slot_used(slot);
  return false;
}

static void active_app_preset_pick_initial_slot(void) {
  uint8_t slot = 0u;
  bool ok = false;

  if (g_app_mode == APP_GRIDS) {
    grids_settings_t preset;
    ok = app_presets_grids_load_last(&preset, &slot);
  } else if (g_app_mode == APP_TRIGSEQ) {
    trigseq_settings_t preset;
    ok = app_presets_trigseq_load_last(&preset, &slot);
  } else if (g_app_mode == APP_EUCLID) {
    euclid_settings_t preset;
    ok = app_presets_euclid_load_last(&preset, &slot);
  } else if (g_app_mode == APP_TR2GATE) {
    tr2gate_settings_t preset;
    ok = app_presets_tr2gate_load_last(&preset, &slot);
  } else if (g_app_mode == APP_TR2ADSR) {
    tr2adsr_settings_t preset;
    ok = app_presets_tr2adsr_load_last(&preset, &slot);
  } else if (g_app_mode == APP_BURSTGEN) {
    burstgen_settings_t preset;
    ok = app_presets_burstgen_load_last(&preset, &slot);
  } else if (g_app_mode == APP_CVGEN) {
    cvgen_settings_t preset;
    ok = app_presets_cvgen_load_last(&preset, &slot);
  }

  g_preset_ui.slot_sel = ok ? slot : 0u;
}

static bool active_app_preset_save_slot(uint8_t slot) {
  capture_runtime_to_app_settings();

  if (g_app_mode == APP_GRIDS) {
    return app_presets_grids_save(slot, &g_app_settings_data.grids);
  }
  if (g_app_mode == APP_TRIGSEQ) {
    return app_presets_trigseq_save(slot, &g_app_settings_data.trigseq);
  }
  if (g_app_mode == APP_EUCLID) {
    return app_presets_euclid_save(slot, &g_app_settings_data.euclid);
  }
  if (g_app_mode == APP_TR2GATE) {
    return app_presets_tr2gate_save(slot, &g_app_settings_data.tr2gate);
  }
  if (g_app_mode == APP_TR2ADSR) {
    return app_presets_tr2adsr_save(slot, &g_app_settings_data.tr2adsr);
  }
  if (g_app_mode == APP_BURSTGEN) {
    return app_presets_burstgen_save(slot, &g_app_settings_data.burstgen);
  }
  if (g_app_mode == APP_CVGEN) {
    return app_presets_cvgen_save(slot, &g_app_settings_data.cvgen);
  }
  return false;
}

static bool active_app_preset_load_slot(uint8_t slot, uint64_t now_ms) {
  bool ok = false;

  if (g_app_mode == APP_GRIDS) {
    grids_settings_t preset;
    ok = app_presets_grids_load(slot, &preset);
    if (ok) g_app_settings_data.grids = preset;
  } else if (g_app_mode == APP_TRIGSEQ) {
    trigseq_settings_t preset;
    ok = app_presets_trigseq_load(slot, &preset);
    if (ok) g_app_settings_data.trigseq = preset;
  } else if (g_app_mode == APP_EUCLID) {
    euclid_settings_t preset;
    ok = app_presets_euclid_load(slot, &preset);
    if (ok) g_app_settings_data.euclid = preset;
  } else if (g_app_mode == APP_TR2GATE) {
    tr2gate_settings_t preset;
    ok = app_presets_tr2gate_load(slot, &preset);
    if (ok) g_app_settings_data.tr2gate = preset;
  } else if (g_app_mode == APP_TR2ADSR) {
    tr2adsr_settings_t preset;
    ok = app_presets_tr2adsr_load(slot, &preset);
    if (ok) g_app_settings_data.tr2adsr = preset;
  } else if (g_app_mode == APP_BURSTGEN) {
    burstgen_settings_t preset;
    ok = app_presets_burstgen_load(slot, &preset);
    if (ok) g_app_settings_data.burstgen = preset;
  } else if (g_app_mode == APP_CVGEN) {
    cvgen_settings_t preset;
    ok = app_presets_cvgen_load(slot, &preset);
    if (ok) g_app_settings_data.cvgen = preset;
  }

  if (!ok) return false;

  load_runtime_from_app_settings();
  (void)now_ms;
  app_enter(g_app_mode);
  return true;
}

static bool preset_ui_is_preset_screen(void) {
  return g_preset_ui.screen == active_app_preset_load_screen() ||
         g_preset_ui.screen == active_app_preset_save_screen();
}

static void preset_ui_step_screen(int delta) {
  uint8_t screen_count = active_app_screen_count();
  int screen = (int)g_preset_ui.screen + delta;
  while (screen < 0) screen += (int)screen_count;
  while (screen >= (int)screen_count) screen -= (int)screen_count;
  preset_ui_set_screen((uint8_t)screen);
}

static bool handle_active_app_preset_ui(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1,
                                        bool edge_sw2, uint64_t now_ms) {
  char msg[24];
  int nav = 0;

  if (!app_supports_presets(g_app_mode)) return false;

  if (g_preset_ui.menu_open) {
    if (edge_sw1 || edge_sw2) {
      preset_ui_close_menu();
      return true;
    }
    if (d_l != 0) nav += (d_l > 0) ? 1 : -1;
    if (d_r != 0) nav += (d_r > 0) ? 1 : -1;
    if (nav != 0) {
      int next = (int)g_preset_ui.menu_sel + ((nav > 0) ? 1 : -1);
      while (next < 0) next += 2;
      while (next >= 2) next -= 2;
      g_preset_ui.menu_sel = (uint8_t)next;
      g_preset_ui.popup_dirty = true;
      return true;
    }
    if (edge_enc_r) {
      g_preset_ui.menu_open = false;
      g_preset_ui.popup_dirty = false;
      preset_ui_set_screen((g_preset_ui.menu_sel == 0u) ? active_app_preset_load_screen()
                                                        : active_app_preset_save_screen());
      return true;
    }
    return true;
  }

  if (edge_sw1 && !edge_sw2) {
    preset_ui_step_screen(-1);
    return true;
  }
  if (edge_sw2 && !edge_sw1) {
    preset_ui_step_screen(1);
    return true;
  }

  if (!preset_ui_is_preset_screen()) {
    if (g_app_mode == APP_TRIGSEQ && g_preset_ui.screen == APP_SCREEN_TRIGSEQ_GRID) {
      return false;
    }
    if (!edge_enc_r) return false;
    g_preset_ui.menu_open = true;
    g_preset_ui.popup_dirty = true;
    g_preset_ui.menu_sel = 0u;
    active_app_preset_pick_initial_slot();
    return true;
  }

  nav = (int)d_l + (int)d_r;
  if (nav != 0) {
    g_preset_ui.slot_sel = (uint8_t)clamp_i((int)g_preset_ui.slot_sel + nav, 0, (int)APP_PRESET_SLOTS - 1);
    return true;
  }

  if (edge_enc_r) {
    if (g_preset_ui.screen == active_app_preset_load_screen()) {
      if (active_app_preset_load_slot(g_preset_ui.slot_sel, now_ms)) {
        snprintf(msg, sizeof(msg), "LOAD %u OK", (unsigned)(g_preset_ui.slot_sel + 1u));
        set_active_app_status(msg);
        app_settings_save(&g_app_settings_data);
        g_preset_ui.screen = active_app_preset_load_screen();
        g_preset_ui.menu_open = false;
        preset_ui_full_redraw();
      } else {
        set_active_app_status("LOAD EMPTY");
      }
    } else {
      if (active_app_preset_save_slot(g_preset_ui.slot_sel)) {
        snprintf(msg, sizeof(msg), "SAVE %u OK", (unsigned)(g_preset_ui.slot_sel + 1u));
        set_active_app_status(msg);
        app_settings_save(&g_app_settings_data);
      } else {
        set_active_app_status("SAVE ERR");
      }
    }
    return true;
  }

  return true;
}

static void draw_active_app_preset_screen(void) {
  char line[32];
  bool is_load = g_preset_ui.screen == active_app_preset_load_screen();
  const char* status = active_app_status_text(to_ms_since_boot(get_absolute_time()));

  snprintf(line, sizeof(line), "%s %u/%u %s", app_display_name(g_app_mode),
           (unsigned)(g_preset_ui.screen + 1u), (unsigned)active_app_screen_count(),
           is_load ? "LOAD" : "SAVE");
  hal_io_oled_draw_line(0, line, false);
  for (uint8_t i = 0u; i < APP_PRESET_SLOTS; ++i) {
    snprintf(line, sizeof(line), "%c P%02u %s", i == g_preset_ui.slot_sel ? '>' : ' ',
             (unsigned)(i + 1u), active_app_preset_slot_used(i) ? "USED" : "EMPTY");
    hal_io_oled_draw_line((uint8_t)(2u + i), line, i == g_preset_ui.slot_sel);
  }
  hal_io_oled_draw_line(11, "", false);
  hal_io_oled_draw_line(12, status, false);
  hal_io_oled_draw_line(13, "ENC_R EXEC", false);
  hal_io_oled_draw_line(14, "SW1/SW2 SCREEN", false);
  hal_io_oled_draw_line(15, "ENC_L BACK", false);
}

static void draw_active_app_preset_popup(void) {
  const uint8_t x = 34u;
  const uint8_t y = 36u;
  const uint8_t w = 92u;
  const uint8_t h = 40u;

  hal_io_oled_fill_rect(x, y, w, h, false);
  hal_io_oled_draw_rect(x, y, w, h, true);
  hal_io_oled_draw_text((uint8_t)(x + 22u), (uint8_t)(y + 4u), "PRESETS", false);
  hal_io_oled_draw_text((uint8_t)(x + 8u), (uint8_t)(y + 16u),
                        g_preset_ui.menu_sel == 0u ? "> LOAD" : "  LOAD", false);
  hal_io_oled_draw_text((uint8_t)(x + 8u), (uint8_t)(y + 26u),
                        g_preset_ui.menu_sel == 1u ? "> SAVE" : "  SAVE", false);
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
    } else if (g_menu_index == 4) {
      app_enter(APP_EUCLID);
    } else if (g_menu_index == 5) {
      app_enter(APP_TR2GATE);
    } else if (g_menu_index == 6) {
      app_enter(APP_TR2ADSR);
    } else if (g_menu_index == 7) {
      app_enter(APP_BURSTGEN);
    } else {
      app_enter(APP_CVGEN);
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
    } else if (g_cal_param == CAL_PARAM_MODE) {
      int next = (int)calibration_get_mode(&g_calibration_data) + (int)d_r;
      while (next < 0) next += (int)CAL_RANGE_MODE_COUNT;
      while (next >= (int)CAL_RANGE_MODE_COUNT) next -= (int)CAL_RANGE_MODE_COUNT;
      calibration_set_mode(&g_calibration_data, (cal_range_mode_t)next);
      for (uint8_t ch = 0u; ch < 4u; ++ch) {
        g_hrdw.dac_mv[ch] = clamp_mv(g_hrdw.dac_mv[ch]);
      }
      g_calibration_dirty = true;
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

static void update_tr2gate(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                           uint64_t now_ms) {
  bool edge_src[4];
  sample_trigger_edges(g_tr2gate.src_active, edge_src);
  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
  }

  if (d_l != 0) {
    int next = (int)g_tr2gate.selected_param + (int)d_l;
    while (next < 0) next += (int)TR2GATE_PARAM_COUNT;
    while (next >= (int)TR2GATE_PARAM_COUNT) next -= (int)TR2GATE_PARAM_COUNT;
    g_tr2gate.selected_param = (tr2gate_param_t)next;
  }

  if (d_r != 0) {
    uint8_t ch = g_tr2gate.selected_channel;
    if (g_tr2gate.selected_param == TR2GATE_PARAM_MODE) {
      g_tr2gate.mode = (g_tr2gate.mode == TR2GATE_MODE_DIRECT) ? TR2GATE_MODE_ROUND_ROBIN
                                                                : TR2GATE_MODE_DIRECT;
    } else if (g_tr2gate.selected_param == TR2GATE_PARAM_CHANNEL) {
      g_tr2gate.selected_channel = wrap_u8_delta(g_tr2gate.selected_channel, d_r, 4u);
    } else if (g_tr2gate.selected_param == TR2GATE_PARAM_SOURCE) {
      g_tr2gate.source[ch] = wrap_u8_delta(g_tr2gate.source[ch], d_r, 4u);
    } else if (g_tr2gate.selected_param == TR2GATE_PARAM_TIME) {
      int step = (d_r > 0) ? 10 : -10;
      int next = (int)g_tr2gate.gate_time_cs[ch] + step;
      g_tr2gate.gate_time_cs[ch] =
          (uint16_t)clamp_i(next, TR2GATE_GATE_MIN_CS, TR2GATE_GATE_MAX_CS);
    } else if (g_tr2gate.selected_param == TR2GATE_PARAM_PROB) {
      g_tr2gate.prob[ch] = clamp_u8i_100((int)g_tr2gate.prob[ch] + (int)d_r);
    } else if (g_tr2gate.selected_param == TR2GATE_PARAM_LEVEL) {
      g_tr2gate.level_10v = !g_tr2gate.level_10v;
    }
  }

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    bool fire = edge_src[g_tr2gate.source[ch]];
    if (g_tr2gate.mode == TR2GATE_MODE_ROUND_ROBIN) {
      fire = edge_src[ch] && (g_tr2gate.rr_channel == ch);
    }
    if (fire && prob_pass(g_tr2gate.prob[ch], &g_trigseq.prob_rng_state)) {
      g_tr2gate.gate_out[ch] = true;
      g_tr2gate.gate_off_ms[ch] = now_ms + ((uint64_t)g_tr2gate.gate_time_cs[ch] * 10u);
      if (g_tr2gate.mode == TR2GATE_MODE_ROUND_ROBIN) {
        g_tr2gate.rr_channel = (uint8_t)((g_tr2gate.rr_channel + 1u) & 0x03u);
      }
    }
    if (g_tr2gate.gate_out[ch] && now_ms >= g_tr2gate.gate_off_ms[ch]) {
      g_tr2gate.gate_out[ch] = false;
    }
  }
  tr2gate_apply_outputs();

  if (edge_enc_r) {
    save_tr2gate_settings();
  }
}

static void update_tr2adsr(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                           uint64_t now_ms) {
  bool edge_src[4];
  sample_trigger_edges(g_tr2adsr.src_active, edge_src);
  tr2adsr_refresh_cv_inputs();
  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
  }

  if (d_l != 0) {
    g_tr2adsr.selected_param =
        tr2adsr_next_visible_param(g_tr2adsr.selected_channel, g_tr2adsr.selected_param, d_l);
  }

  if (d_r != 0) {
    uint8_t ch = g_tr2adsr.selected_channel;
    if (g_tr2adsr.selected_param == TR2ADSR_PARAM_CHANNEL) {
      g_tr2adsr.selected_channel = wrap_u8_delta(g_tr2adsr.selected_channel, d_r, 4u);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_SOURCE) {
      g_tr2adsr.source[ch] = wrap_u8_delta(g_tr2adsr.source[ch], d_r, 4u);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_TYPE) {
      g_tr2adsr.type[ch] = (tr2adsr_env_type_t)wrap_u8_delta((uint8_t)g_tr2adsr.type[ch], d_r, 3u);
      if (!tr2adsr_param_visible(ch, g_tr2adsr.selected_param)) {
        g_tr2adsr.selected_param = tr2adsr_next_visible_param(ch, g_tr2adsr.selected_param, 1);
      }
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_TIME) {
      int next = (int)g_tr2adsr.total_time_ds[ch] + ((d_r > 0) ? 1 : -1);
      g_tr2adsr.total_time_ds[ch] =
          (uint16_t)clamp_i(next, TR2ADSR_TIME_MIN_DS, TR2ADSR_TIME_MAX_DS);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_PROB) {
      g_tr2adsr.prob[ch] = clamp_u8i_100((int)g_tr2adsr.prob[ch] + (int)d_r);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_LEVEL) {
      g_tr2adsr.level_10v = !g_tr2adsr.level_10v;
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_ATTACK) {
      g_tr2adsr.attack_pct[ch] = (uint8_t)clamp_i((int)g_tr2adsr.attack_pct[ch] + (int)d_r, 1, 99);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_DECAY) {
      g_tr2adsr.decay_pct[ch] = (uint8_t)clamp_i((int)g_tr2adsr.decay_pct[ch] + (int)d_r, 1, 99);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_SUSTAIN) {
      g_tr2adsr.sustain_pct[ch] = clamp_u8i_100((int)g_tr2adsr.sustain_pct[ch] + (int)d_r);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_RELEASE) {
      g_tr2adsr.release_pct[ch] =
          (uint8_t)clamp_i((int)g_tr2adsr.release_pct[ch] + (int)d_r, 1, 99);
    }
  }

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    uint8_t src = g_tr2adsr.source[ch];
    if (edge_src[src] && prob_pass(g_tr2adsr.prob[ch], &g_euclid.prob_rng_state)) {
      tr2adsr_on_trigger(ch, now_ms);
    }
    {
      bool gate_high = read_trigger_active_src(src) || (g_tr2adsr.gate_hold_until_ms[ch] > now_ms);
      tr2adsr_advance_env(ch, now_ms, gate_high);
    }
  }
  tr2adsr_apply_outputs();

  if (edge_enc_r) {
    save_tr2adsr_settings();
  }
}

static void update_burstgen(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                            uint64_t now_ms) {
  bool edge_src[4];
  sample_trigger_edges(g_burstgen.src_active, edge_src);

  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, false, false, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
    edge_sw1 = false;
    edge_sw2 = false;
  }

  if (d_l != 0) {
    int next = (int)g_burstgen.selected_param + (int)d_l;
    while (next < 0) next += (int)BURSTGEN_PARAM_COUNT;
    while (next >= (int)BURSTGEN_PARAM_COUNT) next -= (int)BURSTGEN_PARAM_COUNT;
    g_burstgen.selected_param = (burstgen_param_t)next;
  }

  if (d_r != 0) {
    if (g_burstgen.selected_param == BURSTGEN_PARAM_CHANNEL) {
      g_burstgen.selected_channel = wrap_u8_delta(g_burstgen.selected_channel, d_r, 4u);
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_SIGNATURE) {
      int next = (int)g_burstgen.signature_mode + ((d_r > 0) ? 1 : -1);
      while (next < 0) next += 4;
      while (next >= 4) next -= 4;
      g_burstgen.signature_mode = (uint8_t)next;
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_BPM) {
      g_burstgen.bpm = (uint16_t)clamp_i((int)g_burstgen.bpm + (int)d_r, BURSTGEN_BPM_MIN, BURSTGEN_BPM_MAX);
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_SWING) {
      g_burstgen.swing_pct =
          (uint8_t)clamp_i((int)g_burstgen.swing_pct + (int)d_r, 0, (int)BURSTGEN_SWING_MAX);
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_PROB) {
      g_burstgen.probability = clamp_u8i_100((int)g_burstgen.probability + (int)d_r);
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_LEVEL) {
      g_burstgen.level_10v = !g_burstgen.level_10v;
    }
  }

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    if (edge_src[ch]) {
      burstgen_start_channel(ch, now_ms);
    }
  }

  if (edge_sw1) {
    burstgen_start_channel(g_burstgen.selected_channel, now_ms);
  }
  if (edge_sw2) {
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
      burstgen_start_channel(ch, now_ms);
    }
  }

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    burstgen_update_channel(ch, now_ms);
  }
  burstgen_apply_outputs();

  if (edge_enc_r) {
    save_burstgen_settings();
  }
}

static void update_cvgen(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                         uint64_t now_ms) {
  uint8_t screen = g_preset_ui.screen;
  uint8_t ch_idx = screen < 4u ? screen : 0u;
  cvgen_channel_state_t* channel = &g_cvgen.ch[ch_idx];

  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
  }

  screen = g_preset_ui.screen;
  ch_idx = screen < 4u ? screen : ch_idx;
  channel = &g_cvgen.ch[ch_idx];

  if (!preset_ui_is_preset_screen()) {
    if (d_l != 0) {
      int next = (int)channel->selected_param + (int)d_l;
      while (next < 0) next += (int)CVGEN_PARAM_COUNT;
      while (next >= (int)CVGEN_PARAM_COUNT) next -= (int)CVGEN_PARAM_COUNT;
      channel->selected_param = (cvgen_param_t)next;
    }

    if (d_r != 0) {
      if (channel->selected_param == CVGEN_PARAM_ALGO) {
        channel->algo = (cvgen_algo_t)wrap_u8_delta((uint8_t)channel->algo, d_r, 4u);
        cvgen_init_channel(channel, now_ms, &g_cvgen.rng_state);
      } else if (channel->selected_param == CVGEN_PARAM_CLOCK) {
        channel->clock_mode = (cvgen_clock_t)wrap_u8_delta((uint8_t)channel->clock_mode, d_r, 2u);
        channel->prev_src_active = read_trigger_active_src(channel->source);
        channel->last_edge_ms = now_ms;
        channel->segment_start_ms = now_ms;
      } else if (channel->selected_param == CVGEN_PARAM_SOURCE) {
        channel->source = wrap_u8_delta(channel->source, d_r, 4u);
        channel->prev_src_active = read_trigger_active_src(channel->source);
      } else if (channel->selected_param == CVGEN_PARAM_RATE) {
        channel->rate_dhz =
            (uint16_t)clamp_i((int)channel->rate_dhz + (int)d_r, CVGEN_RATE_MIN_DHZ, CVGEN_RATE_MAX_DHZ);
      } else if (channel->selected_param == CVGEN_PARAM_MIN) {
        int next = (int)channel->min_mv + ((d_r > 0) ? CVGEN_VOLT_STEP_MV : -CVGEN_VOLT_STEP_MV);
        channel->min_mv =
            (int16_t)clamp_i(next, CVGEN_VOLT_MIN_MV, (int)channel->max_mv - CVGEN_MIN_SPAN_MV);
      } else if (channel->selected_param == CVGEN_PARAM_MAX) {
        int next = (int)channel->max_mv + ((d_r > 0) ? CVGEN_VOLT_STEP_MV : -CVGEN_VOLT_STEP_MV);
        channel->max_mv =
            (int16_t)clamp_i(next, (int)channel->min_mv + CVGEN_MIN_SPAN_MV, CVGEN_VOLT_MAX_MV);
      } else if (channel->selected_param == CVGEN_PARAM_A) {
        channel->param1 = clamp_u8i_100((int)channel->param1 + (int)d_r);
      } else if (channel->selected_param == CVGEN_PARAM_B) {
        if (channel->algo == CVGEN_ALGO_WALK) {
          uint8_t next_mode = wrap_u8_delta(cvgen_walk_mode_from_param(channel->param2), d_r, 3u);
          channel->param2 = cvgen_walk_mode_to_param(next_mode);
        } else {
          channel->param2 = clamp_u8i_100((int)channel->param2 + (int)d_r);
        }
      }
    }
  }

  for (uint8_t i = 0u; i < 4u; ++i) {
    cvgen_update_channel(&g_cvgen.ch[i], now_ms);
  }
  cvgen_apply_outputs();
}

static void update_grids(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                         uint64_t now_ms) {
  bool clk_active;
  bool rst_active;

  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
  }

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
    } else if (g_grids.selected_param == GRIDS_PARAM_CHAOS) {
      g_grids.chaos = clamp_u8i((int)g_grids.chaos + (int)d_r);
    } else {
      uint8_t idx = (uint8_t)((int)g_grids.selected_param - (int)GRIDS_PARAM_PROB1);
      if (idx < 4u) {
        g_grids.prob[idx] = clamp_u8i_100((int)g_grids.prob[idx] + (int)d_r);
      }
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
  g_trigseq.run = true;

  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
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

    if (edge_enc_r) {
      bool now_on = trigseq_grid_get_bit(g_trigseq.cursor_step);
      trigseq_grid_set_bit(g_trigseq.cursor_step, !now_on);
      edge_enc_r = false;
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
        uint8_t idx = (uint8_t)((int)g_trigseq.selected_param - (int)TRIGSEQ_PARAM_PROB1);
        if (idx < 4u) {
          g_trigseq.prob[idx] = clamp_u8i_100((int)g_trigseq.prob[idx] + (int)d_r);
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
    if (clk_active && !g_trigseq.prev_clk_active) {
      trigseq_do_step(now_ms);
    }
    g_trigseq.prev_clk_active = clk_active;
  } else {
    uint32_t interval = trigseq_int_interval_ms();
    uint8_t guard = 0;
    while (now_ms >= g_trigseq.next_int_tick_ms && guard < 8u) {
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

static void update_euclid(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                          uint64_t now_ms) {
  bool clk_active;
  bool rst_active;

  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
  }

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
      uint8_t base = (uint8_t)((int)g_euclid.selected_param - 2);
      uint8_t ch = (uint8_t)(base / 3u);
      uint8_t field = (uint8_t)(base % 3u);  // 0=steps, 1=hits, 2=prob
      if (ch < 4u) {
        if (field == 0u) {
          int next_steps = clamp_i((int)g_euclid.steps[ch] + (int)d_r, 4, 16);
          g_euclid.steps[ch] = (uint8_t)next_steps;
          if (g_euclid.hits[ch] > g_euclid.steps[ch]) {
            g_euclid.hits[ch] = g_euclid.steps[ch];
          }
          g_euclid.phase[ch] %= g_euclid.steps[ch];
        } else if (field == 1u) {
          int next_hits = clamp_i((int)g_euclid.hits[ch] + (int)d_r, 0, (int)g_euclid.steps[ch]);
          g_euclid.hits[ch] = (uint8_t)next_hits;
        } else {
          g_euclid.prob[ch] = clamp_u8i_100((int)g_euclid.prob[ch] + (int)d_r);
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
  clear_rows((uint8_t)(2 + menu_count));
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
  int32_t target_mv = calibration_point_millivolts(&g_calibration_data, g_cal_point);
  bool show_status =
      (g_cal_status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_cal_status[0] != '\0');

  hal_io_oled_draw_line(0, "CALIBRATION", true);

  snprintf(line, sizeof(line), "%c MODE: %s", g_cal_param == CAL_PARAM_MODE ? '>' : ' ',
           calibration_mode_label(calibration_get_mode(&g_calibration_data)));
  hal_io_oled_draw_line(2, line, g_cal_param == CAL_PARAM_MODE);

  snprintf(line, sizeof(line), "%c CH: %c", g_cal_param == CAL_PARAM_CHANNEL ? '>' : ' ',
           (char)('A' + g_cal_channel));
  hal_io_oled_draw_line(3, line, g_cal_param == CAL_PARAM_CHANNEL);

  snprintf(line, sizeof(line), "%c POINT: %s", g_cal_param == CAL_PARAM_POINT ? '>' : ' ',
           calibration_point_label(&g_calibration_data, g_cal_point));
  hal_io_oled_draw_line(4, line, g_cal_param == CAL_PARAM_POINT);

  snprintf(line, sizeof(line), "%c CODE: %4u", g_cal_param == CAL_PARAM_CODE ? '>' : ' ',
           (unsigned)calibration_get_code(&g_calibration_data, g_cal_point, g_cal_channel));
  hal_io_oled_draw_line(5, line, g_cal_param == CAL_PARAM_CODE);

  snprintf(line, sizeof(line), "RANGE %ld..%ldV",
           (long)(calibration_min_millivolts(&g_calibration_data) / 1000),
           (long)(calibration_max_millivolts(&g_calibration_data) / 1000));
  hal_io_oled_draw_line(7, line, false);

  snprintf(line, sizeof(line), "TARGET %ld.%03ldV", (long)(target_mv / 1000),
           (long)(target_mv < 0 ? -(target_mv % 1000) : (target_mv % 1000)));
  hal_io_oled_draw_line(8, line, false);

  hal_io_oled_draw_line(9, "ENC_L PARAM", false);
  hal_io_oled_draw_line(10, "ENC_R VALUE", false);
  hal_io_oled_draw_line(11, "ER_SW SAVE", false);
  hal_io_oled_draw_line(12, "ENC_L_SW BACK", false);

  if (show_status) {
    snprintf(line, sizeof(line), "%s%s", g_cal_status, g_calibration_dirty ? " *" : "");
    hal_io_oled_draw_line(14, line, false);
  } else {
    if (g_calibration_dirty) {
      hal_io_oled_draw_line(14, "DIRTY", false);
    } else {
      hal_io_oled_draw_line(14, "", false);
    }
  }

  clear_rows(15);
}

static void draw_tr2gate(void) {
  char line[32];
  uint8_t ch = g_tr2gate.selected_channel;
  unsigned time_tenths = (unsigned)((g_tr2gate.gate_time_cs[ch] + 5u) / 10u);
  bool show_status =
      (g_tr2gate.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_tr2gate.status[0] != '\0');

  hal_io_oled_draw_line(0, "TR2GATE", true);
  snprintf(line, sizeof(line), "%c MODE: %s", g_tr2gate.selected_param == TR2GATE_PARAM_MODE ? '>' : ' ',
           tr2gate_mode_label(g_tr2gate.mode));
  hal_io_oled_draw_line(2, line, g_tr2gate.selected_param == TR2GATE_PARAM_MODE);
  snprintf(line, sizeof(line), "%c CH: %c", g_tr2gate.selected_param == TR2GATE_PARAM_CHANNEL ? '>' : ' ',
           (char)('A' + ch));
  hal_io_oled_draw_line(3, line, g_tr2gate.selected_param == TR2GATE_PARAM_CHANNEL);
  snprintf(line, sizeof(line), "%c SRC: TR%u", g_tr2gate.selected_param == TR2GATE_PARAM_SOURCE ? '>' : ' ',
           (unsigned)(g_tr2gate.source[ch] + 1u));
  hal_io_oled_draw_line(4, line, g_tr2gate.selected_param == TR2GATE_PARAM_SOURCE);
  snprintf(line, sizeof(line), "%c TIME: %u.%1us", g_tr2gate.selected_param == TR2GATE_PARAM_TIME ? '>' : ' ',
           time_tenths / 10u, time_tenths % 10u);
  hal_io_oled_draw_line(5, line, g_tr2gate.selected_param == TR2GATE_PARAM_TIME);
  snprintf(line, sizeof(line), "%c PROB: %u", g_tr2gate.selected_param == TR2GATE_PARAM_PROB ? '>' : ' ',
           (unsigned)g_tr2gate.prob[ch]);
  hal_io_oled_draw_line(6, line, g_tr2gate.selected_param == TR2GATE_PARAM_PROB);
  snprintf(line, sizeof(line), "%c LEVEL: %s", g_tr2gate.selected_param == TR2GATE_PARAM_LEVEL ? '>' : ' ',
           g_tr2gate.level_10v ? "10V" : "5V");
  hal_io_oled_draw_line(7, line, g_tr2gate.selected_param == TR2GATE_PARAM_LEVEL);
  snprintf(line, sizeof(line), "TR IN: %u%u%u%u", read_trigger_active_src(0) ? 1u : 0u,
           read_trigger_active_src(1) ? 1u : 0u, read_trigger_active_src(2) ? 1u : 0u,
           read_trigger_active_src(3) ? 1u : 0u);
  hal_io_oled_draw_line(9, line, false);
  snprintf(line, sizeof(line), "GT OUT:%u%u%u%u", g_tr2gate.gate_out[0] ? 1u : 0u,
           g_tr2gate.gate_out[1] ? 1u : 0u, g_tr2gate.gate_out[2] ? 1u : 0u,
           g_tr2gate.gate_out[3] ? 1u : 0u);
  hal_io_oled_draw_line(10, line, false);
  hal_io_oled_draw_line(12, "", false);
  hal_io_oled_draw_line(13, "", false);
  if (show_status) {
    hal_io_oled_draw_line(14, g_tr2gate.status, false);
  } else {
    hal_io_oled_draw_line(14, "", false);
  }
  hal_io_oled_draw_line(15, "", false);
}

static void draw_tr2adsr(void) {
  char line[32];
  uint8_t ch = g_tr2adsr.selected_channel;
  uint8_t a_eff = tr2adsr_effective_attack_pct(ch);
  uint8_t d_eff = tr2adsr_effective_decay_pct(ch);
  uint8_t s_eff = tr2adsr_effective_sustain_pct(ch);
  uint8_t r_eff = tr2adsr_effective_release_pct(ch);
  bool show_status =
      (g_tr2adsr.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_tr2adsr.status[0] != '\0');

  hal_io_oled_draw_line(0, "TR2ADSR", true);
  snprintf(line, sizeof(line), "%c CH: %c", g_tr2adsr.selected_param == TR2ADSR_PARAM_CHANNEL ? '>' : ' ',
           (char)('A' + ch));
  hal_io_oled_draw_line(2, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_CHANNEL);
  snprintf(line, sizeof(line), "%c SRC: TR%u", g_tr2adsr.selected_param == TR2ADSR_PARAM_SOURCE ? '>' : ' ',
           (unsigned)(g_tr2adsr.source[ch] + 1u));
  hal_io_oled_draw_line(3, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_SOURCE);
  snprintf(line, sizeof(line), "%c TYPE: %s", g_tr2adsr.selected_param == TR2ADSR_PARAM_TYPE ? '>' : ' ',
           tr2adsr_type_label(g_tr2adsr.type[ch]));
  hal_io_oled_draw_line(4, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_TYPE);
  snprintf(line, sizeof(line), "%c TIME: %u.%01us", g_tr2adsr.selected_param == TR2ADSR_PARAM_TIME ? '>' : ' ',
           (unsigned)(g_tr2adsr.total_time_ds[ch] / 10u), (unsigned)(g_tr2adsr.total_time_ds[ch] % 10u));
  hal_io_oled_draw_line(5, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_TIME);
  snprintf(line, sizeof(line), "%c PROB: %u", g_tr2adsr.selected_param == TR2ADSR_PARAM_PROB ? '>' : ' ',
           (unsigned)g_tr2adsr.prob[ch]);
  hal_io_oled_draw_line(6, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_PROB);
  snprintf(line, sizeof(line), "%c LEVEL: %s", g_tr2adsr.selected_param == TR2ADSR_PARAM_LEVEL ? '>' : ' ',
           g_tr2adsr.level_10v ? "10V" : "5V");
  hal_io_oled_draw_line(7, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_LEVEL);
  snprintf(line, sizeof(line), "%c A: %u", g_tr2adsr.selected_param == TR2ADSR_PARAM_ATTACK ? '>' : ' ',
           (unsigned)a_eff);
  hal_io_oled_draw_line(8, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_ATTACK);
  if (tr2adsr_param_visible(ch, TR2ADSR_PARAM_DECAY)) {
    snprintf(line, sizeof(line), "%c D: %u", g_tr2adsr.selected_param == TR2ADSR_PARAM_DECAY ? '>' : ' ',
             (unsigned)d_eff);
  } else {
    snprintf(line, sizeof(line), "  D: --");
  }
  hal_io_oled_draw_line(9, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_DECAY);
  if (tr2adsr_param_visible(ch, TR2ADSR_PARAM_SUSTAIN)) {
    snprintf(line, sizeof(line), "%c S: %u", g_tr2adsr.selected_param == TR2ADSR_PARAM_SUSTAIN ? '>' : ' ',
             (unsigned)s_eff);
  } else {
    snprintf(line, sizeof(line), "  S: --");
  }
  hal_io_oled_draw_line(10, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_SUSTAIN);
  snprintf(line, sizeof(line), "%c R: %u", g_tr2adsr.selected_param == TR2ADSR_PARAM_RELEASE ? '>' : ' ',
           (unsigned)r_eff);
  hal_io_oled_draw_line(11, line, g_tr2adsr.selected_param == TR2ADSR_PARAM_RELEASE);
  hal_io_oled_draw_line(12, "", false);
  hal_io_oled_draw_line(13, "", false);
  snprintf(line, sizeof(line), "ENV %s LVL %.2f", tr2adsr_state_label(g_tr2adsr.env_state[ch]), g_tr2adsr.level[ch]);
  hal_io_oled_draw_line(14, line, false);
  if (show_status) {
    hal_io_oled_draw_line(15, g_tr2adsr.status, false);
  } else {
    hal_io_oled_draw_line(15, "", false);
  }
}

static void draw_burstgen(void) {
  char line[32];
  char pattern[12];
  uint8_t ch = g_burstgen.selected_channel;
  uint8_t steps = burstgen_steps_for_mode(g_burstgen.signature_mode);
  bool show_status =
      (g_burstgen.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_burstgen.status[0] != '\0');

  burstgen_format_pattern(pattern, sizeof(pattern), g_burstgen.pattern_mask[ch], steps);

  hal_io_oled_draw_line(0, "BURST GEN", true);
  snprintf(line, sizeof(line), "%c CH: %c", g_burstgen.selected_param == BURSTGEN_PARAM_CHANNEL ? '>' : ' ',
           (char)('A' + ch));
  hal_io_oled_draw_line(2, line, g_burstgen.selected_param == BURSTGEN_PARAM_CHANNEL);
  snprintf(line, sizeof(line), "%c SIG: %s", g_burstgen.selected_param == BURSTGEN_PARAM_SIGNATURE ? '>' : ' ',
           burstgen_signature_label(g_burstgen.signature_mode));
  hal_io_oled_draw_line(3, line, g_burstgen.selected_param == BURSTGEN_PARAM_SIGNATURE);
  snprintf(line, sizeof(line), "%c BPM: %u", g_burstgen.selected_param == BURSTGEN_PARAM_BPM ? '>' : ' ',
           (unsigned)g_burstgen.bpm);
  hal_io_oled_draw_line(4, line, g_burstgen.selected_param == BURSTGEN_PARAM_BPM);
  snprintf(line, sizeof(line), "%c SWG: %u", g_burstgen.selected_param == BURSTGEN_PARAM_SWING ? '>' : ' ',
           (unsigned)g_burstgen.swing_pct);
  hal_io_oled_draw_line(5, line, g_burstgen.selected_param == BURSTGEN_PARAM_SWING);
  snprintf(line, sizeof(line), "%c PRB: %u", g_burstgen.selected_param == BURSTGEN_PARAM_PROB ? '>' : ' ',
           (unsigned)g_burstgen.probability);
  hal_io_oled_draw_line(6, line, g_burstgen.selected_param == BURSTGEN_PARAM_PROB);
  snprintf(line, sizeof(line), "%c LVL: %s", g_burstgen.selected_param == BURSTGEN_PARAM_LEVEL ? '>' : ' ',
           g_burstgen.level_10v ? "10V" : "5V");
  hal_io_oled_draw_line(7, line, g_burstgen.selected_param == BURSTGEN_PARAM_LEVEL);
  snprintf(line, sizeof(line), "RUN: %u%u%u%u", g_burstgen.running[0] ? 1u : 0u, g_burstgen.running[1] ? 1u : 0u,
           g_burstgen.running[2] ? 1u : 0u, g_burstgen.running[3] ? 1u : 0u);
  hal_io_oled_draw_line(9, line, false);
  snprintf(line, sizeof(line), "IN : %u%u%u%u", read_trigger_active_src(0) ? 1u : 0u, read_trigger_active_src(1) ? 1u : 0u,
           read_trigger_active_src(2) ? 1u : 0u, read_trigger_active_src(3) ? 1u : 0u);
  hal_io_oled_draw_line(10, line, false);
  snprintf(line, sizeof(line), "P%c %s %u/%u", (char)('A' + ch), pattern, (unsigned)(g_burstgen.current_step[ch] + 1u),
           (unsigned)steps);
  hal_io_oled_draw_line(11, line, false);
  if (show_status) {
    hal_io_oled_draw_line(12, g_burstgen.status, false);
  } else {
    hal_io_oled_draw_line(12, "", false);
  }
  hal_io_oled_draw_line(13, "ER SAVE", false);
  hal_io_oled_draw_line(14, "SW1 FIRE CH", false);
  hal_io_oled_draw_line(15, "SW2 FIRE ALL", false);
}

static void draw_cvgen(void) {
  char line[32];
  char min_str[12];
  char max_str[12];
  char out_str[12];
  uint8_t ch_idx = g_preset_ui.screen < 4u ? g_preset_ui.screen : 0u;
  cvgen_channel_state_t* channel = &g_cvgen.ch[ch_idx];
  uint8_t walk_mode = cvgen_walk_mode_from_param(channel->param2);
  bool show_status =
      (g_cvgen.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_cvgen.status[0] != '\0');

  cvgen_format_mv(min_str, sizeof(min_str), channel->min_mv);
  cvgen_format_mv(max_str, sizeof(max_str), channel->max_mv);
  cvgen_format_mv(out_str, sizeof(out_str), cvgen_map_norm_to_mv(channel, channel->output_norm));

  snprintf(line, sizeof(line), "CV GEN %u/%u CH %c", (unsigned)(g_preset_ui.screen + 1u),
           (unsigned)active_app_screen_count(), (char)('A' + ch_idx));
  hal_io_oled_draw_line(0, line, true);

  snprintf(line, sizeof(line), "%c ALG: %s", channel->selected_param == CVGEN_PARAM_ALGO ? '>' : ' ',
           cvgen_algo_label(channel->algo));
  hal_io_oled_draw_line(2, line, channel->selected_param == CVGEN_PARAM_ALGO);
  snprintf(line, sizeof(line), "%c CLK: %s", channel->selected_param == CVGEN_PARAM_CLOCK ? '>' : ' ',
           cvgen_clock_label(channel->clock_mode));
  hal_io_oled_draw_line(3, line, channel->selected_param == CVGEN_PARAM_CLOCK);
  snprintf(line, sizeof(line), "%c SRC: TR%u", channel->selected_param == CVGEN_PARAM_SOURCE ? '>' : ' ',
           (unsigned)(channel->source + 1u));
  hal_io_oled_draw_line(4, line, channel->selected_param == CVGEN_PARAM_SOURCE);
  snprintf(line, sizeof(line), "%c RATE: %u.%01uHz", channel->selected_param == CVGEN_PARAM_RATE ? '>' : ' ',
           (unsigned)(channel->rate_dhz / 10u), (unsigned)(channel->rate_dhz % 10u));
  hal_io_oled_draw_line(5, line, channel->selected_param == CVGEN_PARAM_RATE);
  snprintf(line, sizeof(line), "%c MIN: %s", channel->selected_param == CVGEN_PARAM_MIN ? '>' : ' ', min_str);
  hal_io_oled_draw_line(6, line, channel->selected_param == CVGEN_PARAM_MIN);
  snprintf(line, sizeof(line), "%c MAX: %s", channel->selected_param == CVGEN_PARAM_MAX ? '>' : ' ', max_str);
  hal_io_oled_draw_line(7, line, channel->selected_param == CVGEN_PARAM_MAX);
  snprintf(line, sizeof(line), "%c %s: %u", channel->selected_param == CVGEN_PARAM_A ? '>' : ' ',
           cvgen_param_a_label(channel->algo), (unsigned)channel->param1);
  hal_io_oled_draw_line(8, line, channel->selected_param == CVGEN_PARAM_A);
  if (channel->algo == CVGEN_ALGO_WALK) {
    snprintf(line, sizeof(line), "%c %s: %s", channel->selected_param == CVGEN_PARAM_B ? '>' : ' ',
             cvgen_param_b_label(channel->algo), cvgen_walk_mode_label(walk_mode));
  } else {
    snprintf(line, sizeof(line), "%c %s: %u", channel->selected_param == CVGEN_PARAM_B ? '>' : ' ',
             cvgen_param_b_label(channel->algo), (unsigned)channel->param2);
  }
  hal_io_oled_draw_line(9, line, channel->selected_param == CVGEN_PARAM_B);

  snprintf(line, sizeof(line), "OUT: %s", out_str);
  hal_io_oled_draw_line(11, line, false);
  if (show_status) {
    hal_io_oled_draw_line(12, g_cvgen.status, false);
  } else {
    hal_io_oled_draw_line(12, "", false);
  }
  hal_io_oled_draw_line(13, channel->clock_mode == CVGEN_CLOCK_EXT ? "EXT=TR1..TR4" : "INT FREE RUN", false);
  hal_io_oled_draw_line(14, "SW1/SW2 SCREEN", false);
  hal_io_oled_draw_line(15, "ENC_L_SW BACK", false);
}

static void draw_grids(void) {
  char line[32];
  char token[16];
  bool show_status =
      (g_grids.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_grids.status[0] != '\0');
  bool preview[4][32];
  bool progress[4][32];
  uint8_t prob1 = g_grids.prob[0];
  uint8_t prob2 = g_grids.prob[1];
  uint8_t prob3 = g_grids.prob[2];
  uint8_t prob4 = g_grids.prob[3];
  const uint8_t grid_x = 14u;
  const uint8_t grid_top = 49u;
  const uint8_t channel_stride = 14u;  // 2 rows * 6px + 2px visual gap between channels
  const uint8_t cell_pitch = 6u;
  const uint8_t cell_size = 6u;

  hal_io_oled_draw_line(0, "GRIDS", true);
  hal_io_oled_draw_line(1, "", false);

  snprintf(line, sizeof(line), "CLK:%s BPM:%3d CHAOS:%3u", g_grids.clock == GRIDS_CLOCK_INT ? "INT" : "EXT",
           g_grids.bpm, (unsigned)g_grids.chaos);
  hal_io_oled_draw_line(2, line, false);
  if (g_grids.selected_param == GRIDS_PARAM_CLOCK) {
    snprintf(token, sizeof(token), "CLK:%s", g_grids.clock == GRIDS_CLOCK_INT ? "INT" : "EXT");
    hal_io_oled_draw_text(0u, 16u, token, true);
  } else if (g_grids.selected_param == GRIDS_PARAM_BPM) {
    snprintf(token, sizeof(token), "BPM:%3d", g_grids.bpm);
    hal_io_oled_draw_text((uint8_t)(8u * 6u), 16u, token, true);
  } else if (g_grids.selected_param == GRIDS_PARAM_CHAOS) {
    snprintf(token, sizeof(token), "CHAOS:%3u", (unsigned)g_grids.chaos);
    hal_io_oled_draw_text((uint8_t)(16u * 6u), 16u, token, true);
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

  snprintf(line, sizeof(line), "PROB1:%3u PROB2:%3u", (unsigned)prob1, (unsigned)prob2);
  hal_io_oled_draw_line(4, line, false);
  snprintf(line, sizeof(line), "PROB3:%3u PROB4:%3u", (unsigned)prob3, (unsigned)prob4);
  hal_io_oled_draw_line(5, line, false);
  if (g_grids.selected_param == GRIDS_PARAM_PROB1) {
    snprintf(token, sizeof(token), "PROB1:%3u", (unsigned)prob1);
    hal_io_oled_draw_text(0u, 32u, token, true);
  } else if (g_grids.selected_param == GRIDS_PARAM_PROB2) {
    snprintf(token, sizeof(token), "PROB2:%3u", (unsigned)prob2);
    hal_io_oled_draw_text((uint8_t)(10u * 6u), 32u, token, true);
  } else if (g_grids.selected_param == GRIDS_PARAM_PROB3) {
    snprintf(token, sizeof(token), "PROB3:%3u", (unsigned)prob3);
    hal_io_oled_draw_text(0u, 40u, token, true);
  } else if (g_grids.selected_param == GRIDS_PARAM_PROB4) {
    snprintf(token, sizeof(token), "PROB4:%3u", (unsigned)prob4);
    hal_io_oled_draw_text((uint8_t)(10u * 6u), 40u, token, true);
  }

  {
    grids_engine_t sim = g_grids.engine;
    uint32_t sim_prob_rng = g_grids.prob_rng_state;
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
        preview[ch][idx] = trig[ch] && prob_pass(g_grids.prob[ch], &sim_prob_rng);
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
    hal_io_oled_fill_rect(0u, 49u, 160u, 55u, false);
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
  hal_io_oled_draw_line(14, "SW1/SW2 SCREEN", false);
  hal_io_oled_draw_line(15, "ER PRESETS EL BACK", false);
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
  char token[20];
  uint8_t prob1 = g_trigseq.prob[0];
  uint8_t prob2 = g_trigseq.prob[1];
  uint8_t prob3 = g_trigseq.prob[2];
  uint8_t prob4 = g_trigseq.prob[3];
  bool force_grid = !g_trigseq.grid_cache_valid || (g_trigseq.grid_cache_mode != g_trigseq.len_mode);
  bool show_status =
      (g_trigseq.status_until_ms > to_ms_since_boot(get_absolute_time())) &&
      (g_trigseq.status[0] != '\0');

  snprintf(line, sizeof(line), "TRIG SEQ %u/%u %s", (unsigned)(g_preset_ui.screen + 1u),
           (unsigned)active_app_screen_count(),
           g_trigseq.focus == TRIGSEQ_FOCUS_GRID ? "GRID" : "MENU");
  hal_io_oled_draw_line(0, line, true);

  snprintf(line, sizeof(line), "LEN:%s", trigseq_mode_label(g_trigseq.len_mode));
  hal_io_oled_draw_line(1, line, false);
  if (g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_LEN) {
    snprintf(token, sizeof(token), "LEN:%s", trigseq_mode_label(g_trigseq.len_mode));
    hal_io_oled_draw_text(0u, 8u, token, true);
  }

  if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
    snprintf(line, sizeof(line), "CLOCK:%s BPM:%3d", g_trigseq.clock == TRIGSEQ_CLOCK_INT ? "INT" : "EXT",
             g_trigseq.bpm);
  } else {
    snprintf(line, sizeof(line), "CLOCK:%s BPM:---", g_trigseq.clock == TRIGSEQ_CLOCK_INT ? "INT" : "EXT");
  }
  hal_io_oled_draw_line(2, line, false);
  if (g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_CLOCK) {
    snprintf(token, sizeof(token), "CLOCK:%s", g_trigseq.clock == TRIGSEQ_CLOCK_INT ? "INT" : "EXT");
    hal_io_oled_draw_text(0u, 16u, token, true);
  } else if (g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_BPM) {
    if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
      snprintf(token, sizeof(token), "BPM:%3d", g_trigseq.bpm);
    } else {
      snprintf(token, sizeof(token), "BPM:---");
    }
    hal_io_oled_draw_text((uint8_t)(10u * 6u), 16u, token, true);
  }

  snprintf(line, sizeof(line), "PROB1:%3u PROB2:%3u", (unsigned)prob1, (unsigned)prob2);
  hal_io_oled_draw_line(3, line, false);
  snprintf(line, sizeof(line), "PROB3:%3u PROB4:%3u", (unsigned)prob3, (unsigned)prob4);
  hal_io_oled_draw_line(4, line, false);
  if (g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_PROB1) {
    snprintf(token, sizeof(token), "PROB1:%3u", (unsigned)prob1);
    hal_io_oled_draw_text(0u, 24u, token, true);
  } else if (g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_PROB2) {
    snprintf(token, sizeof(token), "PROB2:%3u", (unsigned)prob2);
    hal_io_oled_draw_text((uint8_t)(10u * 6u), 24u, token, true);
  } else if (g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_PROB3) {
    snprintf(token, sizeof(token), "PROB3:%3u", (unsigned)prob3);
    hal_io_oled_draw_text(0u, 32u, token, true);
  } else if (g_trigseq.focus == TRIGSEQ_FOCUS_MENU && g_trigseq.selected_param == TRIGSEQ_PARAM_PROB4) {
    snprintf(token, sizeof(token), "PROB4:%3u", (unsigned)prob4);
    hal_io_oled_draw_text((uint8_t)(10u * 6u), 32u, token, true);
  }

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
  hal_io_oled_draw_line(12, "SW1/SW2 SCREEN", false);
  hal_io_oled_draw_line(13, g_trigseq.focus == TRIGSEQ_FOCUS_GRID ? "ER ON/OFF" : "", false);
  hal_io_oled_draw_line(14, g_trigseq.focus == TRIGSEQ_FOCUS_GRID ? "" : "ER PRESETS", false);
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
  bool sel_c1p = g_euclid.selected_param == EUCLID_PARAM_CH1_PRB;
  bool sel_c2s = g_euclid.selected_param == EUCLID_PARAM_CH2_STEPS;
  bool sel_c2h = g_euclid.selected_param == EUCLID_PARAM_CH2_HITS;
  bool sel_c2p = g_euclid.selected_param == EUCLID_PARAM_CH2_PRB;
  bool sel_c3s = g_euclid.selected_param == EUCLID_PARAM_CH3_STEPS;
  bool sel_c3h = g_euclid.selected_param == EUCLID_PARAM_CH3_HITS;
  bool sel_c3p = g_euclid.selected_param == EUCLID_PARAM_CH3_PRB;
  bool sel_c4s = g_euclid.selected_param == EUCLID_PARAM_CH4_STEPS;
  bool sel_c4h = g_euclid.selected_param == EUCLID_PARAM_CH4_HITS;
  bool sel_c4p = g_euclid.selected_param == EUCLID_PARAM_CH4_PRB;

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

  snprintf(line, sizeof(line), "1 STPS:%2u HITS:%2u PRB:%3u", g_euclid.steps[0], g_euclid.hits[0], g_euclid.prob[0]);
  oled_draw_text_26(y_ch1, line, false);
  if (sel_c1s) {
    snprintf(token, sizeof(token), "STPS:%2u", g_euclid.steps[0]);
    hal_io_oled_draw_text((uint8_t)(2u * 6u), y_ch1, token, true);
  } else if (sel_c1h) {
    snprintf(token, sizeof(token), "HITS:%2u", g_euclid.hits[0]);
    hal_io_oled_draw_text((uint8_t)(10u * 6u), y_ch1, token, true);
  } else if (sel_c1p) {
    snprintf(token, sizeof(token), "PRB:%3u", g_euclid.prob[0]);
    hal_io_oled_draw_text((uint8_t)(18u * 6u), y_ch1, token, true);
  }
  snprintf(line, sizeof(line), "2 STPS:%2u HITS:%2u PRB:%3u", g_euclid.steps[1], g_euclid.hits[1], g_euclid.prob[1]);
  oled_draw_text_26(y_ch2, line, false);
  if (sel_c2s) {
    snprintf(token, sizeof(token), "STPS:%2u", g_euclid.steps[1]);
    hal_io_oled_draw_text((uint8_t)(2u * 6u), y_ch2, token, true);
  } else if (sel_c2h) {
    snprintf(token, sizeof(token), "HITS:%2u", g_euclid.hits[1]);
    hal_io_oled_draw_text((uint8_t)(10u * 6u), y_ch2, token, true);
  } else if (sel_c2p) {
    snprintf(token, sizeof(token), "PRB:%3u", g_euclid.prob[1]);
    hal_io_oled_draw_text((uint8_t)(18u * 6u), y_ch2, token, true);
  }
  snprintf(line, sizeof(line), "3 STPS:%2u HITS:%2u PRB:%3u", g_euclid.steps[2], g_euclid.hits[2], g_euclid.prob[2]);
  oled_draw_text_26(y_ch3, line, false);
  if (sel_c3s) {
    snprintf(token, sizeof(token), "STPS:%2u", g_euclid.steps[2]);
    hal_io_oled_draw_text((uint8_t)(2u * 6u), y_ch3, token, true);
  } else if (sel_c3h) {
    snprintf(token, sizeof(token), "HITS:%2u", g_euclid.hits[2]);
    hal_io_oled_draw_text((uint8_t)(10u * 6u), y_ch3, token, true);
  } else if (sel_c3p) {
    snprintf(token, sizeof(token), "PRB:%3u", g_euclid.prob[2]);
    hal_io_oled_draw_text((uint8_t)(18u * 6u), y_ch3, token, true);
  }
  snprintf(line, sizeof(line), "4 STPS:%2u HITS:%2u PRB:%3u", g_euclid.steps[3], g_euclid.hits[3], g_euclid.prob[3]);
  oled_draw_text_26(y_ch4, line, false);
  if (sel_c4s) {
    snprintf(token, sizeof(token), "STPS:%2u", g_euclid.steps[3]);
    hal_io_oled_draw_text((uint8_t)(2u * 6u), y_ch4, token, true);
  } else if (sel_c4h) {
    snprintf(token, sizeof(token), "HITS:%2u", g_euclid.hits[3]);
    hal_io_oled_draw_text((uint8_t)(10u * 6u), y_ch4, token, true);
  } else if (sel_c4p) {
    snprintf(token, sizeof(token), "PRB:%3u", g_euclid.prob[3]);
    hal_io_oled_draw_text((uint8_t)(18u * 6u), y_ch4, token, true);
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
  hal_io_oled_draw_line(14, "SW1/SW2 SCREEN", false);
  hal_io_oled_draw_line(15, "ENC_L_SW BACK", false);
}

static void service_active_app_pulses(uint64_t now_ms) {
  if (g_app_mode == APP_GRIDS) {
    grids_update_pulses(now_ms);
  } else if (g_app_mode == APP_TRIGSEQ) {
    trigseq_update_pulses(now_ms);
  } else if (g_app_mode == APP_EUCLID) {
    euclid_update_pulses(now_ms);
  } else if (g_app_mode == APP_BURSTGEN) {
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
      burstgen_update_channel(ch, now_ms);
    }
    burstgen_apply_outputs();
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
  if (g_app_mode == APP_BURSTGEN) {
    return g_burstgen.gate_out[0] || g_burstgen.gate_out[1] || g_burstgen.gate_out[2] || g_burstgen.gate_out[3];
  }
  return false;
}

static bool active_app_timing_priority_block_draw(uint64_t now_ms) {
  const uint64_t tick_guard_ms = 4u;

  if (active_app_has_live_pulse()) return true;

  if (g_app_mode == APP_GRIDS) {
    if (g_grids.clock == GRIDS_CLOCK_INT && g_grids.next_int_tick_ms <= (now_ms + tick_guard_ms)) {
      return true;
    }
    if (g_grids.clock == GRIDS_CLOCK_EXT && hal_io_trigger_active(HAL_IO_TR1)) {
      return true;
    }
  } else if (g_app_mode == APP_TRIGSEQ) {
    if (g_trigseq.clock == TRIGSEQ_CLOCK_INT && g_trigseq.next_int_tick_ms <= (now_ms + tick_guard_ms)) {
      return true;
    }
    if (g_trigseq.clock == TRIGSEQ_CLOCK_EXT && hal_io_trigger_active(HAL_IO_TR1)) {
      return true;
    }
  } else if (g_app_mode == APP_EUCLID) {
    if (g_euclid.clock == EUCLID_CLOCK_INT && g_euclid.next_int_tick_ms <= (now_ms + tick_guard_ms)) {
      return true;
    }
    if (g_euclid.clock == EUCLID_CLOCK_EXT && hal_io_trigger_active(HAL_IO_TR1)) {
      return true;
    }
  } else if (g_app_mode == APP_BURSTGEN) {
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
      if (g_burstgen.running[ch] && g_burstgen.next_step_at_ms[ch] <= (now_ms + tick_guard_ms)) {
        return true;
      }
    }
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
  app_presets_init();
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
      if (g_preset_ui.menu_open) {
        preset_ui_close_menu();
      } else if (g_preset_ui.screen != APP_SCREEN_MAIN) {
        preset_ui_set_screen(APP_SCREEN_MAIN);
      } else {
        save_active_app_runtime_settings_quiet();
        app_enter(APP_MENU);
      }
    } else if (g_app_mode == APP_MENU) {
      update_menu(d_l, edge_enc_r);
    } else if (g_app_mode == APP_HRDW_TEST) {
      update_hrdw_test(d_l, d_r, edge_sw1);
      (void)edge_enc_r;
      (void)edge_sw2;
    } else if (g_app_mode == APP_CALIBRATION) {
      update_calibration(d_l, d_r, edge_enc_r);
      (void)edge_sw1;
      (void)edge_sw2;
    } else if (g_app_mode == APP_GRIDS) {
      update_grids(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms);
    } else if (g_app_mode == APP_TRIGSEQ) {
      update_trigseq(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms);
    } else if (g_app_mode == APP_EUCLID) {
      update_euclid(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms);
    } else if (g_app_mode == APP_TR2GATE) {
      update_tr2gate(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms);
    } else if (g_app_mode == APP_TR2ADSR) {
      update_tr2adsr(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms);
    } else if (g_app_mode == APP_BURSTGEN) {
      update_burstgen(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms);
    } else {
      update_cvgen(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms);
    }

    if ((now_ms - last_draw_ms) >= DRAW_PERIOD_MS) {
      uint64_t draw_now = to_ms_since_boot(get_absolute_time());
      service_active_app_pulses(draw_now);

      // Sequencer timing has higher priority than UI rendering.
      // If a pulse is active, external clock is high, or internal tick is imminent,
      // skip draw and return quickly to timing update path.
      if (active_app_timing_priority_block_draw(draw_now)) {
        sleep_ms(LOOP_SLEEP_MS);
        tight_loop_contents();
        continue;
      }

      if (g_app_mode == APP_MENU) {
        draw_menu();
      } else if (g_preset_ui.menu_open) {
        if (g_preset_ui.popup_dirty) {
          draw_active_app_preset_popup();
          g_preset_ui.popup_dirty = false;
        }
      } else {
        if (preset_ui_is_preset_screen()) {
          draw_active_app_preset_screen();
        } else if (g_app_mode == APP_HRDW_TEST) {
          draw_hrdw_test();
        } else if (g_app_mode == APP_CALIBRATION) {
          draw_calibration();
        } else if (g_app_mode == APP_GRIDS) {
          draw_grids();
        } else if (g_app_mode == APP_TRIGSEQ) {
          draw_trigseq();
        } else if (g_app_mode == APP_EUCLID) {
          draw_euclid();
        } else if (g_app_mode == APP_TR2GATE) {
          draw_tr2gate();
        } else if (g_app_mode == APP_TR2ADSR) {
          draw_tr2adsr();
        } else if (g_app_mode == APP_BURSTGEN) {
          draw_burstgen();
        } else {
          draw_cvgen();
        }
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
