#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_presets.h"
#include "app_settings.h"
#include "calibration.h"
#include "clock_input.h"
#include "firmware_version.h"
#include "grids_engine.h"
#include "hal_io.h"
#include "hal_mux_adc.h"
#include "pico/stdlib.h"
#include "trigger_output.h"
#include "trigseq_engine.h"

#define DRAW_PERIOD_MS 120u
#define LOOP_SLEEP_MS 1u
#define ENABLE_TIMING_DIAG_PRINT 0
#define TIMING_DIAG_PRINT_PERIOD_MS 1000u
#define TRIGGER_OUTPUT_TIMER_US 500
#define CLOCK_INPUT_MIN_INTERVAL_US 500u
#define CV_CACHE_PERIOD_US 8000u
#define GRIDS_TRIG_HIGH_MV 5000
#define GRIDS_TRIG_LOW_MV 0
#define GRIDS_TRIG_PULSE_US 15000u
#define GRIDS_BPM_MIN 30
#define GRIDS_BPM_MAX 300
#define TRIGSEQ_TRIG_HIGH_MV 5000
#define TRIGSEQ_TRIG_LOW_MV 0
#define TRIGSEQ_TRIG_PULSE_US 15000u
#define TRIGSEQ_BPM_MIN 30
#define TRIGSEQ_BPM_MAX 300
#define EUCLID_TRIG_HIGH_MV 5000
#define EUCLID_TRIG_LOW_MV 0
#define EUCLID_TRIG_PULSE_US 15000u
#define EUCLID_BPM_MIN 30
#define EUCLID_BPM_MAX 300
#define TR2GATE_GATE_MIN_CS 10u
#define TR2GATE_GATE_MAX_CS 3000u
#define TR2ADSR_TIME_MIN_DS 1u
#define TR2ADSR_TIME_MAX_DS 100u
#define BURSTGEN_BPM_MIN 40
#define BURSTGEN_BPM_MAX 220
#define BURSTGEN_SWING_MAX 40u
#define BURSTGEN_PULSE_US 35000u
#define BURSTGEN_MAX_CATCHUP_STEPS 4u
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
  APP_CALIBRATION = 1,
  APP_NOTES = 2,
  APP_VOLTS = 3,
  APP_GRIDS = 4,
  APP_TRIGSEQ = 5,
  APP_EUCLID = 6,
  APP_TR2GATE = 7,
  APP_TR2ADSR = 8,
  APP_BURSTGEN = 9,
  APP_CVGEN = 10,
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
  TRIGSEQ_PARAM_EDIT_GRID = 7,
  TRIGSEQ_PARAM_COUNT = 8,
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
  uint64_t next_cv_cache_us;
  uint32_t prob_rng_state;
  uint64_t next_int_tick_us;
  uint32_t step_count;
  bool prev_rst_active;
  bool preview_cache_valid;
  bool preview_cache_bits[4][32];
  bool preview_cache_progress[4][32];
  bool ui_cache_valid;
  bool ui_force_full_redraw;
  bool preview_force_full_redraw;
  bool ui_header_dirty;
  bool ui_token_dirty[9];
  bool ui_status_dirty;
  char ui_header[27];
  char ui_token_text[9][16];
  bool ui_token_selected[9];
  char ui_status[27];
  bool ui_status_visible;
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
  uint64_t next_int_tick_us;
  trigseq_focus_t focus;
  trigseq_param_t selected_param;
  bool grid_cache_valid;
  trigseq_len_mode_t grid_cache_mode;
  uint8_t grid_cache_cursor;
  bool grid_cache_bits[64];
  bool grid_cache_progress[64];
  bool ui_cache_valid;
  bool ui_force_full_redraw;
  bool ui_header_dirty;
  bool ui_token_dirty[8];
  bool ui_status_dirty;
  bool ui_clk_src_dirty;
  char ui_header[27];
  char ui_token_text[8][20];
  bool ui_token_selected[8];
  char ui_status[27];
  bool ui_status_visible;
  char ui_clk_src[27];
  uint8_t step16[4];
  uint8_t step32[2];
  uint8_t step64;
  bool run;
  uint8_t prob[4];
  uint32_t prob_rng_state;
  bool prev_rst_active;
  uint32_t step_count;
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
  uint64_t next_int_tick_us;
  euclid_focus_t focus;
  euclid_param_t selected_param;
  bool grid_cache_valid;
  bool grid_cache_bits[64];
  bool grid_cache_progress[64];
  bool ui_cache_valid;
  bool ui_force_full_redraw;
  bool ui_header_dirty;
  bool ui_token_dirty[14];
  bool ui_status_dirty;
  bool ui_clk_src_dirty;
  char ui_header[27];
  char ui_token_text[14][16];
  bool ui_token_selected[14];
  char ui_status[27];
  bool ui_status_visible;
  char ui_clk_src[27];
  bool prev_rst_active;
  uint32_t step_count;
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
  uint32_t rng_state;
  bool level_10v;
  bool src_active[4];
  bool gate_out[4];
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
  uint32_t rng_state;
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
  bool ui_cache_valid;
  bool ui_force_full_redraw;
  bool ui_header_dirty;
  bool ui_token_dirty[9];
  bool ui_graph_dirty;
  bool ui_env_dirty;
  bool ui_status_dirty;
  char ui_header[27];
  char ui_token_text[9][20];
  bool ui_token_selected[9];
  char ui_env_text[27];
  char ui_status[27];
  bool ui_status_visible;
  uint8_t ui_graph_channel;
  uint8_t ui_graph_type;
  uint8_t ui_graph_attack;
  uint8_t ui_graph_decay;
  uint8_t ui_graph_sustain;
  uint8_t ui_graph_release;
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
  uint64_t next_step_at_us[4];
  uint32_t rng_state;
  bool ui_cache_valid;
  bool ui_force_full_redraw;
  bool ui_header_dirty;
  bool ui_token_dirty[6];
  bool ui_run_dirty;
  bool ui_in_dirty;
  bool ui_pattern_dirty;
  bool ui_status_dirty;
  uint32_t late_step_count;
  uint32_t missed_step_count;
  uint32_t catchup_limit_count;
  char ui_header[27];
  char ui_token_text[6][20];
  bool ui_token_selected[6];
  char ui_run_text[27];
  char ui_in_text[27];
  char ui_pattern_text[27];
  char ui_status[27];
  bool ui_status_visible;
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

typedef struct {
  uint8_t selected_channel;
  int32_t mv[4];
  bool outputs_ok;
} volts_state_t;

typedef struct {
  uint8_t selected_channel;
  int semi[4];
  bool outputs_ok;
} notes_state_t;

static const char* k_menu_items[10] = {
    "CALIBRATION", "NOTES", "VOLTS", "GRIDS", "TRIG SEQ", "4XEUCLID",
    "TR2GATE",     "TR2ADSR", "BURST GEN", "CV GEN"};

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
static uint32_t g_max_clock_event_latency_us = 0u;
static uint32_t g_oled_frame_time_max_us = 0u;

static uint64_t g_cal_status_until_ms = 0;
static char g_cal_status[16] = {0};

static grids_state_t g_grids = {
    .selected_param = GRIDS_PARAM_CLOCK,
    .clock = GRIDS_CLOCK_EXT,
    .bpm = 120,
    .map_x = 128,
    .map_y = 128,
    .chaos = 64,
    .prob = {100u, 100u, 100u, 100u},
    .next_cv_cache_us = 0u,
    .prob_rng_state = 0x4D595DF4u,
    .preview_cache_valid = false,
    .preview_cache_bits = {{false}},
    .preview_cache_progress = {{false}},
    .ui_cache_valid = false,
    .ui_force_full_redraw = true,
    .preview_force_full_redraw = true,
    .ui_header_dirty = true,
    .ui_token_dirty = {true, true, true, true, true, true, true, true, true},
    .ui_status_dirty = true,
    .ui_header = {0},
    .ui_token_text = {{0}},
    .ui_token_selected = {false},
    .ui_status = {0},
    .ui_status_visible = false,
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
    .next_int_tick_us = 0u,
    .focus = TRIGSEQ_FOCUS_MENU,
    .selected_param = TRIGSEQ_PARAM_LEN,
    .grid_cache_valid = false,
    .grid_cache_mode = TRIGSEQ_LEN_4X16,
    .grid_cache_cursor = 0u,
    .grid_cache_bits = {false},
    .grid_cache_progress = {false},
    .ui_cache_valid = false,
    .ui_force_full_redraw = true,
    .ui_header_dirty = true,
    .ui_token_dirty = {true, true, true, true, true, true, true, true},
    .ui_status_dirty = true,
    .ui_clk_src_dirty = true,
    .ui_header = {0},
    .ui_token_text = {{0}},
    .ui_token_selected = {false},
    .ui_status = {0},
    .ui_status_visible = false,
    .ui_clk_src = {0},
    .step16 = {0u, 0u, 0u, 0u},
    .step32 = {0u, 0u},
    .step64 = 0u,
    .run = true,
    .prob = {100u, 100u, 100u, 100u},
    .prob_rng_state = 0x2A1F6C3Du,
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
    .next_int_tick_us = 0u,
    .focus = EUCLID_FOCUS_MENU,
    .selected_param = EUCLID_PARAM_CLOCK,
    .grid_cache_valid = false,
    .grid_cache_bits = {false},
    .grid_cache_progress = {false},
    .ui_cache_valid = false,
    .ui_force_full_redraw = true,
    .ui_header_dirty = true,
    .ui_token_dirty = {true, true, true, true, true, true, true, true, true, true, true, true, true, true},
    .ui_status_dirty = true,
    .ui_clk_src_dirty = true,
    .ui_header = {0},
    .ui_token_text = {{0}},
    .ui_token_selected = {false},
    .ui_status = {0},
    .ui_status_visible = false,
    .ui_clk_src = {0},
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static tr2gate_state_t g_tr2gate = {
    .selected_param = TR2GATE_PARAM_CHANNEL,
    .mode = TR2GATE_MODE_DIRECT,
    .selected_channel = 0u,
    .source = {0u, 1u, 2u, 3u},
    .gate_time_cs = {50u, 50u, 50u, 50u},
    .prob = {100u, 100u, 100u, 100u},
    .rng_state = 0x6B8B4567u,
    .level_10v = true,
    .src_active = {false, false, false, false},
    .gate_out = {false, false, false, false},
    .rr_channel = 0u,
    .outputs_ok = false,
    .status_until_ms = 0u,
    .status = {0},
};

static tr2adsr_state_t g_tr2adsr = {
    .selected_param = TR2ADSR_PARAM_SOURCE,
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
    .rng_state = 0x42A7C91Du,
    .level = {0.0f, 0.0f, 0.0f, 0.0f},
    .release_start = {0.0f, 0.0f, 0.0f, 0.0f},
    .latched_sustain = {0.7f, 0.7f, 0.7f, 0.7f},
    .gate_hold_until_ms = {0u, 0u, 0u, 0u},
    .state_start_ms = {0u, 0u, 0u, 0u},
    .latched_total_ms = {1000u, 1000u, 1000u, 1000u},
    .ui_cache_valid = false,
    .ui_force_full_redraw = true,
    .ui_header_dirty = true,
    .ui_token_dirty = {true, true, true, true, true, true, true, true, true},
    .ui_graph_dirty = true,
    .ui_env_dirty = true,
    .ui_status_dirty = true,
    .ui_header = {0},
    .ui_token_text = {{0}},
    .ui_token_selected = {false, false, false, false, false, false, false, false, false},
    .ui_env_text = {0},
    .ui_status = {0},
    .ui_status_visible = false,
    .ui_graph_channel = 0xFFu,
    .ui_graph_type = 0xFFu,
    .ui_graph_attack = 0xFFu,
    .ui_graph_decay = 0xFFu,
    .ui_graph_sustain = 0xFFu,
    .ui_graph_release = 0xFFu,
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
    .next_step_at_us = {0u, 0u, 0u, 0u},
    .rng_state = 0x53A91C27u,
    .ui_cache_valid = false,
    .ui_force_full_redraw = true,
    .ui_header_dirty = true,
    .ui_token_dirty = {true, true, true, true, true, true},
    .ui_run_dirty = true,
    .ui_in_dirty = true,
    .ui_pattern_dirty = true,
    .ui_status_dirty = true,
    .late_step_count = 0u,
    .missed_step_count = 0u,
    .catchup_limit_count = 0u,
    .ui_header = {0},
    .ui_token_text = {{0}},
    .ui_token_selected = {false, false, false, false, false, false},
    .ui_run_text = {0},
    .ui_in_text = {0},
    .ui_pattern_text = {0},
    .ui_status = {0},
    .ui_status_visible = false,
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

static volts_state_t g_volts = {
    .selected_channel = 0u,
    .mv = {0, 0, 0, 0},
    .outputs_ok = false,
};

static notes_state_t g_notes = {
    .selected_channel = 0u,
    .semi = {36, 36, 36, 36},
    .outputs_ok = false,
};

static int32_t clamp_mv(int32_t mv) {
  return calibration_clamp_voltage_mv(&g_calibration_data, mv);
}

static int clamp_i(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int note_pitch_class(int semitone) {
  int pc = semitone % 12;
  if (pc < 0) pc += 12;
  return pc;
}

static int note_octave(int semitone) {
  int pc = semitone % 12;
  int oct = semitone / 12;
  if (pc < 0) oct -= 1;
  return oct;
}

static int32_t semitone_to_mv(int semitone) {
  return (int32_t)lroundf(((float)(semitone - 36) * 1000.0f) / 12.0f);
}

static int mv_to_nearest_semitone(int32_t mv) {
  return (int)lroundf(((float)mv * 12.0f) / 1000.0f) + 36;
}

static void note_label(int semitone, char* out, size_t out_sz) {
  static const char* const k_note_names[12] = {"C",  "CS", "D",  "DS", "E",  "F",
                                               "FS", "G",  "GS", "A",  "AS", "B"};
  snprintf(out, out_sz, "%s%d", k_note_names[note_pitch_class(semitone)], note_octave(semitone));
}

static int notes_min_semitone(void) {
  return mv_to_nearest_semitone(calibration_min_millivolts(&g_calibration_data));
}

static int notes_max_semitone(void) {
  return mv_to_nearest_semitone(calibration_max_millivolts(&g_calibration_data));
}

static void volts_clamp_state(void) {
  int32_t min_mv = calibration_min_millivolts(&g_calibration_data);
  int32_t max_mv = calibration_max_millivolts(&g_calibration_data);
  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    g_volts.mv[ch] = clamp_i(g_volts.mv[ch], min_mv, max_mv);
  }
}

static void notes_clamp_state(void) {
  int min_semi = notes_min_semitone();
  int max_semi = notes_max_semitone();
  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    g_notes.semi[ch] = clamp_i(g_notes.semi[ch], min_semi, max_semi);
  }
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

static uint8_t grids_effective_map_x(void) {
  return g_grids.map_x;
}

static uint8_t grids_effective_map_y(void) {
  return g_grids.map_y;
}

static uint8_t grids_effective_randomness(void) {
  return g_grids.chaos;
}

static void grids_mark_token_dirty(uint8_t idx) {
  if (idx < GRIDS_PARAM_COUNT) g_grids.ui_token_dirty[idx] = true;
}

static void grids_mark_all_tokens_dirty(void) {
  for (uint8_t i = 0u; i < GRIDS_PARAM_COUNT; ++i) {
    g_grids.ui_token_dirty[i] = true;
  }
}

static void grids_mark_full_redraw(void) {
  g_grids.ui_cache_valid = false;
  g_grids.preview_cache_valid = false;
  g_grids.ui_force_full_redraw = true;
  g_grids.preview_force_full_redraw = true;
  g_grids.ui_header_dirty = true;
  g_grids.ui_status_dirty = true;
  grids_mark_all_tokens_dirty();
}

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint8_t prob_rng8(uint32_t* state);
static bool prob_pass(uint8_t prob_percent, uint32_t* rng_state);
static void oled_draw_text_26(uint8_t y, const char* text, bool inverted);

static void trigseq_mark_token_dirty(uint8_t idx) {
  if (idx < TRIGSEQ_PARAM_COUNT) g_trigseq.ui_token_dirty[idx] = true;
}

static void trigseq_mark_all_tokens_dirty(void) {
  for (uint8_t i = 0u; i < TRIGSEQ_PARAM_COUNT; ++i) {
    g_trigseq.ui_token_dirty[i] = true;
  }
}

static void trigseq_mark_full_redraw(void) {
  g_trigseq.ui_cache_valid = false;
  g_trigseq.grid_cache_valid = false;
  g_trigseq.ui_force_full_redraw = true;
  g_trigseq.ui_header_dirty = true;
  g_trigseq.ui_status_dirty = true;
  g_trigseq.ui_clk_src_dirty = true;
  trigseq_mark_all_tokens_dirty();
}

static void euclid_mark_token_dirty(uint8_t idx) {
  if (idx < EUCLID_PARAM_COUNT) g_euclid.ui_token_dirty[idx] = true;
}

static void euclid_mark_all_tokens_dirty(void) {
  for (uint8_t i = 0u; i < EUCLID_PARAM_COUNT; ++i) {
    g_euclid.ui_token_dirty[i] = true;
  }
}

static void euclid_mark_full_redraw(void) {
  g_euclid.ui_cache_valid = false;
  g_euclid.grid_cache_valid = false;
  g_euclid.ui_force_full_redraw = true;
  g_euclid.ui_header_dirty = true;
  g_euclid.ui_status_dirty = true;
  g_euclid.ui_clk_src_dirty = true;
  euclid_mark_all_tokens_dirty();
}

static uint8_t tr2adsr_param_to_ui_index(tr2adsr_param_t param) {
  if (param < TR2ADSR_PARAM_SOURCE || param > TR2ADSR_PARAM_RELEASE) return 0xFFu;
  return (uint8_t)((int)param - (int)TR2ADSR_PARAM_SOURCE);
}

static void tr2adsr_mark_token_dirty(tr2adsr_param_t param) {
  uint8_t idx = tr2adsr_param_to_ui_index(param);
  if (idx < 9u) g_tr2adsr.ui_token_dirty[idx] = true;
}

static void tr2adsr_mark_all_tokens_dirty(void) {
  for (uint8_t i = 0u; i < 9u; ++i) {
    g_tr2adsr.ui_token_dirty[i] = true;
  }
}

static void tr2adsr_mark_full_redraw(void) {
  g_tr2adsr.ui_cache_valid = false;
  g_tr2adsr.ui_force_full_redraw = true;
  g_tr2adsr.ui_header_dirty = true;
  g_tr2adsr.ui_graph_dirty = true;
  g_tr2adsr.ui_env_dirty = true;
  g_tr2adsr.ui_status_dirty = true;
  tr2adsr_mark_all_tokens_dirty();
}

static void burstgen_mark_token_dirty(uint8_t idx) {
  if (idx < BURSTGEN_PARAM_COUNT) g_burstgen.ui_token_dirty[idx] = true;
}

static void burstgen_mark_all_tokens_dirty(void) {
  for (uint8_t i = 0u; i < BURSTGEN_PARAM_COUNT; ++i) {
    g_burstgen.ui_token_dirty[i] = true;
  }
}

static void burstgen_mark_full_redraw(void) {
  g_burstgen.ui_cache_valid = false;
  g_burstgen.ui_force_full_redraw = true;
  g_burstgen.ui_header_dirty = true;
  g_burstgen.ui_run_dirty = true;
  g_burstgen.ui_in_dirty = true;
  g_burstgen.ui_pattern_dirty = true;
  g_burstgen.ui_status_dirty = true;
  burstgen_mark_all_tokens_dirty();
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

static uint32_t trigseq_int_interval_us(void) {
  int bpm = clamp_i(g_trigseq.bpm, TRIGSEQ_BPM_MIN, TRIGSEQ_BPM_MAX);
  return (uint32_t)(60000000u / (uint32_t)bpm / 4u);
}

static void trigseq_set_clock_source(trigseq_clock_t source, uint64_t now_us) {
  g_trigseq.clock = source;
  trigseq_mark_token_dirty(TRIGSEQ_PARAM_CLOCK);
  trigseq_mark_token_dirty(TRIGSEQ_PARAM_BPM);
  g_trigseq.ui_clk_src_dirty = true;
  if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
    g_trigseq.next_int_tick_us = now_us + trigseq_int_interval_us();
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
  g_grids.ui_status_dirty = true;
}

static void set_trigseq_status(const char* s) {
  snprintf(g_trigseq.status, sizeof(g_trigseq.status), "%s", s);
  g_trigseq.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
  g_trigseq.ui_status_dirty = true;
}

static void set_euclid_status(const char* s) {
  snprintf(g_euclid.status, sizeof(g_euclid.status), "%s", s);
  g_euclid.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
  g_euclid.ui_status_dirty = true;
}

static void set_tr2gate_status(const char* s) {
  snprintf(g_tr2gate.status, sizeof(g_tr2gate.status), "%s", s);
  g_tr2gate.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
}

static void set_tr2adsr_status(const char* s) {
  snprintf(g_tr2adsr.status, sizeof(g_tr2adsr.status), "%s", s);
  g_tr2adsr.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
  g_tr2adsr.ui_status_dirty = true;
}

static void set_burstgen_status(const char* s) {
  snprintf(g_burstgen.status, sizeof(g_burstgen.status), "%s", s);
  g_burstgen.status_until_ms = to_ms_since_boot(get_absolute_time()) + 1500u;
  g_burstgen.ui_status_dirty = true;
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

static uint32_t burstgen_step_interval_us(uint8_t step_index, uint32_t base_step_us) {
  uint32_t swing = (uint32_t)clamp_i((int)g_burstgen.swing_pct, 0, (int)BURSTGEN_SWING_MAX);
  uint32_t swing_amount = (swing == 0u) ? 0u : (uint32_t)(burstgen_rng8() % (swing + 1u));
  uint32_t factor_pct = (step_index & 1u) == 0u ? (100u - swing_amount) : (100u + swing_amount);
  uint32_t interval = (base_step_us * factor_pct) / 100u;
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
static bool prob_pass(uint8_t prob_percent, uint32_t* rng_state);

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

static bool trigger_output_write_mv_cb(const int32_t millivolts_4[4], void* user_data) {
  (void)user_data;
  return app_write_outputs_mv(millivolts_4);
}

static void trigger_output_set_unipolar_levels(int32_t high_mv) {
  int32_t high[4] = {high_mv, high_mv, high_mv, high_mv};
  int32_t low[4] = {0, 0, 0, 0};
  trigger_output_set_levels_mv(high, low);
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

static void trigseq_reset_outputs_and_state(void) {
  trigger_output_set_unipolar_levels(TRIGSEQ_TRIG_HIGH_MV);
  g_trigseq.outputs_ok = trigger_engine_force_all_low();
}

static void trigseq_update_pulses(uint64_t now_us) {
  trigger_output_set_unipolar_levels(TRIGSEQ_TRIG_HIGH_MV);
  g_trigseq.outputs_ok = trigger_engine_update(now_us);
}

static void trigseq_on_clock_tick(uint64_t timestamp_us) {
  bool trig[4] = {false, false, false, false};
  uint8_t output_mask = 0u;

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
      output_mask |= (uint8_t)(1u << i);
    }
  }

  g_trigseq.step_count += 1u;
  if (output_mask != 0u) {
    trigger_output_set_unipolar_levels(TRIGSEQ_TRIG_HIGH_MV);
    g_trigseq.outputs_ok = trigger_engine_fire_mask(output_mask, timestamp_us, TRIGSEQ_TRIG_PULSE_US);
  }
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
  uint64_t now_us = time_us_64();
  if (!g_trigseq.engine_initialized) {
    trigseq_engine_init(&g_trigseq.engine, (uint32_t)now_us);
    g_trigseq.engine_initialized = true;
  }
  g_trigseq.prob_rng_state = (uint32_t)now_us ^ 0x5EED1234u;
  g_trigseq.run = true;
  g_trigseq.focus = TRIGSEQ_FOCUS_MENU;
  g_trigseq.selected_param = TRIGSEQ_PARAM_LEN;
  g_trigseq.cursor_step = 0u;
  trigseq_reset_engine();
  if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
    g_trigseq.next_int_tick_us = now_us + trigseq_int_interval_us();
  }
  g_trigseq.prev_rst_active = hal_io_trigger_active(HAL_IO_TR2);
  trigseq_reset_outputs_and_state();
  trigseq_mark_full_redraw();
}

static uint32_t euclid_int_interval_us(void) {
  int bpm = clamp_i(g_euclid.bpm, EUCLID_BPM_MIN, EUCLID_BPM_MAX);
  return (uint32_t)(60000000u / (uint32_t)bpm / 4u);
}

static void euclid_set_clock_source(euclid_clock_t source, uint64_t now_us) {
  g_euclid.clock = source;
  euclid_mark_token_dirty(EUCLID_PARAM_CLOCK);
  euclid_mark_token_dirty(EUCLID_PARAM_BPM);
  g_euclid.ui_clk_src_dirty = true;
  if (g_euclid.clock == EUCLID_CLOCK_INT) {
    g_euclid.next_int_tick_us = now_us + euclid_int_interval_us();
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

static void euclid_reset_outputs_and_state(void) {
  trigger_output_set_unipolar_levels(EUCLID_TRIG_HIGH_MV);
  g_euclid.outputs_ok = trigger_engine_force_all_low();
}

static void euclid_update_pulses(uint64_t now_us) {
  trigger_output_set_unipolar_levels(EUCLID_TRIG_HIGH_MV);
  g_euclid.outputs_ok = trigger_engine_update(now_us);
}

static void euclid_on_clock_tick(uint64_t timestamp_us) {
  uint8_t output_mask = 0u;
  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    uint8_t step = g_euclid.phase[ch];
    if (euclid_step_is_hit(g_euclid.steps[ch], g_euclid.hits[ch], step) &&
        prob_pass(g_euclid.prob[ch], &g_euclid.prob_rng_state)) {
      output_mask |= (uint8_t)(1u << ch);
    }
    if (g_euclid.steps[ch] > 0u) {
      g_euclid.phase[ch] = (uint8_t)((step + 1u) % g_euclid.steps[ch]);
    }
  }
  g_euclid.step_count += 1u;
  if (output_mask != 0u) {
    trigger_output_set_unipolar_levels(EUCLID_TRIG_HIGH_MV);
    g_euclid.outputs_ok = trigger_engine_fire_mask(output_mask, timestamp_us, EUCLID_TRIG_PULSE_US);
  }
}

static void euclid_reset_engine(void) {
  g_euclid.phase[0] = 0u;
  g_euclid.phase[1] = 0u;
  g_euclid.phase[2] = 0u;
  g_euclid.phase[3] = 0u;
  g_euclid.step_count = 0u;
}

static void euclid_enter(void) {
  uint64_t now_us = time_us_64();
  g_euclid.focus = EUCLID_FOCUS_MENU;
  g_euclid.selected_param = EUCLID_PARAM_CLOCK;
  g_euclid.prob_rng_state = (uint32_t)now_us ^ 0x1E7A4D99u;
  euclid_reset_engine();
  if (g_euclid.clock == EUCLID_CLOCK_INT) {
    g_euclid.next_int_tick_us = now_us + euclid_int_interval_us();
  }
  g_euclid.prev_rst_active = hal_io_trigger_active(HAL_IO_TR2);
  euclid_reset_outputs_and_state();
  euclid_mark_full_redraw();
}

static void tr2gate_sync_gate_out_from_engine(void) {
  uint8_t active_mask = trigger_engine_active_mask();
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_tr2gate.gate_out[i] = (active_mask & (uint8_t)(1u << i)) != 0u;
  }
}

static void tr2gate_reset_outputs_and_state(void) {
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_tr2gate.src_active[i] = read_trigger_active_src(i);
    g_tr2gate.gate_out[i] = false;
  }
  g_tr2gate.rr_channel = 0u;
  trigger_output_set_unipolar_levels(g_tr2gate.level_10v ? 10000 : 5000);
  g_tr2gate.outputs_ok = trigger_engine_force_all_low();
}

static void tr2gate_enter(void) {
  g_tr2gate.rng_state ^= (uint32_t)time_us_64() ^ 0x2BADB002u;
  tr2gate_reset_outputs_and_state();
}

static void tr2gate_on_source_edge_us(uint8_t src, uint64_t timestamp_us) {
  uint8_t fire_mask = 0u;
  uint32_t gate_width_us[4] = {0u, 0u, 0u, 0u};
  uint64_t gate_timestamp_us[4] = {0u, 0u, 0u, 0u};

  if (src >= 4u) return;

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    if (g_tr2gate.source[ch] == src && prob_pass(g_tr2gate.prob[ch], &g_tr2gate.rng_state)) {
      fire_mask |= (uint8_t)(1u << ch);
      gate_width_us[ch] = (uint32_t)g_tr2gate.gate_time_cs[ch] * 10000u;
      gate_timestamp_us[ch] = timestamp_us;
    }
  }

  if (fire_mask != 0u) {
    trigger_output_set_unipolar_levels(g_tr2gate.level_10v ? 10000 : 5000);
    g_tr2gate.outputs_ok = trigger_engine_fire_mask_events(fire_mask, gate_timestamp_us, gate_width_us);
    tr2gate_sync_gate_out_from_engine();
  }
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

static uint8_t tr2adsr_current_channel(void) {
  if (g_preset_ui.screen < 4u) return g_preset_ui.screen;
  return (uint8_t)(g_tr2adsr.selected_channel & 0x03u);
}

static tr2adsr_param_t tr2adsr_next_menu_param(uint8_t ch, tr2adsr_param_t current, int32_t delta) {
  int next = (int)current;
  do {
    next += (delta > 0) ? 1 : -1;
    while (next < 0) next += (int)TR2ADSR_PARAM_COUNT;
    while (next >= (int)TR2ADSR_PARAM_COUNT) next -= (int)TR2ADSR_PARAM_COUNT;
  } while ((tr2adsr_param_t)next == TR2ADSR_PARAM_CHANNEL ||
           !tr2adsr_param_visible(ch, (tr2adsr_param_t)next));
  return (tr2adsr_param_t)next;
}

static void tr2adsr_sync_screen_channel(void) {
  g_tr2adsr.selected_channel = tr2adsr_current_channel();
  if (g_tr2adsr.selected_param == TR2ADSR_PARAM_CHANNEL) {
    g_tr2adsr.selected_param = TR2ADSR_PARAM_SOURCE;
  }
  if (!tr2adsr_param_visible(g_tr2adsr.selected_channel, g_tr2adsr.selected_param)) {
    g_tr2adsr.selected_param =
        tr2adsr_next_menu_param(g_tr2adsr.selected_channel, g_tr2adsr.selected_param, 1);
  }
}

static void tr2adsr_mark_channel_changed(void) {
  g_tr2adsr.ui_header_dirty = true;
  g_tr2adsr.ui_graph_dirty = true;
  g_tr2adsr.ui_env_dirty = true;
  g_tr2adsr.ui_status_dirty = true;
  tr2adsr_mark_all_tokens_dirty();
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

static void tr2adsr_on_trigger_us(uint8_t ch, uint64_t timestamp_us) {
  uint64_t now_ms = timestamp_us / 1000u;
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
  g_tr2adsr.rng_state ^= (uint32_t)time_us_64() ^ 0x3141592Bu;
  tr2adsr_reset_outputs_and_state();
}

static void tr2adsr_on_source_edge_us(uint8_t src, uint64_t timestamp_us) {
  if (src >= 4u) return;

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    if (g_tr2adsr.source[ch] == src && prob_pass(g_tr2adsr.prob[ch], &g_tr2adsr.rng_state)) {
      tr2adsr_on_trigger_us(ch, timestamp_us);
    }
  }
}

static void burstgen_reset_outputs_and_state(void) {
  uint64_t now_us = time_us_64();
  g_burstgen.rng_state = (uint32_t)now_us ^ 0x53A91C27u;
  g_burstgen.late_step_count = 0u;
  g_burstgen.missed_step_count = 0u;
  g_burstgen.catchup_limit_count = 0u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_burstgen.src_active[i] = read_trigger_active_src(i);
    g_burstgen.running[i] = false;
    g_burstgen.gate_out[i] = false;
    g_burstgen.pattern_mask[i] = 0u;
    g_burstgen.current_step[i] = 0u;
    g_burstgen.next_step_at_us[i] = now_us;
  }
  trigger_output_set_unipolar_levels(g_burstgen.level_10v ? 10000 : 5000);
  g_burstgen.outputs_ok = trigger_engine_force_all_low();
}

static void burstgen_start_channel(uint8_t ch, uint64_t now_us) {
  uint8_t steps = burstgen_steps_for_mode(g_burstgen.signature_mode);
  uint8_t max_hits = burstgen_max_hits_for_mode(g_burstgen.signature_mode);
  g_burstgen.pattern_mask[ch] = burstgen_pick_pattern_mask(steps, max_hits);
  g_burstgen.current_step[ch] = 0u;
  g_burstgen.next_step_at_us[ch] = now_us;
  g_burstgen.running[ch] = true;
}

static void burstgen_sync_gate_out_from_engine(void) {
  uint8_t active_mask = trigger_engine_active_mask();
  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    g_burstgen.gate_out[ch] = (active_mask & (uint8_t)(1u << ch)) != 0u;
  }
}

static void burstgen_update_all(uint64_t now_us) {
  uint8_t steps = burstgen_steps_for_mode(g_burstgen.signature_mode);
  uint32_t quarter_us;
  uint32_t base_step_us;
  uint8_t fire_mask = 0u;

  trigger_output_set_unipolar_levels(g_burstgen.level_10v ? 10000 : 5000);
  g_burstgen.outputs_ok = trigger_engine_update(now_us);

  quarter_us = (uint32_t)(60000000u / (uint32_t)clamp_i((int)g_burstgen.bpm, BURSTGEN_BPM_MIN, BURSTGEN_BPM_MAX));
  base_step_us = (quarter_us * 4u) / steps;
  if (base_step_us == 0u) base_step_us = 1u;

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    uint8_t catchup_steps = 0u;

    while (g_burstgen.running[ch] && now_us >= g_burstgen.next_step_at_us[ch] &&
           catchup_steps < BURSTGEN_MAX_CATCHUP_STEPS) {
      uint8_t ch_bit = (uint8_t)(1u << ch);
      if (now_us > (g_burstgen.next_step_at_us[ch] + 1000u)) {
        g_burstgen.late_step_count += 1u;
      }

      if (burstgen_pattern_hit(g_burstgen.pattern_mask[ch], steps, g_burstgen.current_step[ch]) &&
          prob_pass(g_burstgen.probability, &g_burstgen.rng_state)) {
        if ((fire_mask & ch_bit) != 0u) {
          g_burstgen.missed_step_count += 1u;
        }
        fire_mask |= ch_bit;
      }

      {
        uint32_t interval_us = burstgen_step_interval_us(g_burstgen.current_step[ch], base_step_us);
        g_burstgen.current_step[ch] = (uint8_t)(g_burstgen.current_step[ch] + 1u);
        if (g_burstgen.current_step[ch] >= steps) {
          g_burstgen.running[ch] = false;
          g_burstgen.current_step[ch] = 0u;
        } else {
          g_burstgen.next_step_at_us[ch] += interval_us;
        }
      }
      catchup_steps = (uint8_t)(catchup_steps + 1u);
    }

    if (g_burstgen.running[ch] && now_us >= g_burstgen.next_step_at_us[ch]) {
      g_burstgen.catchup_limit_count += 1u;
      g_burstgen.missed_step_count += 1u;
    }
  }

  if (fire_mask != 0u) {
    g_burstgen.outputs_ok = trigger_engine_fire_mask(fire_mask, now_us, BURSTGEN_PULSE_US);
  }
  burstgen_sync_gate_out_from_engine();
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
  euclid_mark_full_redraw();

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
  trigger_output_set_unipolar_levels(GRIDS_TRIG_HIGH_MV);
  g_grids.outputs_ok = trigger_engine_force_all_low();
}

static void grids_update_cv_cache(uint64_t now_us) {
  if (now_us < g_grids.next_cv_cache_us) return;
  g_grids.next_cv_cache_us = now_us + CV_CACHE_PERIOD_US;
  grids_sample_fill_from_cv();
}

static void grids_on_clock_tick(uint64_t timestamp_us) {
  bool trig[4] = {false, false, false, false};
  uint8_t map_x;
  uint8_t map_y;
  uint8_t randomness;
  uint8_t output_mask = 0u;

  map_x = grids_effective_map_x();
  map_y = grids_effective_map_y();
  randomness = grids_effective_randomness();
  grids_engine_step(&g_grids.engine, map_x, map_y, randomness, g_grids.fill_u8, trig);

  for (uint8_t i = 0u; i < 4u; ++i) {
    if (trig[i] && prob_pass(g_grids.prob[i], &g_grids.prob_rng_state)) {
      output_mask |= (uint8_t)(1u << i);
    }
  }

  g_grids.step_count += 1u;
  if (output_mask != 0u) {
    trigger_output_set_unipolar_levels(GRIDS_TRIG_HIGH_MV);
    g_grids.outputs_ok = trigger_engine_fire_mask(output_mask, timestamp_us, GRIDS_TRIG_PULSE_US);
  }
}

static void grids_update_pulses(uint64_t now_us) {
  trigger_output_set_unipolar_levels(GRIDS_TRIG_HIGH_MV);
  g_grids.outputs_ok = trigger_engine_update(now_us);
}

static uint32_t grids_int_interval_us(void) {
  int bpm = clamp_i(g_grids.bpm, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
  return (uint32_t)(60000000u / (uint32_t)bpm / 4u);
}

static void grids_set_clock_source(grids_clock_t source, uint64_t now_us) {
  g_grids.clock = source;
  grids_mark_token_dirty(GRIDS_PARAM_CLOCK);
  if (g_grids.clock == GRIDS_CLOCK_INT) {
    g_grids.next_int_tick_us = now_us + grids_int_interval_us();
  }
}

static void grids_reset_engine(void) {
  grids_engine_reset(&g_grids.engine);
  g_grids.step_count = 0u;
}

static void grids_enter(void) {
  uint64_t now_us = time_us_64();
  grids_mark_full_redraw();
  grids_engine_init(&g_grids.engine, (uint32_t)now_us);
  g_grids.prob_rng_state = (uint32_t)now_us ^ 0xA5A55A5Au;
  grids_reset_engine();
  grids_sample_fill_from_cv();
  g_grids.next_cv_cache_us = now_us + CV_CACHE_PERIOD_US;
  g_grids.prev_rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (g_grids.clock == GRIDS_CLOCK_INT) {
    g_grids.next_int_tick_us = now_us + grids_int_interval_us();
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
  } else if (mode == APP_CALIBRATION) {
    (void)apply_calibration_preview();
    set_cal_status("CAL MODE");
  } else if (mode == APP_NOTES) {
    int32_t mv[4];
    notes_clamp_state();
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
      mv[ch] = semitone_to_mv(g_notes.semi[ch]);
    }
    g_notes.outputs_ok = app_write_outputs_mv(mv);
  } else if (mode == APP_VOLTS) {
    volts_clamp_state();
    g_volts.outputs_ok = app_write_outputs_mv(g_volts.mv);
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
  if (mode == APP_NOTES) return "NOTES";
  if (mode == APP_VOLTS) return "VOLTS";
  if (mode == APP_CALIBRATION) return "CALIBRATION";
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
    grids_mark_full_redraw();
  } else if (g_app_mode == APP_TRIGSEQ) {
    trigseq_mark_full_redraw();
  } else if (g_app_mode == APP_EUCLID) {
    euclid_mark_full_redraw();
  } else if (g_app_mode == APP_TR2ADSR) {
    tr2adsr_mark_full_redraw();
  } else if (g_app_mode == APP_BURSTGEN) {
    burstgen_mark_full_redraw();
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
  if (g_app_mode == APP_TR2ADSR) return 6u;
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
    if (g_app_mode == APP_TRIGSEQ && g_preset_ui.screen == APP_SCREEN_MAIN &&
        g_trigseq.selected_param == TRIGSEQ_PARAM_EDIT_GRID) {
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

  if (g_app_mode == APP_TRIGSEQ || g_app_mode == APP_EUCLID || g_app_mode == APP_TR2ADSR ||
      g_app_mode == APP_CVGEN) {
    const uint8_t item_y0 = 20u;
    const uint8_t item_pitch = 10u;

    snprintf(line, sizeof(line), "%s %u/%u %s", app_display_name(g_app_mode),
             (unsigned)(g_preset_ui.screen + 1u), (unsigned)active_app_screen_count(),
             is_load ? "LOAD" : "SAVE");
    hal_io_oled_draw_line(0, line, false);
    for (uint8_t i = 0u; i < APP_PRESET_SLOTS; ++i) {
      snprintf(line, sizeof(line), "%c P%02u %s", i == g_preset_ui.slot_sel ? '>' : ' ',
               (unsigned)(i + 1u), active_app_preset_slot_used(i) ? "USED" : "EMPTY");
      oled_draw_text_26((uint8_t)(item_y0 + i * item_pitch), line, i == g_preset_ui.slot_sel);
    }
    oled_draw_text_26(104u, "", false);
    oled_draw_text_26(112u, status, false);
    oled_draw_text_26(120u, "ENC_R EXEC", false);
    return;
  }

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
      app_enter(APP_CALIBRATION);
    } else if (g_menu_index == 1) {
      app_enter(APP_NOTES);
    } else if (g_menu_index == 2) {
      app_enter(APP_VOLTS);
    } else if (g_menu_index == 3) {
      app_enter(APP_GRIDS);
    } else if (g_menu_index == 4) {
      app_enter(APP_TRIGSEQ);
    } else if (g_menu_index == 5) {
      app_enter(APP_EUCLID);
    } else if (g_menu_index == 6) {
      app_enter(APP_TR2GATE);
    } else if (g_menu_index == 7) {
      app_enter(APP_TR2ADSR);
    } else if (g_menu_index == 8) {
      app_enter(APP_BURSTGEN);
    } else {
      app_enter(APP_CVGEN);
    }
  }
}

static void draw_notes(void) {
  char line[32];
  char min_note[8];
  char max_note[8];
  char note_buf[8];

  note_label(notes_min_semitone(), min_note, sizeof(min_note));
  note_label(notes_max_semitone(), max_note, sizeof(max_note));

  snprintf(line, sizeof(line), "NOTES %s..%s", min_note, max_note);
  hal_io_oled_draw_line(0, line, true);

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    int32_t mv = semitone_to_mv(g_notes.semi[ch]);
    int32_t abs_mv = (mv < 0) ? -mv : mv;
    note_label(g_notes.semi[ch], note_buf, sizeof(note_buf));
    snprintf(line, sizeof(line), "%c CH%c %-4s %s%ld.%03ldV",
             ch == g_notes.selected_channel ? '>' : ' ', (char)('A' + ch), note_buf,
             mv < 0 ? "-" : "", (long)(abs_mv / 1000), (long)(abs_mv % 1000));
    hal_io_oled_draw_line((uint8_t)(2u + ch), line, ch == g_notes.selected_channel);
  }

  hal_io_oled_draw_line(8, "ENC_L CH", false);
  hal_io_oled_draw_line(9, "ENC_R NOTE", false);
  hal_io_oled_draw_line(10, "ENC_L_SW BACK", false);
  clear_rows(11);
}

static void draw_volts(void) {
  char line[32];
  int32_t min_mv = calibration_min_millivolts(&g_calibration_data);
  int32_t max_mv = calibration_max_millivolts(&g_calibration_data);
  int32_t abs_min = (min_mv < 0) ? -min_mv : min_mv;
  int32_t abs_max = (max_mv < 0) ? -max_mv : max_mv;

  snprintf(line, sizeof(line), "VOLTS %s%ld.%01ld..%s%ld.%01ldV", min_mv < 0 ? "-" : "",
           (long)(abs_min / 1000), (long)((abs_min % 1000) / 100), max_mv < 0 ? "-" : "",
           (long)(abs_max / 1000), (long)((abs_max % 1000) / 100));
  hal_io_oled_draw_line(0, line, true);

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    int32_t mv = g_volts.mv[ch];
    int32_t abs_mv = (mv < 0) ? -mv : mv;
    snprintf(line, sizeof(line), "%c CH%c %s%ld.%01ldV", ch == g_volts.selected_channel ? '>' : ' ',
             (char)('A' + ch), mv < 0 ? "-" : "", (long)(abs_mv / 1000),
             (long)((abs_mv % 1000) / 100));
    hal_io_oled_draw_line((uint8_t)(2u + ch), line, ch == g_volts.selected_channel);
  }

  hal_io_oled_draw_line(8, "ENC_L CH", false);
  hal_io_oled_draw_line(9, "ENC_R 0.1V", false);
  hal_io_oled_draw_line(10, "ENC_L_SW BACK", false);
  clear_rows(11);
}

static void update_notes(int32_t d_l, int32_t d_r) {
  int32_t mv[4];

  if (d_l != 0) {
    g_notes.selected_channel = wrap_u8_delta(g_notes.selected_channel, d_l, 4u);
  }

  if (d_r != 0) {
    int ch = (int)g_notes.selected_channel;
    g_notes.semi[ch] = clamp_i(g_notes.semi[ch] + (int)d_r, notes_min_semitone(), notes_max_semitone());
  }

  notes_clamp_state();
  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    mv[ch] = semitone_to_mv(g_notes.semi[ch]);
  }
  g_notes.outputs_ok = app_write_outputs_mv(mv);
}

static void update_volts(int32_t d_l, int32_t d_r) {
  if (d_l != 0) {
    g_volts.selected_channel = wrap_u8_delta(g_volts.selected_channel, d_l, 4u);
  }

  if (d_r != 0) {
    int ch = (int)g_volts.selected_channel;
    g_volts.mv[ch] =
        clamp_i(g_volts.mv[ch] + ((int)d_r * 100), calibration_min_millivolts(&g_calibration_data),
                calibration_max_millivolts(&g_calibration_data));
  }

  volts_clamp_state();
  g_volts.outputs_ok = app_write_outputs_mv(g_volts.mv);
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
    (void)apply_calibration_preview();
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
  uint64_t now_us = time_us_64();
  // TR1-TR4 rising edges are delivered through the GPIO IRQ queues. Active
  // levels are still polled for the OLED input monitor only.
  g_tr2gate.mode = TR2GATE_MODE_DIRECT;
  if (g_tr2gate.selected_param == TR2GATE_PARAM_MODE) {
    g_tr2gate.selected_param = TR2GATE_PARAM_CHANNEL;
  }
  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
  }

  if (d_l != 0) {
    int next = (int)g_tr2gate.selected_param + (int)d_l;
    while (next < (int)TR2GATE_PARAM_CHANNEL) next += (int)(TR2GATE_PARAM_COUNT - 1);
    while (next >= (int)TR2GATE_PARAM_COUNT) next -= (int)(TR2GATE_PARAM_COUNT - 1);
    g_tr2gate.selected_param = (tr2gate_param_t)next;
  }

  if (d_r != 0) {
    uint8_t ch = g_tr2gate.selected_channel;
    if (g_tr2gate.selected_param == TR2GATE_PARAM_CHANNEL) {
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
      trigger_output_set_unipolar_levels(g_tr2gate.level_10v ? 10000 : 5000);
    }
  }

  trigger_output_set_unipolar_levels(g_tr2gate.level_10v ? 10000 : 5000);
  g_tr2gate.outputs_ok = trigger_engine_update(now_us);
  tr2gate_sync_gate_out_from_engine();

  if (edge_enc_r) {
    save_tr2gate_settings();
  }
}

static void update_tr2adsr(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                           uint64_t now_ms) {
  uint8_t prev_channel = g_tr2adsr.selected_channel;
  tr2adsr_param_t prev_param = g_tr2adsr.selected_param;
  // TR1-TR4 rising edges are delivered through the GPIO IRQ queues. Gate level
  // is still polled because ASR/T&H-style behavior needs to know when input is high.
  tr2adsr_refresh_cv_inputs();
  tr2adsr_sync_screen_channel();
  if (handle_active_app_preset_ui(d_l, d_r, edge_enc_r, edge_sw1, edge_sw2, now_ms)) {
    d_l = 0;
    d_r = 0;
    edge_enc_r = false;
  }
  tr2adsr_sync_screen_channel();
  if (g_tr2adsr.selected_channel != prev_channel) {
    tr2adsr_mark_channel_changed();
    prev_channel = g_tr2adsr.selected_channel;
    prev_param = g_tr2adsr.selected_param;
  }

  if (d_l != 0) {
    g_tr2adsr.selected_param =
        tr2adsr_next_menu_param(g_tr2adsr.selected_channel, g_tr2adsr.selected_param, d_l);
    if (g_tr2adsr.selected_param != prev_param) {
      tr2adsr_mark_token_dirty(prev_param);
      tr2adsr_mark_token_dirty(g_tr2adsr.selected_param);
      prev_param = g_tr2adsr.selected_param;
    }
  }

  if (d_r != 0) {
    uint8_t ch = g_tr2adsr.selected_channel;
    if (g_tr2adsr.selected_param == TR2ADSR_PARAM_SOURCE) {
      g_tr2adsr.source[ch] = wrap_u8_delta(g_tr2adsr.source[ch], d_r, 4u);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_SOURCE);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_TYPE) {
      g_tr2adsr.type[ch] = (tr2adsr_env_type_t)wrap_u8_delta((uint8_t)g_tr2adsr.type[ch], d_r, 3u);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_TYPE);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_DECAY);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_SUSTAIN);
      g_tr2adsr.ui_graph_dirty = true;
      if (!tr2adsr_param_visible(ch, g_tr2adsr.selected_param)) {
        g_tr2adsr.selected_param = tr2adsr_next_menu_param(ch, g_tr2adsr.selected_param, 1);
        tr2adsr_mark_token_dirty(prev_param);
        tr2adsr_mark_token_dirty(g_tr2adsr.selected_param);
        prev_param = g_tr2adsr.selected_param;
      }
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_TIME) {
      int next = (int)g_tr2adsr.total_time_ds[ch] + ((d_r > 0) ? 1 : -1);
      g_tr2adsr.total_time_ds[ch] =
          (uint16_t)clamp_i(next, TR2ADSR_TIME_MIN_DS, TR2ADSR_TIME_MAX_DS);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_TIME);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_PROB) {
      g_tr2adsr.prob[ch] = clamp_u8i_100((int)g_tr2adsr.prob[ch] + (int)d_r);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_PROB);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_LEVEL) {
      g_tr2adsr.level_10v = !g_tr2adsr.level_10v;
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_LEVEL);
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_ATTACK) {
      g_tr2adsr.attack_pct[ch] = (uint8_t)clamp_i((int)g_tr2adsr.attack_pct[ch] + (int)d_r, 1, 99);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_ATTACK);
      g_tr2adsr.ui_graph_dirty = true;
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_DECAY) {
      g_tr2adsr.decay_pct[ch] = (uint8_t)clamp_i((int)g_tr2adsr.decay_pct[ch] + (int)d_r, 1, 99);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_DECAY);
      g_tr2adsr.ui_graph_dirty = true;
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_SUSTAIN) {
      g_tr2adsr.sustain_pct[ch] = clamp_u8i_100((int)g_tr2adsr.sustain_pct[ch] + (int)d_r);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_SUSTAIN);
      g_tr2adsr.ui_graph_dirty = true;
    } else if (g_tr2adsr.selected_param == TR2ADSR_PARAM_RELEASE) {
      g_tr2adsr.release_pct[ch] =
          (uint8_t)clamp_i((int)g_tr2adsr.release_pct[ch] + (int)d_r, 1, 99);
      tr2adsr_mark_token_dirty(TR2ADSR_PARAM_RELEASE);
      g_tr2adsr.ui_graph_dirty = true;
    }
  }

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    uint8_t src = g_tr2adsr.source[ch];
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
  uint64_t now_us = time_us_64();
  burstgen_param_t prev_selected = g_burstgen.selected_param;
  // TR1-TR4 burst starts are timestamped by GPIO IRQ queues.

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
    burstgen_mark_token_dirty((uint8_t)prev_selected);
    burstgen_mark_token_dirty((uint8_t)g_burstgen.selected_param);
  }

  if (d_r != 0) {
    if (g_burstgen.selected_param == BURSTGEN_PARAM_CHANNEL) {
      g_burstgen.selected_channel = wrap_u8_delta(g_burstgen.selected_channel, d_r, 4u);
      burstgen_mark_token_dirty(BURSTGEN_PARAM_CHANNEL);
      g_burstgen.ui_pattern_dirty = true;
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_SIGNATURE) {
      int next = (int)g_burstgen.signature_mode + ((d_r > 0) ? 1 : -1);
      while (next < 0) next += 4;
      while (next >= 4) next -= 4;
      g_burstgen.signature_mode = (uint8_t)next;
      burstgen_mark_token_dirty(BURSTGEN_PARAM_SIGNATURE);
      g_burstgen.ui_pattern_dirty = true;
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_BPM) {
      g_burstgen.bpm = (uint16_t)clamp_i((int)g_burstgen.bpm + (int)d_r, BURSTGEN_BPM_MIN, BURSTGEN_BPM_MAX);
      burstgen_mark_token_dirty(BURSTGEN_PARAM_BPM);
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_SWING) {
      g_burstgen.swing_pct =
          (uint8_t)clamp_i((int)g_burstgen.swing_pct + (int)d_r, 0, (int)BURSTGEN_SWING_MAX);
      burstgen_mark_token_dirty(BURSTGEN_PARAM_SWING);
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_PROB) {
      g_burstgen.probability = clamp_u8i_100((int)g_burstgen.probability + (int)d_r);
      burstgen_mark_token_dirty(BURSTGEN_PARAM_PROB);
    } else if (g_burstgen.selected_param == BURSTGEN_PARAM_LEVEL) {
      g_burstgen.level_10v = !g_burstgen.level_10v;
      trigger_output_set_unipolar_levels(g_burstgen.level_10v ? 10000 : 5000);
      burstgen_mark_token_dirty(BURSTGEN_PARAM_LEVEL);
    }
  }

  if (edge_sw1) {
    burstgen_start_channel(g_burstgen.selected_channel, now_us);
  }
  if (edge_sw2) {
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
      burstgen_start_channel(ch, now_us);
    }
  }

  burstgen_update_all(now_us);

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
  bool rst_active;
  uint64_t now_us = time_us_64();
  grids_param_t prev_selected = g_grids.selected_param;

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
    grids_mark_token_dirty((uint8_t)prev_selected);
    grids_mark_token_dirty((uint8_t)g_grids.selected_param);
  }

  if (d_r != 0) {
    if (g_grids.selected_param == GRIDS_PARAM_CLOCK) {
      if (d_r > 0) {
        grids_set_clock_source(GRIDS_CLOCK_EXT, now_us);
      } else {
        grids_set_clock_source(GRIDS_CLOCK_INT, now_us);
      }
    } else if (g_grids.selected_param == GRIDS_PARAM_BPM) {
      if (g_grids.clock == GRIDS_CLOCK_INT) {
        g_grids.bpm = clamp_i(g_grids.bpm + (int)d_r, GRIDS_BPM_MIN, GRIDS_BPM_MAX);
        g_grids.next_int_tick_us = now_us + grids_int_interval_us();
        grids_mark_token_dirty(GRIDS_PARAM_BPM);
      }
    } else if (g_grids.selected_param == GRIDS_PARAM_MAP_X) {
      g_grids.map_x = clamp_u8i((int)g_grids.map_x + (int)d_r);
      grids_mark_token_dirty(GRIDS_PARAM_MAP_X);
    } else if (g_grids.selected_param == GRIDS_PARAM_MAP_Y) {
      g_grids.map_y = clamp_u8i((int)g_grids.map_y + (int)d_r);
      grids_mark_token_dirty(GRIDS_PARAM_MAP_Y);
    } else if (g_grids.selected_param == GRIDS_PARAM_CHAOS) {
      g_grids.chaos = clamp_u8i((int)g_grids.chaos + (int)d_r);
      grids_mark_token_dirty(GRIDS_PARAM_CHAOS);
    } else {
      uint8_t idx = (uint8_t)((int)g_grids.selected_param - (int)GRIDS_PARAM_PROB1);
      if (idx < 4u) {
        g_grids.prob[idx] = clamp_u8i_100((int)g_grids.prob[idx] + (int)d_r);
        grids_mark_token_dirty((uint8_t)g_grids.selected_param);
      }
    }
  }

  grids_update_cv_cache(now_us);

  rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (rst_active && !g_grids.prev_rst_active) {
    grids_reset_engine();
  }
  g_grids.prev_rst_active = rst_active;

  if (g_grids.clock == GRIDS_CLOCK_INT) {
    uint32_t interval = grids_int_interval_us();
    uint8_t guard = 0;
    while (now_us >= g_grids.next_int_tick_us && guard < 8u) {
      grids_on_clock_tick(g_grids.next_int_tick_us);
      g_grids.next_int_tick_us += interval;
      ++guard;
    }
  }

  grids_update_pulses(now_us);

  if (edge_enc_r) {
    save_grids_settings();
  }
}

static void update_trigseq(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                           uint64_t now_ms) {
  bool rst_active;
  uint64_t now_us = time_us_64();
  trigseq_param_t prev_selected = g_trigseq.selected_param;
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
      col += (int)d_l;
      while (col < 0) col += 16;
      while (col >= 16) col -= 16;
      g_trigseq.cursor_step = (uint8_t)(row * 16 + col);
    }

    if (d_r != 0) {
      int row = (int)(g_trigseq.cursor_step / 16u);
      int col = (int)(g_trigseq.cursor_step % 16u);
      row += (int)d_r;
      while (row < 0) row += 4;
      while (row >= 4) row -= 4;
      g_trigseq.cursor_step = (uint8_t)(row * 16 + col);
    }

    if (edge_enc_r || edge_sw1 || edge_sw2) {
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
      trigseq_mark_token_dirty((uint8_t)prev_selected);
      trigseq_mark_token_dirty((uint8_t)g_trigseq.selected_param);
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
        trigseq_mark_token_dirty(TRIGSEQ_PARAM_LEN);
      } else if (g_trigseq.selected_param == TRIGSEQ_PARAM_CLOCK) {
        if (d_r > 0) {
          trigseq_set_clock_source(TRIGSEQ_CLOCK_EXT, now_us);
        } else {
          trigseq_set_clock_source(TRIGSEQ_CLOCK_INT, now_us);
        }
      } else if (g_trigseq.selected_param == TRIGSEQ_PARAM_BPM) {
        if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
          g_trigseq.bpm = clamp_i(g_trigseq.bpm + (int)d_r, TRIGSEQ_BPM_MIN, TRIGSEQ_BPM_MAX);
          g_trigseq.next_int_tick_us = now_us + trigseq_int_interval_us();
          trigseq_mark_token_dirty(TRIGSEQ_PARAM_BPM);
        }
      } else {
        uint8_t idx = (uint8_t)((int)g_trigseq.selected_param - (int)TRIGSEQ_PARAM_PROB1);
        if (idx < 4u) {
          g_trigseq.prob[idx] = clamp_u8i_100((int)g_trigseq.prob[idx] + (int)d_r);
          trigseq_mark_token_dirty((uint8_t)g_trigseq.selected_param);
        }
      }
    }

    if (g_trigseq.selected_param == TRIGSEQ_PARAM_EDIT_GRID && edge_enc_r) {
      preset_ui_set_screen(APP_SCREEN_TRIGSEQ_GRID);
      edge_enc_r = false;
    }
  }

  rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (rst_active && !g_trigseq.prev_rst_active) {
    trigseq_reset_engine();
  }
  g_trigseq.prev_rst_active = rst_active;

  if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
    uint32_t interval = trigseq_int_interval_us();
    uint8_t guard = 0;
    while (now_us >= g_trigseq.next_int_tick_us && guard < 8u) {
      trigseq_on_clock_tick(g_trigseq.next_int_tick_us);
      g_trigseq.next_int_tick_us += interval;
      ++guard;
    }
  }

  trigseq_update_pulses(now_us);

  if (edge_enc_r) {
    save_trigseq_settings();
  }
}

static void update_euclid(int32_t d_l, int32_t d_r, bool edge_enc_r, bool edge_sw1, bool edge_sw2,
                          uint64_t now_ms) {
  bool rst_active;
  uint64_t now_us = time_us_64();
  euclid_param_t prev_selected = g_euclid.selected_param;

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
    euclid_mark_token_dirty((uint8_t)prev_selected);
    euclid_mark_token_dirty((uint8_t)g_euclid.selected_param);
  }

  if (d_r != 0) {
    if (g_euclid.selected_param == EUCLID_PARAM_CLOCK) {
      if (d_r > 0) {
        euclid_set_clock_source(EUCLID_CLOCK_EXT, now_us);
      } else {
        euclid_set_clock_source(EUCLID_CLOCK_INT, now_us);
      }
    } else if (g_euclid.selected_param == EUCLID_PARAM_BPM) {
      if (g_euclid.clock == EUCLID_CLOCK_INT) {
        g_euclid.bpm = clamp_i(g_euclid.bpm + (int)d_r, EUCLID_BPM_MIN, EUCLID_BPM_MAX);
        g_euclid.next_int_tick_us = now_us + euclid_int_interval_us();
        euclid_mark_token_dirty(EUCLID_PARAM_BPM);
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
            euclid_mark_token_dirty((uint8_t)(2u + ch * 3u + 1u));
          }
          g_euclid.phase[ch] %= g_euclid.steps[ch];
        } else if (field == 1u) {
          int next_hits = clamp_i((int)g_euclid.hits[ch] + (int)d_r, 0, (int)g_euclid.steps[ch]);
          g_euclid.hits[ch] = (uint8_t)next_hits;
        } else {
          g_euclid.prob[ch] = clamp_u8i_100((int)g_euclid.prob[ch] + (int)d_r);
        }
        euclid_mark_token_dirty((uint8_t)g_euclid.selected_param);
      }
    }
    g_euclid.grid_cache_valid = false;
  }

  rst_active = hal_io_trigger_active(HAL_IO_TR2);
  if (rst_active && !g_euclid.prev_rst_active) {
    euclid_reset_engine();
  }
  g_euclid.prev_rst_active = rst_active;

  if (g_euclid.clock == EUCLID_CLOCK_INT) {
    uint32_t interval = euclid_int_interval_us();
    uint8_t guard = 0;
    while (now_us >= g_euclid.next_int_tick_us && guard < 8u) {
      euclid_on_clock_tick(g_euclid.next_int_tick_us);
      g_euclid.next_int_tick_us += interval;
      ++guard;
    }
  }

  euclid_update_pulses(now_us);

  if (edge_enc_r) {
    save_euclid_settings();
  }
}

static void draw_menu(void) {
  char line[32];
  const int menu_count = (int)(sizeof(k_menu_items) / sizeof(k_menu_items[0]));
  const uint8_t item_y0 = 12u;
  const uint8_t item_pitch = 10u;

  hal_io_oled_draw_line(0, AZURE_MENU_HEADER, true);
  for (int i = 0; i < menu_count; ++i) {
    snprintf(line, sizeof(line), "%c %s", (i == g_menu_index) ? '>' : ' ', k_menu_items[i]);
    oled_draw_text_26((uint8_t)(item_y0 + i * item_pitch), line, i == g_menu_index);
  }
  oled_draw_text_26((uint8_t)(item_y0 + menu_count * item_pitch), "", false);
  oled_draw_text_26((uint8_t)(item_y0 + (menu_count + 1) * item_pitch), "", false);
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
  const uint8_t y_ch = 12u;
  const uint8_t y_src = 22u;
  const uint8_t y_time = 32u;
  const uint8_t y_prob = 42u;
  const uint8_t y_level = 52u;
  const uint8_t y_tr_in = 84u;
  const uint8_t y_gt_out = 94u;
  const uint8_t y_status = 112u;
  uint8_t ch = g_tr2gate.selected_channel;
  unsigned time_tenths = (unsigned)((g_tr2gate.gate_time_cs[ch] + 5u) / 10u);
  bool show_status =
      (g_tr2gate.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_tr2gate.status[0] != '\0');

  hal_io_oled_draw_line(0, "TR2GATE", true);
  snprintf(line, sizeof(line), "%c CH: %c", g_tr2gate.selected_param == TR2GATE_PARAM_CHANNEL ? '>' : ' ',
           (char)('A' + ch));
  oled_draw_text_26(y_ch, line, g_tr2gate.selected_param == TR2GATE_PARAM_CHANNEL);
  snprintf(line, sizeof(line), "%c SRC: TR%u", g_tr2gate.selected_param == TR2GATE_PARAM_SOURCE ? '>' : ' ',
           (unsigned)(g_tr2gate.source[ch] + 1u));
  oled_draw_text_26(y_src, line, g_tr2gate.selected_param == TR2GATE_PARAM_SOURCE);
  snprintf(line, sizeof(line), "%c TIME: %u.%1usek", g_tr2gate.selected_param == TR2GATE_PARAM_TIME ? '>' : ' ',
           time_tenths / 10u, time_tenths % 10u);
  oled_draw_text_26(y_time, line, g_tr2gate.selected_param == TR2GATE_PARAM_TIME);
  snprintf(line, sizeof(line), "%c PROB: %u", g_tr2gate.selected_param == TR2GATE_PARAM_PROB ? '>' : ' ',
           (unsigned)g_tr2gate.prob[ch]);
  oled_draw_text_26(y_prob, line, g_tr2gate.selected_param == TR2GATE_PARAM_PROB);
  snprintf(line, sizeof(line), "%c LEVEL: %s", g_tr2gate.selected_param == TR2GATE_PARAM_LEVEL ? '>' : ' ',
           g_tr2gate.level_10v ? "10V" : "5V");
  oled_draw_text_26(y_level, line, g_tr2gate.selected_param == TR2GATE_PARAM_LEVEL);
  snprintf(line, sizeof(line), "TR IN: %u %u %u %u", read_trigger_active_src(0) ? 1u : 0u,
           read_trigger_active_src(1) ? 1u : 0u, read_trigger_active_src(2) ? 1u : 0u,
           read_trigger_active_src(3) ? 1u : 0u);
  oled_draw_text_26(y_tr_in, line, false);
  snprintf(line, sizeof(line), "GT OUT:%u %u %u %u", g_tr2gate.gate_out[0] ? 1u : 0u,
           g_tr2gate.gate_out[1] ? 1u : 0u, g_tr2gate.gate_out[2] ? 1u : 0u,
           g_tr2gate.gate_out[3] ? 1u : 0u);
  oled_draw_text_26(y_gt_out, line, false);
  if (show_status) {
    oled_draw_text_26(y_status, g_tr2gate.status, false);
  } else {
    oled_draw_text_26(y_status, "", false);
  }
}

static void tr2adsr_draw_preview_segment(int x0, int y0, int x1, int y1, uint16_t color) {
  int dx = x1 - x0;
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  for (;;) {
    if (x0 >= 0 && x0 < 160 && y0 >= 0 && y0 < 128) {
      hal_io_oled_fill_rect_color((uint8_t)x0, (uint8_t)y0, 1u, 1u, color);
    }
    if (x0 == x1 && y0 == y1) break;
    {
      int e2 = err * 2;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
    }
  }
}

static void tr2adsr_draw_preview_graph(uint8_t ch, uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  uint8_t a_eff = tr2adsr_effective_attack_pct(ch);
  uint8_t d_eff = tr2adsr_effective_decay_pct(ch);
  uint8_t s_eff = tr2adsr_effective_sustain_pct(ch);
  uint8_t r_eff = tr2adsr_effective_release_pct(ch);
  uint32_t attack_weight = a_eff;
  uint32_t decay_weight = tr2adsr_param_visible(ch, TR2ADSR_PARAM_DECAY) ? d_eff : 0u;
  uint32_t release_weight = r_eff;
  uint32_t sum = attack_weight + decay_weight + release_weight;
  uint8_t sustain_width = (g_tr2adsr.type[ch] == TR2ADSR_ENV_AR) ? 0u : (uint8_t)(w / 4u);
  uint8_t usable_w = (w > sustain_width) ? (uint8_t)(w - sustain_width) : w;
  uint8_t attack_w;
  uint8_t decay_w;
  uint8_t release_w;
  uint8_t base_y = (uint8_t)(y + h - 3u);
  uint8_t peak_y = (uint8_t)(y + 3u);
  uint8_t sustain_y =
      (uint8_t)(base_y - (uint8_t)(((uint16_t)(h > 6u ? (h - 6u) : 1u) * (uint16_t)s_eff) / 100u));
  uint8_t x0 = (uint8_t)(x + 2u);
  uint8_t x1;
  uint8_t x2;
  uint8_t x3;
  uint8_t x4;

  if (sum < 1u) sum = 1u;
  if (usable_w < 6u) usable_w = 6u;

  attack_w = (uint8_t)((usable_w * attack_weight) / sum);
  decay_w = (uint8_t)((usable_w * decay_weight) / sum);
  release_w = (uint8_t)(usable_w - attack_w - decay_w);

  if (attack_w < 2u) attack_w = 2u;
  if (release_w < 2u) release_w = 2u;
  if (g_tr2adsr.type[ch] == TR2ADSR_ENV_ADSR && decay_w < 2u) decay_w = 2u;
  if ((uint16_t)attack_w + (uint16_t)decay_w + (uint16_t)release_w > usable_w) {
    release_w = (uint8_t)(usable_w - attack_w - decay_w);
    if (release_w < 2u) release_w = 2u;
  }

  hal_io_oled_fill_rect(x, y, w, h, false);
  hal_io_oled_draw_rect(x, y, w, h, true);
  hal_io_oled_fill_rect((uint8_t)(x + 1u), base_y, (uint8_t)(w - 2u), 1u, true);

  x1 = (uint8_t)(x0 + attack_w);
  x2 = (uint8_t)(x1 + decay_w);
  x3 = (uint8_t)(x2 + sustain_width);
  x4 = (uint8_t)(x + w - 3u);
  if (x1 > x4) x1 = x4;
  if (x2 > x4) x2 = x4;
  if (x3 > x4) x3 = x4;

  tr2adsr_draw_preview_segment(x0, base_y, x1, peak_y, 0xFFFFu);

  if (g_tr2adsr.type[ch] == TR2ADSR_ENV_AR) {
    tr2adsr_draw_preview_segment(x1, peak_y, x4, base_y, 0xFFFFu);
  } else if (g_tr2adsr.type[ch] == TR2ADSR_ENV_ASR) {
    tr2adsr_draw_preview_segment(x1, peak_y, x3, peak_y, 0xFFFFu);
    tr2adsr_draw_preview_segment(x3, peak_y, x4, base_y, 0xFFFFu);
  } else {
    tr2adsr_draw_preview_segment(x1, peak_y, x2, sustain_y, 0xFFFFu);
    tr2adsr_draw_preview_segment(x2, sustain_y, x3, sustain_y, 0xFFFFu);
    tr2adsr_draw_preview_segment(x3, sustain_y, x4, base_y, 0xFFFFu);
  }
}

static void draw_tr2adsr(void) {
  char header[27];
  char env_text[27];
  char status[27];
  char token_text[9][20];
  bool token_selected[9];
  uint8_t ch;
  uint8_t a_eff;
  uint8_t d_eff;
  uint8_t s_eff;
  uint8_t r_eff;
  const uint8_t menu_x = 0u;
  const uint8_t menu_w = 82u;
  const uint8_t graph_x = 88u;
  const uint8_t graph_y = 64u;
  const uint8_t graph_w = 68u;
  const uint8_t graph_h = 38u;
  const uint8_t y_src = 12u;
  const uint8_t y_type = 22u;
  const uint8_t y_time = 32u;
  const uint8_t y_prob = 42u;
  const uint8_t y_level = 52u;
  const uint8_t y_attack = 64u;
  const uint8_t y_decay = 74u;
  const uint8_t y_sustain = 84u;
  const uint8_t y_release = 94u;
  bool graph_changed;
  static const uint8_t token_x[9] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
  static const uint8_t token_y[9] = {12u, 22u, 32u, 42u, 52u, 64u, 74u, 84u, 94u};
  static const uint8_t token_w[9] = {82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u};
  bool show_status =
      (g_tr2adsr.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_tr2adsr.status[0] != '\0');

  tr2adsr_sync_screen_channel();
  ch = g_tr2adsr.selected_channel;
  a_eff = tr2adsr_effective_attack_pct(ch);
  d_eff = tr2adsr_effective_decay_pct(ch);
  s_eff = tr2adsr_effective_sustain_pct(ch);
  r_eff = tr2adsr_effective_release_pct(ch);

  snprintf(header, sizeof(header), "TR2ADSR %u/4 CH %c", (unsigned)(ch + 1u), (char)('A' + ch));
  snprintf(token_text[0], sizeof(token_text[0]), " SRC: TR%u", (unsigned)(g_tr2adsr.source[ch] + 1u));
  snprintf(token_text[1], sizeof(token_text[1]), " TYPE: %s", tr2adsr_type_label(g_tr2adsr.type[ch]));
  snprintf(token_text[2], sizeof(token_text[2]), " TIME: %u.%01usek", (unsigned)(g_tr2adsr.total_time_ds[ch] / 10u),
           (unsigned)(g_tr2adsr.total_time_ds[ch] % 10u));
  snprintf(token_text[3], sizeof(token_text[3]), " PROB: %u", (unsigned)g_tr2adsr.prob[ch]);
  snprintf(token_text[4], sizeof(token_text[4]), " LEVEL: %s", g_tr2adsr.level_10v ? "10V" : "5V");
  snprintf(token_text[5], sizeof(token_text[5]), " A: %u", (unsigned)a_eff);
  if (tr2adsr_param_visible(ch, TR2ADSR_PARAM_DECAY)) {
    snprintf(token_text[6], sizeof(token_text[6]), " D: %u", (unsigned)d_eff);
  } else {
    snprintf(token_text[6], sizeof(token_text[6]), " D: --");
  }
  if (tr2adsr_param_visible(ch, TR2ADSR_PARAM_SUSTAIN)) {
    snprintf(token_text[7], sizeof(token_text[7]), " S: %u", (unsigned)s_eff);
  } else {
    snprintf(token_text[7], sizeof(token_text[7]), " S: --");
  }
  snprintf(token_text[8], sizeof(token_text[8]), " R: %u", (unsigned)r_eff);

  for (uint8_t i = 0u; i < 9u; ++i) {
    token_selected[i] = (tr2adsr_param_to_ui_index(g_tr2adsr.selected_param) == i);
    token_text[i][0] = token_selected[i] ? '>' : ' ';
  }

  snprintf(env_text, sizeof(env_text), "ENV %s LVL %.2f", tr2adsr_state_label(g_tr2adsr.env_state[ch]), g_tr2adsr.level[ch]);
  if (show_status) {
    snprintf(status, sizeof(status), "%s", g_tr2adsr.status);
  } else {
    status[0] = '\0';
  }

  if (g_tr2adsr.ui_force_full_redraw || !g_tr2adsr.ui_cache_valid) {
    hal_io_oled_fill_rect(0u, 0u, 160u, 8u, false);
    hal_io_oled_fill_rect(menu_x, y_src, menu_w, 92u, false);
    hal_io_oled_fill_rect(graph_x, y_attack, graph_w, 42u, false);
    hal_io_oled_fill_rect(0u, 112u, 160u, 16u, false);
  }

  if (g_tr2adsr.ui_header_dirty || !g_tr2adsr.ui_cache_valid || strcmp(header, g_tr2adsr.ui_header) != 0) {
    hal_io_oled_draw_line(0, header, true);
    snprintf(g_tr2adsr.ui_header, sizeof(g_tr2adsr.ui_header), "%s", header);
    g_tr2adsr.ui_header_dirty = false;
  }

  for (uint8_t i = 0u; i < 9u; ++i) {
    if (g_tr2adsr.ui_token_dirty[i] || !g_tr2adsr.ui_cache_valid ||
        token_selected[i] != g_tr2adsr.ui_token_selected[i] ||
        strcmp(token_text[i], g_tr2adsr.ui_token_text[i]) != 0) {
      hal_io_oled_fill_rect(token_x[i], token_y[i], token_w[i], 8u, token_selected[i]);
      hal_io_oled_draw_text(token_x[i], token_y[i], token_text[i], token_selected[i]);
      snprintf(g_tr2adsr.ui_token_text[i], sizeof(g_tr2adsr.ui_token_text[i]), "%s", token_text[i]);
      g_tr2adsr.ui_token_selected[i] = token_selected[i];
      g_tr2adsr.ui_token_dirty[i] = false;
    }
  }

  graph_changed = !g_tr2adsr.ui_cache_valid || g_tr2adsr.ui_graph_dirty || g_tr2adsr.ui_graph_channel != ch ||
                  g_tr2adsr.ui_graph_type != (uint8_t)g_tr2adsr.type[ch] || g_tr2adsr.ui_graph_attack != a_eff ||
                  g_tr2adsr.ui_graph_decay != d_eff || g_tr2adsr.ui_graph_sustain != s_eff ||
                  g_tr2adsr.ui_graph_release != r_eff;
  if (graph_changed) {
    tr2adsr_draw_preview_graph(ch, graph_x, graph_y, graph_w, graph_h);
    g_tr2adsr.ui_graph_channel = ch;
    g_tr2adsr.ui_graph_type = (uint8_t)g_tr2adsr.type[ch];
    g_tr2adsr.ui_graph_attack = a_eff;
    g_tr2adsr.ui_graph_decay = d_eff;
    g_tr2adsr.ui_graph_sustain = s_eff;
    g_tr2adsr.ui_graph_release = r_eff;
    g_tr2adsr.ui_graph_dirty = false;
  }

  if (g_tr2adsr.ui_env_dirty || !g_tr2adsr.ui_cache_valid || strcmp(env_text, g_tr2adsr.ui_env_text) != 0) {
    oled_draw_text_26(112u, env_text, false);
    snprintf(g_tr2adsr.ui_env_text, sizeof(g_tr2adsr.ui_env_text), "%s", env_text);
    g_tr2adsr.ui_env_dirty = false;
  }

  if (g_tr2adsr.ui_status_dirty || !g_tr2adsr.ui_cache_valid || show_status != g_tr2adsr.ui_status_visible ||
      strcmp(status, g_tr2adsr.ui_status) != 0) {
    oled_draw_text_26(120u, status, false);
    snprintf(g_tr2adsr.ui_status, sizeof(g_tr2adsr.ui_status), "%s", status);
    g_tr2adsr.ui_status_visible = show_status;
    g_tr2adsr.ui_status_dirty = false;
  }

  g_tr2adsr.ui_force_full_redraw = false;
  g_tr2adsr.ui_cache_valid = true;
}

static void draw_burstgen(void) {
  char header[27];
  char run_text[27];
  char in_text[27];
  char pattern_text[27];
  char status[27];
  char token_text[6][20];
  bool token_selected[6];
  char pattern[12];
  uint8_t ch = g_burstgen.selected_channel;
  uint8_t steps = burstgen_steps_for_mode(g_burstgen.signature_mode);
  const uint8_t y_ch = 12u;
  const uint8_t y_sig = 22u;
  const uint8_t y_bpm = 32u;
  const uint8_t y_swg = 42u;
  const uint8_t y_prb = 52u;
  const uint8_t y_lvl = 62u;
  const uint8_t y_run = 92u;
  const uint8_t y_in = 100u;
  const uint8_t y_pattern = 108u;
  const uint8_t y_status = 120u;
  static const uint8_t token_x[6] = {0u, 0u, 0u, 0u, 0u, 0u};
  static const uint8_t token_y[6] = {12u, 22u, 32u, 42u, 52u, 62u};
  static const uint8_t token_w[6] = {82u, 82u, 82u, 82u, 82u, 82u};
  bool show_status =
      (g_burstgen.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_burstgen.status[0] != '\0');

  burstgen_format_pattern(pattern, sizeof(pattern), g_burstgen.pattern_mask[ch], steps);

  snprintf(header, sizeof(header), "BURST GEN");
  snprintf(token_text[0], sizeof(token_text[0]), " CH: %c", (char)('A' + ch));
  snprintf(token_text[1], sizeof(token_text[1]), " SIG: %s", burstgen_signature_label(g_burstgen.signature_mode));
  snprintf(token_text[2], sizeof(token_text[2]), " BPM: %u", (unsigned)g_burstgen.bpm);
  snprintf(token_text[3], sizeof(token_text[3]), " SWG: %u", (unsigned)g_burstgen.swing_pct);
  snprintf(token_text[4], sizeof(token_text[4]), " PRB: %u", (unsigned)g_burstgen.probability);
  snprintf(token_text[5], sizeof(token_text[5]), " LVL: %s", g_burstgen.level_10v ? "10V" : "5V");
  for (uint8_t i = 0u; i < BURSTGEN_PARAM_COUNT; ++i) {
    token_selected[i] = (g_burstgen.selected_param == (burstgen_param_t)i);
    token_text[i][0] = token_selected[i] ? '>' : ' ';
  }
  snprintf(run_text, sizeof(run_text), "RUN: %u %u %u %u", g_burstgen.running[0] ? 1u : 0u,
           g_burstgen.running[1] ? 1u : 0u, g_burstgen.running[2] ? 1u : 0u, g_burstgen.running[3] ? 1u : 0u);
  snprintf(in_text, sizeof(in_text), "IN : %u %u %u %u", read_trigger_active_src(0) ? 1u : 0u,
           read_trigger_active_src(1) ? 1u : 0u, read_trigger_active_src(2) ? 1u : 0u,
           read_trigger_active_src(3) ? 1u : 0u);
  snprintf(pattern_text, sizeof(pattern_text), "P%c %s %u/%u", (char)('A' + ch), pattern,
           (unsigned)(g_burstgen.current_step[ch] + 1u), (unsigned)steps);
  if (show_status) {
    snprintf(status, sizeof(status), "%s", g_burstgen.status);
  } else {
    status[0] = '\0';
  }

  if (g_burstgen.ui_force_full_redraw || !g_burstgen.ui_cache_valid) {
    hal_io_oled_fill_rect(0u, 0u, 160u, 8u, false);
    hal_io_oled_fill_rect(0u, y_ch, 160u, 60u, false);
    hal_io_oled_fill_rect(0u, y_run, 160u, 36u, false);
  }

  if (g_burstgen.ui_header_dirty || !g_burstgen.ui_cache_valid || strcmp(header, g_burstgen.ui_header) != 0) {
    hal_io_oled_draw_line(0, header, true);
    snprintf(g_burstgen.ui_header, sizeof(g_burstgen.ui_header), "%s", header);
    g_burstgen.ui_header_dirty = false;
  }

  for (uint8_t i = 0u; i < BURSTGEN_PARAM_COUNT; ++i) {
    if (g_burstgen.ui_token_dirty[i] || !g_burstgen.ui_cache_valid ||
        token_selected[i] != g_burstgen.ui_token_selected[i] ||
        strcmp(token_text[i], g_burstgen.ui_token_text[i]) != 0) {
      hal_io_oled_fill_rect(token_x[i], token_y[i], token_w[i], 8u, token_selected[i]);
      hal_io_oled_draw_text(token_x[i], token_y[i], token_text[i], token_selected[i]);
      snprintf(g_burstgen.ui_token_text[i], sizeof(g_burstgen.ui_token_text[i]), "%s", token_text[i]);
      g_burstgen.ui_token_selected[i] = token_selected[i];
      g_burstgen.ui_token_dirty[i] = false;
    }
  }

  if (g_burstgen.ui_run_dirty || !g_burstgen.ui_cache_valid || strcmp(run_text, g_burstgen.ui_run_text) != 0) {
    oled_draw_text_26(y_run, run_text, false);
    snprintf(g_burstgen.ui_run_text, sizeof(g_burstgen.ui_run_text), "%s", run_text);
    g_burstgen.ui_run_dirty = false;
  }

  if (g_burstgen.ui_in_dirty || !g_burstgen.ui_cache_valid || strcmp(in_text, g_burstgen.ui_in_text) != 0) {
    oled_draw_text_26(y_in, in_text, false);
    snprintf(g_burstgen.ui_in_text, sizeof(g_burstgen.ui_in_text), "%s", in_text);
    g_burstgen.ui_in_dirty = false;
  }

  if (g_burstgen.ui_pattern_dirty || !g_burstgen.ui_cache_valid ||
      strcmp(pattern_text, g_burstgen.ui_pattern_text) != 0) {
    oled_draw_text_26(y_pattern, pattern_text, false);
    snprintf(g_burstgen.ui_pattern_text, sizeof(g_burstgen.ui_pattern_text), "%s", pattern_text);
    g_burstgen.ui_pattern_dirty = false;
  }

  if (g_burstgen.ui_status_dirty || !g_burstgen.ui_cache_valid || show_status != g_burstgen.ui_status_visible ||
      strcmp(status, g_burstgen.ui_status) != 0) {
    oled_draw_text_26(y_status, status, false);
    snprintf(g_burstgen.ui_status, sizeof(g_burstgen.ui_status), "%s", status);
    g_burstgen.ui_status_visible = show_status;
    g_burstgen.ui_status_dirty = false;
  }

  g_burstgen.ui_force_full_redraw = false;
  g_burstgen.ui_cache_valid = true;
}

static void draw_cvgen(void) {
  char line[32];
  char min_str[12];
  char max_str[12];
  char out_str[12];
  uint8_t ch_idx = g_preset_ui.screen < 4u ? g_preset_ui.screen : 0u;
  cvgen_channel_state_t* channel = &g_cvgen.ch[ch_idx];
  uint8_t walk_mode = cvgen_walk_mode_from_param(channel->param2);
  const uint8_t y_alg = 12u;
  const uint8_t y_clk = 22u;
  const uint8_t y_src = 32u;
  const uint8_t y_rate = 42u;
  const uint8_t y_min = 52u;
  const uint8_t y_max = 62u;
  const uint8_t y_a = 72u;
  const uint8_t y_b = 82u;
  const uint8_t y_out = 100u;
  const uint8_t y_status = 112u;
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
  oled_draw_text_26(y_alg, line, channel->selected_param == CVGEN_PARAM_ALGO);
  snprintf(line, sizeof(line), "%c CLK: %s", channel->selected_param == CVGEN_PARAM_CLOCK ? '>' : ' ',
           cvgen_clock_label(channel->clock_mode));
  oled_draw_text_26(y_clk, line, channel->selected_param == CVGEN_PARAM_CLOCK);
  snprintf(line, sizeof(line), "%c SRC: TR%u", channel->selected_param == CVGEN_PARAM_SOURCE ? '>' : ' ',
           (unsigned)(channel->source + 1u));
  oled_draw_text_26(y_src, line, channel->selected_param == CVGEN_PARAM_SOURCE);
  snprintf(line, sizeof(line), "%c RATE: %u.%01uHz", channel->selected_param == CVGEN_PARAM_RATE ? '>' : ' ',
           (unsigned)(channel->rate_dhz / 10u), (unsigned)(channel->rate_dhz % 10u));
  oled_draw_text_26(y_rate, line, channel->selected_param == CVGEN_PARAM_RATE);
  snprintf(line, sizeof(line), "%c MIN: %s", channel->selected_param == CVGEN_PARAM_MIN ? '>' : ' ', min_str);
  oled_draw_text_26(y_min, line, channel->selected_param == CVGEN_PARAM_MIN);
  snprintf(line, sizeof(line), "%c MAX: %s", channel->selected_param == CVGEN_PARAM_MAX ? '>' : ' ', max_str);
  oled_draw_text_26(y_max, line, channel->selected_param == CVGEN_PARAM_MAX);
  snprintf(line, sizeof(line), "%c %s: %u", channel->selected_param == CVGEN_PARAM_A ? '>' : ' ',
           cvgen_param_a_label(channel->algo), (unsigned)channel->param1);
  oled_draw_text_26(y_a, line, channel->selected_param == CVGEN_PARAM_A);
  if (channel->algo == CVGEN_ALGO_WALK) {
    snprintf(line, sizeof(line), "%c %s: %s", channel->selected_param == CVGEN_PARAM_B ? '>' : ' ',
             cvgen_param_b_label(channel->algo), cvgen_walk_mode_label(walk_mode));
  } else {
    snprintf(line, sizeof(line), "%c %s: %u", channel->selected_param == CVGEN_PARAM_B ? '>' : ' ',
             cvgen_param_b_label(channel->algo), (unsigned)channel->param2);
  }
  oled_draw_text_26(y_b, line, channel->selected_param == CVGEN_PARAM_B);

  snprintf(line, sizeof(line), "OUT: %s", out_str);
  oled_draw_text_26(96u, "", false);
  oled_draw_text_26(104u, "", false);
  oled_draw_text_26(y_out, line, false);
  if (show_status) {
    oled_draw_text_26(y_status, g_cvgen.status, false);
  } else {
    oled_draw_text_26(y_status, "", false);
  }
  oled_draw_text_26(120u, "", false);
}

static void draw_grids(void) {
  char header[27];
  char status[27];
  char token_text[9][16];
  bool token_selected[9];
  bool show_status =
      (g_grids.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_grids.status[0] != '\0');
  bool preview[4][32];
  bool progress[4][32];
  uint8_t map_x = grids_effective_map_x();
  uint8_t map_y = grids_effective_map_y();
  uint8_t randomness = grids_effective_randomness();
  const uint8_t grid_x = 14u;
  const uint8_t grid_top = 57u;
  const uint8_t channel_stride = 14u;  // 2 rows * 6px + 2px visual gap between channels
  const uint8_t cell_pitch = 6u;
  const uint8_t cell_size = 6u;
  static const uint8_t token_x[9] = {0u, 60u, 0u, 54u, 108u, 0u, 60u, 0u, 60u};
  static const uint8_t token_y[9] = {14u, 14u, 24u, 24u, 24u, 34u, 34u, 44u, 44u};
  static const uint8_t token_w[9] = {48u, 48u, 48u, 48u, 52u, 48u, 48u, 48u, 48u};

  snprintf(header, sizeof(header), "%-22s%u/%u", "GRIDS", (unsigned)(g_preset_ui.screen + 1u),
           (unsigned)active_app_screen_count());

  snprintf(token_text[0], sizeof(token_text[0]), "CLK:%s", g_grids.clock == GRIDS_CLOCK_INT ? "INT" : "EXT");
  snprintf(token_text[1], sizeof(token_text[1]), "BPM:%3d", g_grids.bpm);
  snprintf(token_text[2], sizeof(token_text[2]), "MAPX:%3u", (unsigned)map_x);
  snprintf(token_text[3], sizeof(token_text[3]), "MAPY:%3u", (unsigned)map_y);
  snprintf(token_text[4], sizeof(token_text[4]), "RNDM:%3u", (unsigned)randomness);
  snprintf(token_text[5], sizeof(token_text[5]), "P1:%3u", (unsigned)g_grids.prob[0]);
  snprintf(token_text[6], sizeof(token_text[6]), "P2:%3u", (unsigned)g_grids.prob[1]);
  snprintf(token_text[7], sizeof(token_text[7]), "P3:%3u", (unsigned)g_grids.prob[2]);
  snprintf(token_text[8], sizeof(token_text[8]), "P4:%3u", (unsigned)g_grids.prob[3]);

  for (uint8_t i = 0u; i < GRIDS_PARAM_COUNT; ++i) {
    token_selected[i] = (g_grids.selected_param == (grids_param_t)i);
  }

  if (show_status) {
    snprintf(status, sizeof(status), "%s", g_grids.status);
  } else {
    status[0] = '\0';
  }

  if (g_grids.ui_force_full_redraw || !g_grids.ui_cache_valid) {
    hal_io_oled_fill_rect(0u, 0u, 160u, 56u, false);
    hal_io_oled_fill_rect(0u, 112u, 160u, 16u, false);
  }

  if (g_grids.ui_header_dirty || !g_grids.ui_cache_valid || strcmp(header, g_grids.ui_header) != 0) {
    hal_io_oled_draw_line(0, header, true);
    snprintf(g_grids.ui_header, sizeof(g_grids.ui_header), "%s", header);
    g_grids.ui_header_dirty = false;
  }

  for (uint8_t i = 0u; i < GRIDS_PARAM_COUNT; ++i) {
    if (g_grids.ui_token_dirty[i] || !g_grids.ui_cache_valid || token_selected[i] != g_grids.ui_token_selected[i] ||
        strcmp(token_text[i], g_grids.ui_token_text[i]) != 0) {
      hal_io_oled_fill_rect(token_x[i], token_y[i], token_w[i], 8u, token_selected[i]);
      hal_io_oled_draw_text(token_x[i], token_y[i], token_text[i], token_selected[i]);
      snprintf(g_grids.ui_token_text[i], sizeof(g_grids.ui_token_text[i]), "%s", token_text[i]);
      g_grids.ui_token_selected[i] = token_selected[i];
      g_grids.ui_token_dirty[i] = false;
    }
  }

  if (g_grids.ui_status_dirty || !g_grids.ui_cache_valid || show_status != g_grids.ui_status_visible ||
      strcmp(status, g_grids.ui_status) != 0) {
    hal_io_oled_fill_rect(0u, 120u, 160u, 8u, false);
    if (show_status) {
      hal_io_oled_draw_text(0u, 120u, status, false);
    } else {
      hal_io_oled_draw_text(0u, 120u, "FILL 1-4: POTS 1-4", false);
    }
    snprintf(g_grids.ui_status, sizeof(g_grids.ui_status), "%s", status);
    g_grids.ui_status_visible = show_status;
    g_grids.ui_status_dirty = false;
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
      grids_engine_step(&sim, map_x, map_y, randomness, fill, trig);
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

  if (g_grids.preview_force_full_redraw || !g_grids.preview_cache_valid) {
    hal_io_oled_fill_rect(0u, grid_top, 160u, 56u, false);
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
  g_grids.preview_force_full_redraw = false;
  g_grids.ui_force_full_redraw = false;
  g_grids.ui_cache_valid = true;
}

static void trigseq_draw_grid_cell(uint8_t step, bool on, bool selected, bool progress) {
  const uint16_t color_blue = 0x001Fu;
  const uint8_t grid_x = 10u;
  const uint8_t grid_y = 81u;
  const uint8_t cell = 7u;
  uint8_t row = (uint8_t)(step / 16u);
  uint8_t col = (uint8_t)(step % 16u);
  uint8_t x = (uint8_t)(grid_x + col * cell);
  uint8_t y = (uint8_t)(grid_y + row * cell);

  hal_io_oled_draw_rect_color(x, y, 7u, 7u, selected ? color_blue : 0xFFFFu);
  hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 1u), 5u, 5u, false);

  if (on) {
    hal_io_oled_fill_rect((uint8_t)(x + 2u), (uint8_t)(y + 2u), 3u, 3u, true);
  }
  if (selected) {
    hal_io_oled_fill_rect_color((uint8_t)(x + 1u), (uint8_t)(y + 1u), 1u, 5u, color_blue);
    hal_io_oled_fill_rect_color((uint8_t)(x + 5u), (uint8_t)(y + 1u), 1u, 5u, color_blue);
    hal_io_oled_fill_rect_color((uint8_t)(x + 1u), (uint8_t)(y + 1u), 5u, 1u, color_blue);
    hal_io_oled_fill_rect_color((uint8_t)(x + 1u), (uint8_t)(y + 5u), 5u, 1u, color_blue);
    hal_io_oled_fill_rect((uint8_t)(x + 2u), (uint8_t)(y + 2u), 3u, 3u, false);
    if (on) {
      hal_io_oled_fill_rect((uint8_t)(x + 2u), (uint8_t)(y + 2u), 3u, 3u, true);
    }
  }
  if (progress) {
    hal_io_oled_fill_rect((uint8_t)(x + 1u), (uint8_t)(y + 5u), 5u, 1u, true);
  }
}

static void euclid_draw_grid_cell(uint8_t step, bool visible, bool on, bool progress) {
  const uint8_t grid_x = 10u;
  const uint8_t grid_y = 67u;
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
  char header[27];
  char status[27];
  char clk_src[27];
  char token_text[8][20];
  bool token_selected[8];
  const uint8_t y_status = 70u;
  const uint8_t y_clk_src = 120u;
  bool force_grid = !g_trigseq.grid_cache_valid || (g_trigseq.grid_cache_mode != g_trigseq.len_mode);
  bool show_status =
      (g_trigseq.status_until_ms > to_ms_since_boot(get_absolute_time())) &&
      (g_trigseq.status[0] != '\0');
  static const uint8_t token_x[8] = {0u, 0u, 72u, 0u, 66u, 0u, 66u, 0u};
  static const uint8_t token_y[8] = {12u, 23u, 23u, 34u, 34u, 45u, 45u, 56u};
  static const uint8_t token_w[8] = {60u, 60u, 48u, 60u, 60u, 60u, 60u, 72u};

  snprintf(header, sizeof(header), "TRIG SEQ %u/%u %s", (unsigned)(g_preset_ui.screen + 1u),
           (unsigned)active_app_screen_count(),
           g_trigseq.focus == TRIGSEQ_FOCUS_GRID ? "GRID" : "MENU");
  snprintf(token_text[TRIGSEQ_PARAM_LEN], sizeof(token_text[TRIGSEQ_PARAM_LEN]), "LEN:%s",
           trigseq_mode_label(g_trigseq.len_mode));
  snprintf(token_text[TRIGSEQ_PARAM_CLOCK], sizeof(token_text[TRIGSEQ_PARAM_CLOCK]), "CLOCK:%s",
           g_trigseq.clock == TRIGSEQ_CLOCK_INT ? "INT" : "EXT");
  if (g_trigseq.clock == TRIGSEQ_CLOCK_INT) {
    snprintf(token_text[TRIGSEQ_PARAM_BPM], sizeof(token_text[TRIGSEQ_PARAM_BPM]), "BPM:%3d", g_trigseq.bpm);
  } else {
    snprintf(token_text[TRIGSEQ_PARAM_BPM], sizeof(token_text[TRIGSEQ_PARAM_BPM]), "BPM:---");
  }
  snprintf(token_text[TRIGSEQ_PARAM_PROB1], sizeof(token_text[TRIGSEQ_PARAM_PROB1]), "PROB1:%3u",
           (unsigned)g_trigseq.prob[0]);
  snprintf(token_text[TRIGSEQ_PARAM_PROB2], sizeof(token_text[TRIGSEQ_PARAM_PROB2]), "PROB2:%3u",
           (unsigned)g_trigseq.prob[1]);
  snprintf(token_text[TRIGSEQ_PARAM_PROB3], sizeof(token_text[TRIGSEQ_PARAM_PROB3]), "PROB3:%3u",
           (unsigned)g_trigseq.prob[2]);
  snprintf(token_text[TRIGSEQ_PARAM_PROB4], sizeof(token_text[TRIGSEQ_PARAM_PROB4]), "PROB4:%3u",
           (unsigned)g_trigseq.prob[3]);
  snprintf(token_text[TRIGSEQ_PARAM_EDIT_GRID], sizeof(token_text[TRIGSEQ_PARAM_EDIT_GRID]), "EDIT GRID");

  for (uint8_t i = 0u; i < TRIGSEQ_PARAM_COUNT; ++i) {
    token_selected[i] = (g_trigseq.focus == TRIGSEQ_FOCUS_MENU) && (g_trigseq.selected_param == (trigseq_param_t)i);
  }

  if (show_status) {
    snprintf(status, sizeof(status), "%s", g_trigseq.status);
  } else {
    status[0] = '\0';
  }

  snprintf(clk_src, sizeof(clk_src), "CLK:TR1 RST:TR2 %s", g_trigseq.clock == TRIGSEQ_CLOCK_INT ? "INT" : "EXT");

  if (g_trigseq.ui_force_full_redraw || !g_trigseq.ui_cache_valid) {
    hal_io_oled_fill_rect(0u, 0u, 160u, 92u, false);
    hal_io_oled_fill_rect(0u, 120u, 160u, 8u, false);
  }

  if (g_trigseq.ui_header_dirty || !g_trigseq.ui_cache_valid || strcmp(header, g_trigseq.ui_header) != 0) {
    hal_io_oled_draw_line(0, header, true);
    snprintf(g_trigseq.ui_header, sizeof(g_trigseq.ui_header), "%s", header);
    g_trigseq.ui_header_dirty = false;
  }

  for (uint8_t i = 0u; i < TRIGSEQ_PARAM_COUNT; ++i) {
    if (g_trigseq.ui_token_dirty[i] || !g_trigseq.ui_cache_valid ||
        token_selected[i] != g_trigseq.ui_token_selected[i] ||
        strcmp(token_text[i], g_trigseq.ui_token_text[i]) != 0) {
      hal_io_oled_fill_rect(token_x[i], token_y[i], token_w[i], 8u, token_selected[i]);
      hal_io_oled_draw_text(token_x[i], token_y[i], token_text[i], token_selected[i]);
      snprintf(g_trigseq.ui_token_text[i], sizeof(g_trigseq.ui_token_text[i]), "%s", token_text[i]);
      g_trigseq.ui_token_selected[i] = token_selected[i];
      g_trigseq.ui_token_dirty[i] = false;
    }
  }

  if (g_trigseq.ui_status_dirty || !g_trigseq.ui_cache_valid || show_status != g_trigseq.ui_status_visible ||
      strcmp(status, g_trigseq.ui_status) != 0) {
    oled_draw_text_26(y_status, status, false);
    snprintf(g_trigseq.ui_status, sizeof(g_trigseq.ui_status), "%s", status);
    g_trigseq.ui_status_visible = show_status;
    g_trigseq.ui_status_dirty = false;
  }

  if (g_trigseq.ui_clk_src_dirty || !g_trigseq.ui_cache_valid || strcmp(clk_src, g_trigseq.ui_clk_src) != 0) {
    oled_draw_text_26(y_clk_src, clk_src, false);
    snprintf(g_trigseq.ui_clk_src, sizeof(g_trigseq.ui_clk_src), "%s", clk_src);
    g_trigseq.ui_clk_src_dirty = false;
  }

  if (force_grid) {
    hal_io_oled_fill_rect(0u, 81u, 160u, 28u, false);
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
  g_trigseq.ui_force_full_redraw = false;
  g_trigseq.ui_cache_valid = true;
}

static void draw_euclid(void) {
  char header[27];
  char status[27];
  char clk_src[27];
  char token_text[14][16];
  bool token_selected[14];
  const uint8_t y_clock = 12u;
  const uint8_t y_ch1 = 22u;
  const uint8_t y_ch2 = 32u;
  const uint8_t y_ch3 = 42u;
  const uint8_t y_ch4 = 52u;
  const uint8_t y_status = 108u;
  const uint8_t y_clk_src = 120u;
  bool force_grid = !g_euclid.grid_cache_valid;
  bool show_status =
      (g_euclid.status_until_ms > to_ms_since_boot(get_absolute_time())) && (g_euclid.status[0] != '\0');
  static const uint8_t token_x[14] = {0u, 60u, 12u, 60u, 108u, 12u, 60u, 108u, 12u, 60u, 108u, 12u, 60u, 108u};
  static const uint8_t token_y[14] = {12u, 12u, 22u, 22u, 22u, 32u, 32u, 32u, 42u, 42u, 42u, 52u, 52u, 52u};
  static const uint8_t token_w[14] = {54u, 48u, 42u, 42u, 48u, 42u, 42u, 48u, 42u, 42u, 48u, 42u, 42u, 48u};

  snprintf(header, sizeof(header), "4X EUCLID %u/%u", (unsigned)(g_preset_ui.screen + 1u),
           (unsigned)active_app_screen_count());

  snprintf(token_text[EUCLID_PARAM_CLOCK], sizeof(token_text[EUCLID_PARAM_CLOCK]), "CLOCK:%s",
           g_euclid.clock == EUCLID_CLOCK_INT ? "INT" : "EXT");
  if (g_euclid.clock == EUCLID_CLOCK_INT) {
    snprintf(token_text[EUCLID_PARAM_BPM], sizeof(token_text[EUCLID_PARAM_BPM]), "BPM:%3d", g_euclid.bpm);
  } else {
    snprintf(token_text[EUCLID_PARAM_BPM], sizeof(token_text[EUCLID_PARAM_BPM]), "BPM:---");
  }
  snprintf(token_text[EUCLID_PARAM_CH1_STEPS], sizeof(token_text[EUCLID_PARAM_CH1_STEPS]), "STPS:%2u",
           g_euclid.steps[0]);
  snprintf(token_text[EUCLID_PARAM_CH1_HITS], sizeof(token_text[EUCLID_PARAM_CH1_HITS]), "HITS:%2u",
           g_euclid.hits[0]);
  snprintf(token_text[EUCLID_PARAM_CH1_PRB], sizeof(token_text[EUCLID_PARAM_CH1_PRB]), "PRB:%3u", g_euclid.prob[0]);
  snprintf(token_text[EUCLID_PARAM_CH2_STEPS], sizeof(token_text[EUCLID_PARAM_CH2_STEPS]), "STPS:%2u",
           g_euclid.steps[1]);
  snprintf(token_text[EUCLID_PARAM_CH2_HITS], sizeof(token_text[EUCLID_PARAM_CH2_HITS]), "HITS:%2u",
           g_euclid.hits[1]);
  snprintf(token_text[EUCLID_PARAM_CH2_PRB], sizeof(token_text[EUCLID_PARAM_CH2_PRB]), "PRB:%3u", g_euclid.prob[1]);
  snprintf(token_text[EUCLID_PARAM_CH3_STEPS], sizeof(token_text[EUCLID_PARAM_CH3_STEPS]), "STPS:%2u",
           g_euclid.steps[2]);
  snprintf(token_text[EUCLID_PARAM_CH3_HITS], sizeof(token_text[EUCLID_PARAM_CH3_HITS]), "HITS:%2u",
           g_euclid.hits[2]);
  snprintf(token_text[EUCLID_PARAM_CH3_PRB], sizeof(token_text[EUCLID_PARAM_CH3_PRB]), "PRB:%3u", g_euclid.prob[2]);
  snprintf(token_text[EUCLID_PARAM_CH4_STEPS], sizeof(token_text[EUCLID_PARAM_CH4_STEPS]), "STPS:%2u",
           g_euclid.steps[3]);
  snprintf(token_text[EUCLID_PARAM_CH4_HITS], sizeof(token_text[EUCLID_PARAM_CH4_HITS]), "HITS:%2u",
           g_euclid.hits[3]);
  snprintf(token_text[EUCLID_PARAM_CH4_PRB], sizeof(token_text[EUCLID_PARAM_CH4_PRB]), "PRB:%3u", g_euclid.prob[3]);

  for (uint8_t i = 0u; i < EUCLID_PARAM_COUNT; ++i) {
    token_selected[i] = (g_euclid.selected_param == (euclid_param_t)i);
  }

  if (show_status) {
    snprintf(status, sizeof(status), "%s", g_euclid.status);
  } else {
    status[0] = '\0';
  }

  snprintf(clk_src, sizeof(clk_src), "CLK:TR1 RST:TR2 %s", g_euclid.clock == EUCLID_CLOCK_INT ? "INT" : "EXT");

  if (g_euclid.ui_force_full_redraw || !g_euclid.ui_cache_valid) {
    hal_io_oled_fill_rect(0u, 0u, 160u, 64u, false);
    hal_io_oled_draw_text(0u, y_ch1, "1", false);
    hal_io_oled_draw_text(0u, y_ch2, "2", false);
    hal_io_oled_draw_text(0u, y_ch3, "3", false);
    hal_io_oled_draw_text(0u, y_ch4, "4", false);
    oled_draw_text_26(y_status, "", false);
    oled_draw_text_26(y_clk_src, "", false);
  }

  if (g_euclid.ui_header_dirty || !g_euclid.ui_cache_valid || strcmp(header, g_euclid.ui_header) != 0) {
    hal_io_oled_draw_line(0, header, true);
    snprintf(g_euclid.ui_header, sizeof(g_euclid.ui_header), "%s", header);
    g_euclid.ui_header_dirty = false;
  }

  for (uint8_t i = 0u; i < EUCLID_PARAM_COUNT; ++i) {
    if (g_euclid.ui_token_dirty[i] || !g_euclid.ui_cache_valid ||
        token_selected[i] != g_euclid.ui_token_selected[i] ||
        strcmp(token_text[i], g_euclid.ui_token_text[i]) != 0) {
      hal_io_oled_fill_rect(token_x[i], token_y[i], token_w[i], 8u, token_selected[i]);
      hal_io_oled_draw_text(token_x[i], token_y[i], token_text[i], token_selected[i]);
      snprintf(g_euclid.ui_token_text[i], sizeof(g_euclid.ui_token_text[i]), "%s", token_text[i]);
      g_euclid.ui_token_selected[i] = token_selected[i];
      g_euclid.ui_token_dirty[i] = false;
    }
  }

  if (force_grid) {
    hal_io_oled_fill_rect(0u, 67u, 160u, 28u, false);
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

  if (g_euclid.ui_clk_src_dirty || !g_euclid.ui_cache_valid || strcmp(clk_src, g_euclid.ui_clk_src) != 0) {
    oled_draw_text_26(y_clk_src, clk_src, false);
    snprintf(g_euclid.ui_clk_src, sizeof(g_euclid.ui_clk_src), "%s", clk_src);
    g_euclid.ui_clk_src_dirty = false;
  }

  if (g_euclid.ui_status_dirty || !g_euclid.ui_cache_valid || show_status != g_euclid.ui_status_visible ||
      strcmp(status, g_euclid.ui_status) != 0) {
    oled_draw_text_26(y_status, status, false);
    snprintf(g_euclid.ui_status, sizeof(g_euclid.ui_status), "%s", status);
    g_euclid.ui_status_visible = show_status;
    g_euclid.ui_status_dirty = false;
  }

  g_euclid.ui_force_full_redraw = false;
  g_euclid.ui_cache_valid = true;
}

static void app_on_clock_tick(app_mode_t active_app, uint64_t timestamp_us) {
  if (active_app == APP_GRIDS && g_grids.clock == GRIDS_CLOCK_EXT) {
    grids_on_clock_tick(timestamp_us);
  } else if (active_app == APP_TRIGSEQ && g_trigseq.clock == TRIGSEQ_CLOCK_EXT) {
    trigseq_on_clock_tick(timestamp_us);
  } else if (active_app == APP_EUCLID && g_euclid.clock == EUCLID_CLOCK_EXT) {
    euclid_on_clock_tick(timestamp_us);
  } else if (active_app == APP_TR2GATE) {
    tr2gate_on_source_edge_us(0u, timestamp_us);
  } else if (active_app == APP_TR2ADSR) {
    tr2adsr_on_source_edge_us(0u, timestamp_us);
  } else if (active_app == APP_BURSTGEN) {
    burstgen_start_channel(0u, timestamp_us);
  }
}

static void app_on_trigger_edge(app_mode_t active_app, uint8_t source, uint64_t timestamp_us) {
  if (active_app == APP_TR2GATE) {
    tr2gate_on_source_edge_us(source, timestamp_us);
  } else if (active_app == APP_TR2ADSR) {
    tr2adsr_on_source_edge_us(source, timestamp_us);
  } else if (active_app == APP_BURSTGEN) {
    burstgen_start_channel(source, timestamp_us);
  }
}

static void service_clock_input_events(void) {
  clock_event_t ev;
  trigger_edge_event_t trig_ev;
  while (clock_input_pop_event(&ev)) {
    uint64_t now_us = time_us_64();
    uint64_t latency_us = now_us > ev.timestamp_us ? now_us - ev.timestamp_us : 0u;
    if (latency_us > g_max_clock_event_latency_us) {
      g_max_clock_event_latency_us = latency_us > UINT32_MAX ? UINT32_MAX : (uint32_t)latency_us;
    }
    app_on_clock_tick(g_app_mode, ev.timestamp_us);
  }

  while (clock_input_pop_trigger_edge(&trig_ev)) {
    app_on_trigger_edge(g_app_mode, trig_ev.source, trig_ev.timestamp_us);
  }
}

static void service_active_app_pulses(uint64_t now_us) {
  (void)trigger_engine_service_timer();
  if (g_app_mode == APP_GRIDS) {
    grids_update_pulses(now_us);
  } else if (g_app_mode == APP_TRIGSEQ) {
    trigseq_update_pulses(now_us);
  } else if (g_app_mode == APP_EUCLID) {
    euclid_update_pulses(now_us);
  } else if (g_app_mode == APP_BURSTGEN) {
    burstgen_update_all(now_us);
  } else if (g_app_mode == APP_TR2GATE) {
    trigger_output_set_unipolar_levels(g_tr2gate.level_10v ? 10000 : 5000);
    g_tr2gate.outputs_ok = trigger_engine_update(now_us);
    tr2gate_sync_gate_out_from_engine();
  }
}

static bool active_app_has_live_pulse(void) {
  if (g_app_mode == APP_GRIDS || g_app_mode == APP_TRIGSEQ || g_app_mode == APP_EUCLID) {
    return trigger_engine_any_active();
  }
  return false;
}

static bool active_app_timing_priority_block_draw(uint64_t now_us) {
  const uint64_t tick_guard_us = 4000u;

  if (active_app_has_live_pulse()) return true;

  if (g_app_mode == APP_GRIDS) {
    if (g_grids.clock == GRIDS_CLOCK_INT && g_grids.next_int_tick_us <= (now_us + tick_guard_us)) {
      return true;
    }
  } else if (g_app_mode == APP_TRIGSEQ) {
    if (g_trigseq.clock == TRIGSEQ_CLOCK_INT && g_trigseq.next_int_tick_us <= (now_us + tick_guard_us)) {
      return true;
    }
  } else if (g_app_mode == APP_EUCLID) {
    if (g_euclid.clock == EUCLID_CLOCK_INT && g_euclid.next_int_tick_us <= (now_us + tick_guard_us)) {
      return true;
    }
  } else if (g_app_mode == APP_BURSTGEN) {
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
      if (g_burstgen.running[ch] && g_burstgen.next_step_at_us[ch] <= (now_us + tick_guard_us)) {
        return true;
      }
    }
  }

  return false;
}

static void maybe_print_timing_diag(uint64_t now_ms) {
#if ENABLE_TIMING_DIAG_PRINT
  static uint64_t last_print_ms = 0u;
  if ((now_ms - last_print_ms) < TIMING_DIAG_PRINT_PERIOD_MS) return;
  last_print_ms = now_ms;

  clock_input_diag_t clock_diag;
  trigger_output_diag_t trigger_diag;
  hal_io_dac_diag_t dac_diag;
  clock_input_get_diag(&clock_diag);
  trigger_output_get_diag(&trigger_diag);
  hal_io_dac_get_diag(&dac_diag);
  printf("timing clk_irq=%lu q_ovf=%lu ign=%lu bpm=%u.%u clk_lat_max_us=%lu trig_lat_max_us=%lu timer_due=%lu dac_w=%lu dac_ch=%lu dac_skip=%lu oled_max_us=%lu burst_late=%lu burst_miss=%lu burst_cap=%lu\n",
         (unsigned long)clock_diag.irq_count,
         (unsigned long)clock_diag.clock_queue_overflow,
         (unsigned long)clock_diag.ignored_clock_edges,
         (unsigned)(clock_diag.measured_bpm_x10 / 10u),
         (unsigned)(clock_diag.measured_bpm_x10 % 10u),
         (unsigned long)g_max_clock_event_latency_us,
         (unsigned long)trigger_diag.max_trigger_fire_latency_us,
         (unsigned long)trigger_diag.timer_due_count,
         (unsigned long)dac_diag.write_calls,
         (unsigned long)dac_diag.channel_writes,
         (unsigned long)(dac_diag.skipped_write_calls + dac_diag.skipped_channel_writes),
         (unsigned long)g_oled_frame_time_max_us,
         (unsigned long)g_burstgen.late_step_count,
         (unsigned long)g_burstgen.missed_step_count,
         (unsigned long)g_burstgen.catchup_limit_count);
#else
  (void)now_ms;
#endif
}

int main(void) {
  uint64_t now_ms;
  uint64_t now_us;
  uint64_t last_draw_ms = 0;

  stdio_init_all();
  sleep_ms(1200);

  hal_io_init();
  clock_input_init(CLOCK_INPUT_MIN_INTERVAL_US);
  trigger_output_init(trigger_output_write_mv_cb, NULL);
  (void)trigger_output_start_timer(-TRIGGER_OUTPUT_TIMER_US);
  hal_mux_adc_init();
  hal_io_oled_clear();

  g_calibration_loaded = calibration_init(&g_calibration_data);
  g_app_settings_loaded = app_settings_init(&g_app_settings_data);
  app_presets_init();
  load_runtime_from_app_settings();
  (void)app_write_outputs_mv((int32_t[4]){0, 0, 0, 0});

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
    now_us = time_us_64();
    hal_io_poll(now_ms);
    service_clock_input_events();
    service_active_app_pulses(now_us);
    maybe_print_timing_diag(now_ms);

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
    } else if (g_app_mode == APP_CALIBRATION) {
      update_calibration(d_l, d_r, edge_enc_r);
      (void)edge_sw1;
      (void)edge_sw2;
    } else if (g_app_mode == APP_NOTES) {
      update_notes(d_l, d_r);
      (void)edge_enc_r;
      (void)edge_sw1;
      (void)edge_sw2;
    } else if (g_app_mode == APP_VOLTS) {
      update_volts(d_l, d_r);
      (void)edge_enc_r;
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
      uint64_t draw_now_us = time_us_64();
      service_active_app_pulses(draw_now_us);

      // Sequencer timing has higher priority than UI rendering.
      // If a pulse is active or an internal tick is imminent, skip draw and
      // return quickly to the timing update path.
      if (active_app_timing_priority_block_draw(draw_now_us)) {
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
        uint64_t draw_start_us = time_us_64();
        if (preset_ui_is_preset_screen()) {
          draw_active_app_preset_screen();
        } else if (g_app_mode == APP_CALIBRATION) {
          draw_calibration();
        } else if (g_app_mode == APP_NOTES) {
          draw_notes();
        } else if (g_app_mode == APP_VOLTS) {
          draw_volts();
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
        {
          uint64_t draw_elapsed_us = time_us_64() - draw_start_us;
          if (draw_elapsed_us > g_oled_frame_time_max_us) {
            g_oled_frame_time_max_us =
                draw_elapsed_us > UINT32_MAX ? UINT32_MAX : (uint32_t)draw_elapsed_us;
          }
        }
      }
      last_draw_ms = now_ms;

      // Rendering can take noticeable time. Service pulse timeouts again right after draw
      // so output pulse width is not stretched by OLED refresh time.
      service_active_app_pulses(time_us_64());
    }

    sleep_ms(LOOP_SLEEP_MS);
    tight_loop_contents();
  }
}
