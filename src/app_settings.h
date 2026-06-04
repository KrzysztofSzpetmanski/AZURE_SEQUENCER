#ifndef APP_SETTINGS_H
#define APP_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t clock_mode; /* 0=INT, 1=EXT */
  uint16_t bpm;
  uint8_t map_x;
  uint8_t map_y;
  uint8_t chaos;
  uint8_t prob[4]; /* 0..100% */
} grids_settings_t;

typedef struct {
  uint8_t length;
  uint8_t edit_channel;
  uint8_t edit_step;
  uint8_t clock_mode; /* 0=INT, 1=EXT */
  uint16_t bpm;
  uint8_t run;
  uint8_t prob[4]; /* 0..100% */
  uint64_t pattern[4];
} trigseq_settings_t;

typedef struct {
  uint8_t clock_mode; /* 0=INT, 1=EXT */
  uint16_t bpm;
  uint8_t steps[4];
  uint8_t hits[4];
  uint8_t prob[4]; /* 0..100% */
} euclid_settings_t;

typedef struct {
  uint8_t src[4]; /* 0..3 => TR1..TR4 */
  uint16_t gate_time_cs[4]; /* 0.01s units */
  uint8_t prob[4]; /* 0..100% */
  uint8_t level_10v; /* 0=5V, 1=10V */
} tr2gate_settings_t;

typedef struct {
  uint8_t src[4]; /* 0..3 => TR1..TR4 */
  uint8_t type[4]; /* 0=AR, 1=ASR, 2=ADSR */
  uint16_t total_time_ds[4]; /* 0.1s units */
  uint8_t prob[4]; /* 0..100% */
  uint8_t level_10v; /* 0=5V, 1=10V */
  uint8_t a_pct[4];
  uint8_t d_pct[4];
  uint8_t s_pct[4];
  uint8_t r_pct[4];
} tr2adsr_settings_t;

typedef struct {
  uint8_t selected_channel; /* 0..3 */
  uint8_t signature_mode; /* 0=4/4, 1=6/6, 2=8/8 */
  uint16_t bpm;
  uint8_t swing_pct; /* 0..40 */
  uint8_t probability; /* 0..100 */
  uint8_t level_10v; /* 0=5V, 1=10V */
} burstgen_settings_t;

typedef struct {
  uint8_t algo[4]; /* 0=STEP, 1=BEZIER, 2=OCEAN, 3=WALK */
  uint8_t clock_mode[4]; /* 0=INT, 1=EXT */
  uint8_t src[4]; /* 0..3 => TR1..TR4 */
  uint16_t rate_dhz[4]; /* 0.1Hz units */
  int16_t min_mv[4];
  int16_t max_mv[4];
  uint8_t param1[4]; /* algorithm-specific */
  uint8_t param2[4]; /* algorithm-specific */
} cvgen_settings_t;

typedef struct {
  grids_settings_t grids;
  trigseq_settings_t trigseq;
  euclid_settings_t euclid;
  tr2gate_settings_t tr2gate;
  tr2adsr_settings_t tr2adsr;
  burstgen_settings_t burstgen;
  cvgen_settings_t cvgen;
} app_settings_data_t;

void app_settings_set_defaults(app_settings_data_t* data);
bool app_settings_init(app_settings_data_t* data);
bool app_settings_save(const app_settings_data_t* data);

#ifdef __cplusplus
}
#endif

#endif
