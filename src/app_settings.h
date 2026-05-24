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
} grids_settings_t;

typedef struct {
  uint8_t length;
  uint8_t edit_channel;
  uint8_t edit_step;
  uint8_t run;
  uint32_t pattern[4];
} trigseq_settings_t;

typedef struct {
  grids_settings_t grids;
  trigseq_settings_t trigseq;
} app_settings_data_t;

void app_settings_set_defaults(app_settings_data_t* data);
bool app_settings_init(app_settings_data_t* data);
bool app_settings_save(const app_settings_data_t* data);

#ifdef __cplusplus
}
#endif

#endif
