#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*trigger_output_write_mv_fn)(const int32_t millivolts_4[4], void* user_data);

typedef struct {
  uint32_t fire_count;
  uint32_t update_count;
  uint32_t timer_due_count;
  uint32_t write_count;
  uint32_t skipped_write_count;
  uint32_t max_trigger_fire_latency_us;
  uint8_t active_mask;
} trigger_output_diag_t;

void trigger_output_init(trigger_output_write_mv_fn write_fn, void* user_data);
bool trigger_output_start_timer(int64_t interval_us);
void trigger_output_set_levels_mv(const int32_t high_mv_4[4], const int32_t low_mv_4[4]);
bool trigger_engine_fire_mask(uint8_t output_mask, uint64_t timestamp_us, uint32_t pulse_width_us);
bool trigger_engine_fire_mask_widths(uint8_t output_mask, uint64_t timestamp_us, const uint32_t pulse_width_us_4[4]);
bool trigger_engine_fire_mask_events(uint8_t output_mask, const uint64_t timestamp_us_4[4],
                                     const uint32_t pulse_width_us_4[4]);
bool trigger_engine_update(uint64_t now_us);
bool trigger_engine_service_timer(void);
bool trigger_engine_force_all_low(void);
bool trigger_engine_any_active(void);
uint8_t trigger_engine_active_mask(void);
void trigger_output_get_diag(trigger_output_diag_t* diag);

#ifdef __cplusplus
}
#endif
