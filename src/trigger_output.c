#include "trigger_output.h"

#include <stddef.h>

#include "pico/stdlib.h"

static bool trigger_timer_cb(struct repeating_timer* timer);

static trigger_output_write_mv_fn g_trigger_write_fn = NULL;
static void* g_trigger_user_data = NULL;
static struct repeating_timer g_trigger_timer;
static volatile bool g_trigger_timer_started = false;
static volatile bool g_trigger_timer_due = false;
static int32_t g_trigger_high_mv[4] = {5000, 5000, 5000, 5000};
static int32_t g_trigger_low_mv[4] = {0, 0, 0, 0};
static uint64_t g_trigger_off_at_us[4] = {0u, 0u, 0u, 0u};
static uint8_t g_trigger_active_mask = 0u;
static uint8_t g_trigger_last_written_mask = 0xFFu;
static uint32_t g_trigger_fire_count = 0u;
static uint32_t g_trigger_update_count = 0u;
static uint32_t g_trigger_timer_due_count = 0u;
static uint32_t g_trigger_write_count = 0u;
static uint32_t g_trigger_skipped_write_count = 0u;
static uint32_t g_trigger_max_fire_latency_us = 0u;

static bool trigger_timer_cb(struct repeating_timer* timer) {
  (void)timer;
  // Timer context must stay non-blocking: no I2C/DAC writes here.
  // Foreground service observes this flag and calls trigger_engine_update().
  g_trigger_timer_due = true;
  return true;
}

static bool trigger_output_write_mask(uint8_t mask, bool force) {
  int32_t out_mv[4];

  if (g_trigger_write_fn == NULL) return false;
  if (!force && mask == g_trigger_last_written_mask) {
    g_trigger_skipped_write_count += 1u;
    return true;
  }

  for (uint8_t i = 0u; i < 4u; ++i) {
    out_mv[i] = (mask & (uint8_t)(1u << i)) != 0u ? g_trigger_high_mv[i] : g_trigger_low_mv[i];
  }

  if (g_trigger_write_fn(out_mv, g_trigger_user_data)) {
    g_trigger_last_written_mask = mask;
    g_trigger_write_count += 1u;
    return true;
  }

  return false;
}

void trigger_output_init(trigger_output_write_mv_fn write_fn, void* user_data) {
  g_trigger_write_fn = write_fn;
  g_trigger_user_data = user_data;
  g_trigger_active_mask = 0u;
  g_trigger_last_written_mask = 0xFFu;
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_trigger_off_at_us[i] = 0u;
    g_trigger_high_mv[i] = 5000;
    g_trigger_low_mv[i] = 0;
  }
  g_trigger_fire_count = 0u;
  g_trigger_update_count = 0u;
  g_trigger_timer_due_count = 0u;
  g_trigger_write_count = 0u;
  g_trigger_skipped_write_count = 0u;
  g_trigger_max_fire_latency_us = 0u;
}

bool trigger_output_start_timer(int64_t interval_us) {
  if (interval_us == 0) return false;
  if (g_trigger_timer_started) return true;
  if (add_repeating_timer_us(interval_us, trigger_timer_cb, NULL, &g_trigger_timer)) {
    g_trigger_timer_started = true;
    return true;
  }
  return false;
}

void trigger_output_set_levels_mv(const int32_t high_mv_4[4], const int32_t low_mv_4[4]) {
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_trigger_high_mv[i] = high_mv_4 == NULL ? 5000 : high_mv_4[i];
    g_trigger_low_mv[i] = low_mv_4 == NULL ? 0 : low_mv_4[i];
  }
  g_trigger_last_written_mask = 0xFFu;
}

bool trigger_engine_fire_mask(uint8_t output_mask, uint64_t timestamp_us, uint32_t pulse_width_us) {
  uint32_t widths[4] = {pulse_width_us, pulse_width_us, pulse_width_us, pulse_width_us};
  return trigger_engine_fire_mask_widths(output_mask, timestamp_us, widths);
}

bool trigger_engine_fire_mask_widths(uint8_t output_mask, uint64_t timestamp_us, const uint32_t pulse_width_us_4[4]) {
  uint64_t timestamps[4] = {timestamp_us, timestamp_us, timestamp_us, timestamp_us};
  return trigger_engine_fire_mask_events(output_mask, timestamps, pulse_width_us_4);
}

bool trigger_engine_fire_mask_events(uint8_t output_mask, const uint64_t timestamp_us_4[4],
                                     const uint32_t pulse_width_us_4[4]) {
  uint64_t now_us;

  output_mask &= 0x0Fu;
  if (output_mask == 0u || timestamp_us_4 == NULL || pulse_width_us_4 == NULL) return true;

  now_us = time_us_64();
  for (uint8_t i = 0u; i < 4u; ++i) {
    if ((output_mask & (uint8_t)(1u << i)) != 0u) {
      if (pulse_width_us_4[i] == 0u) {
        output_mask &= (uint8_t)~(uint8_t)(1u << i);
      } else {
        uint64_t latency_us = now_us > timestamp_us_4[i] ? now_us - timestamp_us_4[i] : 0u;
        g_trigger_off_at_us[i] = timestamp_us_4[i] + (uint64_t)pulse_width_us_4[i];
        if (latency_us > g_trigger_max_fire_latency_us) {
          g_trigger_max_fire_latency_us = latency_us > UINT32_MAX ? UINT32_MAX : (uint32_t)latency_us;
        }
      }
    }
  }
  if (output_mask == 0u) return true;
  g_trigger_active_mask |= output_mask;
  g_trigger_fire_count += 1u;

  return trigger_output_write_mask(g_trigger_active_mask, false);
}

bool trigger_engine_update(uint64_t now_us) {
  uint8_t next_mask = g_trigger_active_mask;

  g_trigger_update_count += 1u;
  if (next_mask == 0u) return true;

  for (uint8_t i = 0u; i < 4u; ++i) {
    uint8_t bit = (uint8_t)(1u << i);
    if ((next_mask & bit) != 0u && now_us >= g_trigger_off_at_us[i]) {
      next_mask &= (uint8_t)~bit;
      g_trigger_off_at_us[i] = 0u;
    }
  }

  if (next_mask == g_trigger_active_mask) return true;
  g_trigger_active_mask = next_mask;
  return trigger_output_write_mask(g_trigger_active_mask, false);
}

bool trigger_engine_service_timer(void) {
  if (!g_trigger_timer_due) return true;
  g_trigger_timer_due = false;
  g_trigger_timer_due_count += 1u;
  return trigger_engine_update(time_us_64());
}

bool trigger_engine_force_all_low(void) {
  for (uint8_t i = 0u; i < 4u; ++i) {
    g_trigger_off_at_us[i] = 0u;
  }
  g_trigger_active_mask = 0u;
  return trigger_output_write_mask(0u, true);
}

bool trigger_engine_any_active(void) {
  return g_trigger_active_mask != 0u;
}

uint8_t trigger_engine_active_mask(void) {
  return g_trigger_active_mask;
}

void trigger_output_get_diag(trigger_output_diag_t* diag) {
  if (diag == NULL) return;
  diag->fire_count = g_trigger_fire_count;
  diag->update_count = g_trigger_update_count;
  diag->timer_due_count = g_trigger_timer_due_count;
  diag->write_count = g_trigger_write_count;
  diag->skipped_write_count = g_trigger_skipped_write_count;
  diag->max_trigger_fire_latency_us = g_trigger_max_fire_latency_us;
  diag->active_mask = g_trigger_active_mask;
}
