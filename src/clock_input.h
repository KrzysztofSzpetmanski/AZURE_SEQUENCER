#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint64_t timestamp_us;
} clock_event_t;

typedef struct {
  uint8_t source;
  uint64_t timestamp_us;
} trigger_edge_event_t;

typedef struct {
  uint32_t irq_count;
  uint32_t accepted_edges;
  uint32_t ignored_clock_edges;
  uint32_t clock_queue_overflow;
  uint32_t last_clock_interval_us;
  uint32_t min_clock_interval_us;
  uint32_t max_clock_interval_us;
  uint32_t avg_clock_interval_us;
  uint16_t measured_bpm_x10;
  uint8_t queued_events;
} clock_input_diag_t;

void clock_input_init(uint32_t min_interval_us);
bool clock_input_pop_event(clock_event_t* ev);
bool clock_input_pop_trigger_edge(trigger_edge_event_t* ev);
void clock_input_get_diag(clock_input_diag_t* diag);

#ifdef __cplusplus
}
#endif
