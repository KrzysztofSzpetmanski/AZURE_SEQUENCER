#include "clock_input.h"

#include <stddef.h>

#include "hal_pins_azure.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define CLOCK_INPUT_QUEUE_SIZE 16u
#define CLOCK_INPUT_QUEUE_MASK (CLOCK_INPUT_QUEUE_SIZE - 1u)
#define TRIGGER_EDGE_QUEUE_SIZE 16u
#define TRIGGER_EDGE_QUEUE_MASK (TRIGGER_EDGE_QUEUE_SIZE - 1u)
#define CLOCK_INPUT_DEFAULT_MIN_INTERVAL_US 500u

static volatile clock_event_t g_clock_queue[CLOCK_INPUT_QUEUE_SIZE];
static volatile uint8_t g_clock_head = 0u;
static volatile uint8_t g_clock_tail = 0u;
static volatile trigger_edge_event_t g_trigger_queue[TRIGGER_EDGE_QUEUE_SIZE];
static volatile uint8_t g_trigger_head = 0u;
static volatile uint8_t g_trigger_tail = 0u;
static volatile uint32_t g_trigger_queue_overflow = 0u;
static volatile uint32_t g_clock_irq_count = 0u;
static volatile uint32_t g_clock_accepted_edges = 0u;
static volatile uint32_t g_clock_ignored_edges = 0u;
static volatile uint32_t g_clock_queue_overflow = 0u;
static volatile uint64_t g_clock_last_edge_us = 0u;
static volatile uint32_t g_clock_min_interval_us = 0u;
static volatile uint32_t g_clock_max_interval_us = 0u;
static volatile uint64_t g_clock_interval_sum_us = 0u;
static volatile uint32_t g_clock_interval_count = 0u;
static volatile uint32_t g_clock_last_interval_us = 0u;
static uint32_t g_clock_min_interval_limit_us = CLOCK_INPUT_DEFAULT_MIN_INTERVAL_US;

static int8_t trigger_source_from_gpio(uint gpio) {
  if (gpio == HRDW_PIN_TR2_IN) return 1;
  if (gpio == HRDW_PIN_TR3_IN) return 2;
  if (gpio == HRDW_PIN_TR4_IN) return 3;
  return -1;
}

static void push_trigger_edge(uint8_t source, uint64_t timestamp_us) {
  uint8_t next_head = (uint8_t)((g_trigger_head + 1u) & TRIGGER_EDGE_QUEUE_MASK);
  if (next_head == g_trigger_tail) {
    g_trigger_queue_overflow += 1u;
    return;
  }
  g_trigger_queue[g_trigger_head].source = source;
  g_trigger_queue[g_trigger_head].timestamp_us = timestamp_us;
  g_trigger_head = next_head;
}

static void clock_input_gpio_irq(uint gpio, uint32_t events) {
  uint64_t now_us;
  uint64_t prev_us;
  uint32_t interval_us;
  uint8_t next_head;
  int8_t trigger_source;

  if ((events & GPIO_IRQ_EDGE_FALL) == 0u) return;

  trigger_source = trigger_source_from_gpio(gpio);
  if (trigger_source >= 0) {
    push_trigger_edge((uint8_t)trigger_source, time_us_64());
    return;
  }

  if (gpio != HRDW_PIN_TR1_IN) return;

  now_us = time_us_64();
  prev_us = g_clock_last_edge_us;
  g_clock_irq_count += 1u;

  if (prev_us != 0u) {
    uint64_t delta = now_us - prev_us;
    interval_us = delta > UINT32_MAX ? UINT32_MAX : (uint32_t)delta;
    if (interval_us < g_clock_min_interval_limit_us) {
      g_clock_ignored_edges += 1u;
      return;
    }
    g_clock_last_interval_us = interval_us;
    if (g_clock_min_interval_us == 0u || interval_us < g_clock_min_interval_us) {
      g_clock_min_interval_us = interval_us;
    }
    if (interval_us > g_clock_max_interval_us) {
      g_clock_max_interval_us = interval_us;
    }
    g_clock_interval_sum_us += interval_us;
    g_clock_interval_count += 1u;
  }

  g_clock_last_edge_us = now_us;
  next_head = (uint8_t)((g_clock_head + 1u) & CLOCK_INPUT_QUEUE_MASK);
  if (next_head == g_clock_tail) {
    g_clock_queue_overflow += 1u;
    return;
  }

  g_clock_queue[g_clock_head].timestamp_us = now_us;
  g_clock_head = next_head;
  g_clock_accepted_edges += 1u;
}

void clock_input_init(uint32_t min_interval_us) {
  uint32_t irq_state = save_and_disable_interrupts();

  g_clock_head = 0u;
  g_clock_tail = 0u;
  g_trigger_head = 0u;
  g_trigger_tail = 0u;
  g_trigger_queue_overflow = 0u;
  g_clock_irq_count = 0u;
  g_clock_accepted_edges = 0u;
  g_clock_ignored_edges = 0u;
  g_clock_queue_overflow = 0u;
  g_clock_last_edge_us = 0u;
  g_clock_min_interval_us = 0u;
  g_clock_max_interval_us = 0u;
  g_clock_interval_sum_us = 0u;
  g_clock_interval_count = 0u;
  g_clock_last_interval_us = 0u;
  g_clock_min_interval_limit_us = min_interval_us == 0u ? CLOCK_INPUT_DEFAULT_MIN_INTERVAL_US : min_interval_us;

  restore_interrupts(irq_state);

  gpio_set_irq_enabled_with_callback(HRDW_PIN_TR1_IN, GPIO_IRQ_EDGE_FALL, true, clock_input_gpio_irq);
  gpio_set_irq_enabled(HRDW_PIN_TR2_IN, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(HRDW_PIN_TR3_IN, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(HRDW_PIN_TR4_IN, GPIO_IRQ_EDGE_FALL, true);
}

bool clock_input_pop_event(clock_event_t* ev) {
  uint32_t irq_state;

  if (ev == NULL) return false;

  irq_state = save_and_disable_interrupts();
  if (g_clock_tail == g_clock_head) {
    restore_interrupts(irq_state);
    return false;
  }

  ev->timestamp_us = g_clock_queue[g_clock_tail].timestamp_us;
  g_clock_tail = (uint8_t)((g_clock_tail + 1u) & CLOCK_INPUT_QUEUE_MASK);
  restore_interrupts(irq_state);
  return true;
}

bool clock_input_pop_trigger_edge(trigger_edge_event_t* ev) {
  uint32_t irq_state;

  if (ev == NULL) return false;

  irq_state = save_and_disable_interrupts();
  if (g_trigger_tail == g_trigger_head) {
    restore_interrupts(irq_state);
    return false;
  }

  ev->source = g_trigger_queue[g_trigger_tail].source;
  ev->timestamp_us = g_trigger_queue[g_trigger_tail].timestamp_us;
  g_trigger_tail = (uint8_t)((g_trigger_tail + 1u) & TRIGGER_EDGE_QUEUE_MASK);
  restore_interrupts(irq_state);
  return true;
}

void clock_input_get_diag(clock_input_diag_t* diag) {
  uint32_t irq_state;
  uint64_t sum;
  uint32_t count;
  uint32_t avg = 0u;
  uint32_t last_interval;
  uint8_t head;
  uint8_t tail;

  if (diag == NULL) return;

  irq_state = save_and_disable_interrupts();
  sum = g_clock_interval_sum_us;
  count = g_clock_interval_count;
  last_interval = g_clock_last_interval_us;
  head = g_clock_head;
  tail = g_clock_tail;

  diag->irq_count = g_clock_irq_count;
  diag->accepted_edges = g_clock_accepted_edges;
  diag->ignored_clock_edges = g_clock_ignored_edges;
  diag->clock_queue_overflow = g_clock_queue_overflow;
  diag->last_clock_interval_us = last_interval;
  diag->min_clock_interval_us = g_clock_min_interval_us;
  diag->max_clock_interval_us = g_clock_max_interval_us;
  restore_interrupts(irq_state);

  if (count > 0u) {
    uint64_t avg64 = sum / count;
    avg = avg64 > UINT32_MAX ? UINT32_MAX : (uint32_t)avg64;
  }
  diag->avg_clock_interval_us = avg;
  diag->measured_bpm_x10 = last_interval == 0u ? 0u : (uint16_t)(600000000ull / (uint64_t)last_interval);
  diag->queued_events = (uint8_t)((head - tail) & CLOCK_INPUT_QUEUE_MASK);
}
