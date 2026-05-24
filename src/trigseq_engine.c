#include "trigseq_engine.h"

#include <stddef.h>

static uint8_t clamp_len(uint8_t len) {
  if (len < 4u) return 4u;
  if (len > 32u) return 32u;
  return len;
}

static uint32_t rng32(trigseq_engine_t* s) {
  uint32_t x = s->rng_state;
  if (x == 0u) x = 0x9E3779B9u;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  s->rng_state = x;
  return x;
}

void trigseq_engine_init(trigseq_engine_t* s, uint32_t seed) {
  if (s == NULL) return;
  s->rng_state = (seed == 0u) ? 0xC001D00Du : seed;
  s->length = 16u;
  s->step = -1;
  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    s->pattern[ch] = rng32(s);
    if (s->pattern[ch] == 0u) {
      s->pattern[ch] = (1u << (ch + 1u));
    }
  }
}

void trigseq_engine_reset(trigseq_engine_t* s) {
  if (s == NULL) return;
  s->step = -1;
}

void trigseq_engine_set_length(trigseq_engine_t* s, uint8_t length_4_to_32) {
  uint8_t len;
  if (s == NULL) return;
  len = clamp_len(length_4_to_32);
  s->length = len;
  if (s->step >= (int8_t)len) {
    s->step = (int8_t)(len - 1u);
  }
}

uint8_t trigseq_engine_get_length(const trigseq_engine_t* s) {
  if (s == NULL) return 16u;
  return s->length;
}

bool trigseq_engine_get_step_bit(const trigseq_engine_t* s, uint8_t ch0_to_3, uint8_t step0_to_31) {
  uint8_t ch;
  uint8_t step;
  if (s == NULL) return false;
  ch = (uint8_t)(ch0_to_3 & 0x03u);
  step = (uint8_t)(step0_to_31 & 0x1Fu);
  return ((s->pattern[ch] >> step) & 0x01u) != 0u;
}

void trigseq_engine_set_step_bit(trigseq_engine_t* s, uint8_t ch0_to_3, uint8_t step0_to_31, bool on) {
  uint8_t ch;
  uint8_t step;
  uint32_t mask;
  if (s == NULL) return;
  ch = (uint8_t)(ch0_to_3 & 0x03u);
  step = (uint8_t)(step0_to_31 & 0x1Fu);
  mask = (uint32_t)1u << step;
  if (on) {
    s->pattern[ch] |= mask;
  } else {
    s->pattern[ch] &= ~mask;
  }
}

void trigseq_engine_clock(trigseq_engine_t* s, bool trig_out_4[4]) {
  if (s == NULL || trig_out_4 == NULL) return;

  if (s->step >= (int8_t)(s->length - 1u)) {
    s->step = -1;
  }
  s->step += 1;

  for (uint8_t ch = 0u; ch < 4u; ++ch) {
    trig_out_4[ch] = trigseq_engine_get_step_bit(s, ch, (uint8_t)s->step);
  }
}
