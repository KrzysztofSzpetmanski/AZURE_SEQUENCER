#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t pattern[4];
  uint8_t length;
  int8_t step;
  uint32_t rng_state;
} trigseq_engine_t;

void trigseq_engine_init(trigseq_engine_t* s, uint32_t seed);
void trigseq_engine_reset(trigseq_engine_t* s);
void trigseq_engine_set_length(trigseq_engine_t* s, uint8_t length_4_to_32);
uint8_t trigseq_engine_get_length(const trigseq_engine_t* s);

bool trigseq_engine_get_step_bit(const trigseq_engine_t* s, uint8_t ch0_to_3, uint8_t step0_to_31);
void trigseq_engine_set_step_bit(trigseq_engine_t* s, uint8_t ch0_to_3, uint8_t step0_to_31, bool on);

/* Advance by one clock and return triggers for current step. */
void trigseq_engine_clock(trigseq_engine_t* s, bool trig_out_4[4]);

#ifdef __cplusplus
}
#endif
