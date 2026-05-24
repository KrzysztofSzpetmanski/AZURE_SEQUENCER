#include "grids_engine.h"

#include <stddef.h>

#include "grids_resources.h"

#define GRIDS_STEPS_PER_PATTERN 32u

static const uint8_t* k_drum_map[5][5] = {
    {node_10, node_8, node_0, node_9, node_11},
    {node_15, node_7, node_13, node_12, node_6},
    {node_18, node_14, node_4, node_5, node_3},
    {node_23, node_16, node_21, node_1, node_2},
    {node_24, node_19, node_17, node_20, node_22},
};

static uint8_t u8_mix(uint8_t a, uint8_t b, uint8_t balance) {
  return (uint8_t)((((uint16_t)a * (uint16_t)(255u - balance)) + ((uint16_t)b * (uint16_t)balance)) / 255u);
}

static uint8_t rng8(grids_engine_t* g) {
  uint32_t x = g->rng_state;
  if (x == 0u) x = 0x6D2B79F5u;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  g->rng_state = x;
  return (uint8_t)(x >> 24);
}

static uint8_t read_drum_map(uint8_t step, uint8_t instrument, uint8_t x, uint8_t y) {
  uint8_t i = x >> 6u;
  uint8_t j = y >> 6u;
  const uint8_t* a_map = k_drum_map[i][j];
  const uint8_t* b_map = k_drum_map[i + 1u][j];
  const uint8_t* c_map = k_drum_map[i][j + 1u];
  const uint8_t* d_map = k_drum_map[i + 1u][j + 1u];
  uint8_t offset = (uint8_t)(instrument * GRIDS_STEPS_PER_PATTERN + step);
  uint8_t a = a_map[offset];
  uint8_t b = b_map[offset];
  uint8_t c = c_map[offset];
  uint8_t d = d_map[offset];
  return u8_mix(u8_mix(a, b, (uint8_t)(x << 2u)), u8_mix(c, d, (uint8_t)(x << 2u)), (uint8_t)(y << 2u));
}

void grids_engine_init(grids_engine_t* g, uint32_t seed) {
  if (g == NULL) return;
  g->step = 0u;
  g->part_perturbation[0] = 0u;
  g->part_perturbation[1] = 0u;
  g->part_perturbation[2] = 0u;
  g->rng_state = (seed == 0u) ? 0xA511E9B3u : seed;
}

void grids_engine_reset(grids_engine_t* g) {
  if (g == NULL) return;
  g->step = 0u;
}

void grids_engine_step(grids_engine_t* g,
                       uint8_t map_x,
                       uint8_t map_y,
                       uint8_t chaos,
                       const uint8_t fill[4],
                       bool trig_out[4]) {
  uint8_t hh_level = 0u;

  if (g == NULL || fill == NULL || trig_out == NULL) return;

  trig_out[0] = false;
  trig_out[1] = false;
  trig_out[2] = false;
  trig_out[3] = false;

  if (g->step == 0u) {
    uint8_t randomness = chaos >> 2u;
    for (uint8_t i = 0u; i < 3u; ++i) {
      uint8_t r = rng8(g);
      g->part_perturbation[i] = (uint8_t)(((uint16_t)r * (uint16_t)randomness) >> 8u);
    }
  }

  for (uint8_t i = 0u; i < 3u; ++i) {
    uint8_t level = read_drum_map(g->step, i, map_x, map_y);
    uint8_t perturb = g->part_perturbation[i];
    uint8_t threshold = (uint8_t)(255u - fill[i]);

    if (level < (uint8_t)(255u - perturb)) {
      level = (uint8_t)(level + perturb);
    } else {
      level = 255u;
    }

    if (i == 2u) {
      hh_level = level;
    }

    trig_out[i] = (level > threshold);
  }

  trig_out[3] = (hh_level > (uint8_t)(255u - fill[3]));

  g->step = (uint8_t)(g->step + 1u);
  if (g->step >= GRIDS_STEPS_PER_PATTERN) {
    g->step = 0u;
  }
}
