#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t step;
  uint8_t part_perturbation[3];
  uint32_t rng_state;
} grids_engine_t;

void grids_engine_init(grids_engine_t* g, uint32_t seed);
void grids_engine_reset(grids_engine_t* g);

/*
 * Generate one Grids step.
 * - map_x/map_y/chaos/fill are 0..255
 * - trig_out[0..3] returns trigger flags for channels 1..4
 */
void grids_engine_step(grids_engine_t* g,
                       uint8_t map_x,
                       uint8_t map_y,
                       uint8_t chaos,
                       const uint8_t fill[4],
                       bool trig_out[4]);

#ifdef __cplusplus
}
#endif
