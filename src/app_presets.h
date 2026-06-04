#ifndef APP_PRESETS_H
#define APP_PRESETS_H

#include <stdbool.h>
#include <stdint.h>

#include "app_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_PRESET_SLOTS 8u

bool app_presets_init(void);

bool app_presets_grids_save(uint8_t slot, const grids_settings_t* preset);
bool app_presets_grids_load(uint8_t slot, grids_settings_t* out_preset);
bool app_presets_grids_load_last(grids_settings_t* out_preset, uint8_t* out_slot);
bool app_presets_grids_slot_used(uint8_t slot);

bool app_presets_trigseq_save(uint8_t slot, const trigseq_settings_t* preset);
bool app_presets_trigseq_load(uint8_t slot, trigseq_settings_t* out_preset);
bool app_presets_trigseq_load_last(trigseq_settings_t* out_preset, uint8_t* out_slot);
bool app_presets_trigseq_slot_used(uint8_t slot);

bool app_presets_euclid_save(uint8_t slot, const euclid_settings_t* preset);
bool app_presets_euclid_load(uint8_t slot, euclid_settings_t* out_preset);
bool app_presets_euclid_load_last(euclid_settings_t* out_preset, uint8_t* out_slot);
bool app_presets_euclid_slot_used(uint8_t slot);

bool app_presets_tr2gate_save(uint8_t slot, const tr2gate_settings_t* preset);
bool app_presets_tr2gate_load(uint8_t slot, tr2gate_settings_t* out_preset);
bool app_presets_tr2gate_load_last(tr2gate_settings_t* out_preset, uint8_t* out_slot);
bool app_presets_tr2gate_slot_used(uint8_t slot);

bool app_presets_tr2adsr_save(uint8_t slot, const tr2adsr_settings_t* preset);
bool app_presets_tr2adsr_load(uint8_t slot, tr2adsr_settings_t* out_preset);
bool app_presets_tr2adsr_load_last(tr2adsr_settings_t* out_preset, uint8_t* out_slot);
bool app_presets_tr2adsr_slot_used(uint8_t slot);

bool app_presets_burstgen_save(uint8_t slot, const burstgen_settings_t* preset);
bool app_presets_burstgen_load(uint8_t slot, burstgen_settings_t* out_preset);
bool app_presets_burstgen_load_last(burstgen_settings_t* out_preset, uint8_t* out_slot);
bool app_presets_burstgen_slot_used(uint8_t slot);

bool app_presets_cvgen_save(uint8_t slot, const cvgen_settings_t* preset);
bool app_presets_cvgen_load(uint8_t slot, cvgen_settings_t* out_preset);
bool app_presets_cvgen_load_last(cvgen_settings_t* out_preset, uint8_t* out_slot);
bool app_presets_cvgen_slot_used(uint8_t slot);

#ifdef __cplusplus
}
#endif

#endif
