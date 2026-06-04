#include "app_presets.h"

#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define APP_PRESETS_FLASH_MAGIC 0x41505052u /* APPR */
#define APP_PRESETS_FLASH_VERSION 1u
#define APP_PRESETS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - (3u * FLASH_SECTOR_SIZE))
#define APP_PRESET_LAST_SLOT_NONE 0xFFu

typedef struct {
  uint8_t used_mask;
  uint8_t last_slot;
  uint8_t reserved[2];
  grids_settings_t slots[APP_PRESET_SLOTS];
} grids_preset_bank_t;

typedef struct {
  uint8_t used_mask;
  uint8_t last_slot;
  uint8_t reserved[2];
  trigseq_settings_t slots[APP_PRESET_SLOTS];
} trigseq_preset_bank_t;

typedef struct {
  uint8_t used_mask;
  uint8_t last_slot;
  uint8_t reserved[2];
  euclid_settings_t slots[APP_PRESET_SLOTS];
} euclid_preset_bank_t;

typedef struct {
  uint8_t used_mask;
  uint8_t last_slot;
  uint8_t reserved[2];
  tr2gate_settings_t slots[APP_PRESET_SLOTS];
} tr2gate_preset_bank_t;

typedef struct {
  uint8_t used_mask;
  uint8_t last_slot;
  uint8_t reserved[2];
  tr2adsr_settings_t slots[APP_PRESET_SLOTS];
} tr2adsr_preset_bank_t;

typedef struct {
  uint8_t used_mask;
  uint8_t last_slot;
  uint8_t reserved[2];
  burstgen_settings_t slots[APP_PRESET_SLOTS];
} burstgen_preset_bank_t;

typedef struct {
  uint8_t used_mask;
  uint8_t last_slot;
  uint8_t reserved[2];
  cvgen_settings_t slots[APP_PRESET_SLOTS];
} cvgen_preset_bank_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  grids_preset_bank_t grids;
  trigseq_preset_bank_t trigseq;
  euclid_preset_bank_t euclid;
  tr2gate_preset_bank_t tr2gate;
  tr2adsr_preset_bank_t tr2adsr;
  burstgen_preset_bank_t burstgen;
  cvgen_preset_bank_t cvgen;
} app_presets_blob_t;

static app_presets_blob_t g_blob;
static bool g_loaded = false;

_Static_assert(sizeof(app_presets_blob_t) <= FLASH_SECTOR_SIZE, "Preset blob exceeds one flash sector");

static uint32_t crc32_calc(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  size_t i;
  uint8_t b;

  for (i = 0; i < len; ++i) {
    crc ^= data[i];
    for (b = 0; b < 8u; ++b) {
      if ((crc & 1u) != 0u) {
        crc = (crc >> 1u) ^ 0xEDB88320u;
      } else {
        crc >>= 1u;
      }
    }
  }

  return ~crc;
}

static uint32_t blob_crc(const app_presets_blob_t* blob) {
  app_presets_blob_t tmp = *blob;
  tmp.crc32 = 0u;
  return crc32_calc((const uint8_t*)&tmp, sizeof(tmp));
}

static bool blob_valid(const app_presets_blob_t* blob) {
  if (blob->magic != APP_PRESETS_FLASH_MAGIC) return false;
  if (blob->version != APP_PRESETS_FLASH_VERSION) return false;
  if (blob->size != sizeof(app_presets_blob_t)) return false;
  return blob->crc32 == blob_crc(blob);
}

static void reset_blob_defaults(void) {
  memset(&g_blob, 0, sizeof(g_blob));
  g_blob.magic = APP_PRESETS_FLASH_MAGIC;
  g_blob.version = APP_PRESETS_FLASH_VERSION;
  g_blob.size = sizeof(app_presets_blob_t);
  g_blob.grids.last_slot = APP_PRESET_LAST_SLOT_NONE;
  g_blob.trigseq.last_slot = APP_PRESET_LAST_SLOT_NONE;
  g_blob.euclid.last_slot = APP_PRESET_LAST_SLOT_NONE;
  g_blob.tr2gate.last_slot = APP_PRESET_LAST_SLOT_NONE;
  g_blob.tr2adsr.last_slot = APP_PRESET_LAST_SLOT_NONE;
  g_blob.burstgen.last_slot = APP_PRESET_LAST_SLOT_NONE;
  g_blob.cvgen.last_slot = APP_PRESET_LAST_SLOT_NONE;
  g_blob.crc32 = blob_crc(&g_blob);
}

static bool persist_blob(void) {
  static uint8_t sector_buf[FLASH_SECTOR_SIZE];
  uint32_t irq_state;

  memset(sector_buf, 0xFF, sizeof(sector_buf));
  g_blob.crc32 = blob_crc(&g_blob);
  memcpy(sector_buf, &g_blob, sizeof(g_blob));

  irq_state = save_and_disable_interrupts();
  flash_range_erase(APP_PRESETS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(APP_PRESETS_FLASH_OFFSET, sector_buf, FLASH_SECTOR_SIZE);
  restore_interrupts(irq_state);
  return true;
}

bool app_presets_init(void) {
  const app_presets_blob_t* flash_blob =
      (const app_presets_blob_t*)(XIP_BASE + APP_PRESETS_FLASH_OFFSET);

  if (blob_valid(flash_blob)) {
    g_blob = *flash_blob;
    g_loaded = true;
    return true;
  }

  reset_blob_defaults();
  persist_blob();
  g_loaded = true;
  return false;
}

#define DEFINE_PRESET_FUNCS(prefix, bank_type, field, preset_type)                                              \
  bool app_presets_##prefix##_slot_used(uint8_t slot) {                                                         \
    if (!g_loaded) return false;                                                                                \
    if (slot >= APP_PRESET_SLOTS) return false;                                                                 \
    return (g_blob.field.used_mask & (uint8_t)(1u << slot)) != 0u;                                              \
  }                                                                                                             \
                                                                                                                \
  bool app_presets_##prefix##_save(uint8_t slot, const preset_type* preset) {                                  \
    if (!g_loaded || preset == NULL) return false;                                                              \
    if (slot >= APP_PRESET_SLOTS) return false;                                                                 \
    g_blob.field.slots[slot] = *preset;                                                                         \
    g_blob.field.used_mask = (uint8_t)(g_blob.field.used_mask | (uint8_t)(1u << slot));                        \
    g_blob.field.last_slot = slot;                                                                              \
    return persist_blob();                                                                                      \
  }                                                                                                             \
                                                                                                                \
  bool app_presets_##prefix##_load(uint8_t slot, preset_type* out_preset) {                                    \
    if (!g_loaded || out_preset == NULL) return false;                                                          \
    if (slot >= APP_PRESET_SLOTS) return false;                                                                 \
    if (!app_presets_##prefix##_slot_used(slot)) return false;                                                  \
    *out_preset = g_blob.field.slots[slot];                                                                     \
    return true;                                                                                                \
  }                                                                                                             \
                                                                                                                \
  bool app_presets_##prefix##_load_last(preset_type* out_preset, uint8_t* out_slot) {                          \
    uint8_t slot;                                                                                               \
    if (!g_loaded || out_preset == NULL) return false;                                                          \
    slot = g_blob.field.last_slot;                                                                              \
    if (slot == APP_PRESET_LAST_SLOT_NONE || slot >= APP_PRESET_SLOTS) return false;                           \
    if (!app_presets_##prefix##_slot_used(slot)) return false;                                                  \
    *out_preset = g_blob.field.slots[slot];                                                                     \
    if (out_slot != NULL) *out_slot = slot;                                                                     \
    return true;                                                                                                \
  }

DEFINE_PRESET_FUNCS(grids, grids_preset_bank_t, grids, grids_settings_t)
DEFINE_PRESET_FUNCS(trigseq, trigseq_preset_bank_t, trigseq, trigseq_settings_t)
DEFINE_PRESET_FUNCS(euclid, euclid_preset_bank_t, euclid, euclid_settings_t)
DEFINE_PRESET_FUNCS(tr2gate, tr2gate_preset_bank_t, tr2gate, tr2gate_settings_t)
DEFINE_PRESET_FUNCS(tr2adsr, tr2adsr_preset_bank_t, tr2adsr, tr2adsr_settings_t)
DEFINE_PRESET_FUNCS(burstgen, burstgen_preset_bank_t, burstgen, burstgen_settings_t)
DEFINE_PRESET_FUNCS(cvgen, cvgen_preset_bank_t, cvgen, cvgen_settings_t)
