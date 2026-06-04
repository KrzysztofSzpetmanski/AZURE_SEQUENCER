#include "app_settings.h"

#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define APP_SETTINGS_FLASH_MAGIC 0x41505053u /* APPS */
#define APP_SETTINGS_FLASH_VERSION 10u
#define APP_SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - (2u * FLASH_SECTOR_SIZE))

typedef struct {
  uint8_t clock_mode; /* 0=INT, 1=EXT */
  uint16_t bpm;
  uint8_t map_x;
  uint8_t map_y;
  uint8_t chaos;
  uint8_t prob[4];
} grids_settings_v7_t;

typedef struct {
  uint8_t length;
  uint8_t edit_channel;
  uint8_t edit_step;
  uint8_t clock_mode;
  uint16_t bpm;
  uint8_t run;
  uint8_t prob[4];
  uint64_t pattern[4];
} trigseq_settings_v7_t;

typedef struct {
  uint8_t clock_mode;
  uint16_t bpm;
  uint8_t steps[4];
  uint8_t hits[4];
  uint8_t prob[4];
} euclid_settings_v7_t;

typedef struct {
  grids_settings_v7_t grids;
  trigseq_settings_v7_t trigseq;
  euclid_settings_v7_t euclid;
} app_settings_data_v7_t;

typedef struct {
  grids_settings_t grids;
  trigseq_settings_t trigseq;
  euclid_settings_t euclid;
  tr2gate_settings_t tr2gate;
  tr2adsr_settings_t tr2adsr;
} app_settings_data_v8_t;

typedef struct {
  grids_settings_t grids;
  trigseq_settings_t trigseq;
  euclid_settings_t euclid;
  tr2gate_settings_t tr2gate;
  tr2adsr_settings_t tr2adsr;
  burstgen_settings_t burstgen;
} app_settings_data_v9_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  app_settings_data_t data;
} app_settings_blob_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  app_settings_data_v7_t data;
} app_settings_blob_v7_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  app_settings_data_v8_t data;
} app_settings_blob_v8_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  app_settings_data_v9_t data;
} app_settings_blob_v9_t;

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

static uint32_t blob_crc(const app_settings_blob_t* blob) {
  app_settings_blob_t tmp = *blob;
  tmp.crc32 = 0u;
  return crc32_calc((const uint8_t*)&tmp, sizeof(tmp));
}

static uint32_t blob_crc_v7(const app_settings_blob_v7_t* blob) {
  app_settings_blob_v7_t tmp = *blob;
  tmp.crc32 = 0u;
  return crc32_calc((const uint8_t*)&tmp, sizeof(tmp));
}

static uint32_t blob_crc_v8(const app_settings_blob_v8_t* blob) {
  app_settings_blob_v8_t tmp = *blob;
  tmp.crc32 = 0u;
  return crc32_calc((const uint8_t*)&tmp, sizeof(tmp));
}

static uint32_t blob_crc_v9(const app_settings_blob_v9_t* blob) {
  app_settings_blob_v9_t tmp = *blob;
  tmp.crc32 = 0u;
  return crc32_calc((const uint8_t*)&tmp, sizeof(tmp));
}

static bool blob_valid(const app_settings_blob_t* blob) {
  if (blob->magic != APP_SETTINGS_FLASH_MAGIC) {
    return false;
  }
  if (blob->version != APP_SETTINGS_FLASH_VERSION) {
    return false;
  }
  if (blob->size != sizeof(app_settings_blob_t)) {
    return false;
  }
  return blob->crc32 == blob_crc(blob);
}

static bool blob_valid_v7(const app_settings_blob_v7_t* blob) {
  if (blob->magic != APP_SETTINGS_FLASH_MAGIC) {
    return false;
  }
  if (blob->version != 7u) {
    return false;
  }
  if (blob->size != sizeof(app_settings_blob_v7_t)) {
    return false;
  }
  return blob->crc32 == blob_crc_v7(blob);
}

static bool blob_valid_v8(const app_settings_blob_v8_t* blob) {
  if (blob->magic != APP_SETTINGS_FLASH_MAGIC) {
    return false;
  }
  if (blob->version != 8u) {
    return false;
  }
  if (blob->size != sizeof(app_settings_blob_v8_t)) {
    return false;
  }
  return blob->crc32 == blob_crc_v8(blob);
}

static bool blob_valid_v9(const app_settings_blob_v9_t* blob) {
  if (blob->magic != APP_SETTINGS_FLASH_MAGIC) {
    return false;
  }
  if (blob->version != 9u) {
    return false;
  }
  if (blob->size != sizeof(app_settings_blob_v9_t)) {
    return false;
  }
  return blob->crc32 == blob_crc_v9(blob);
}

void app_settings_set_defaults(app_settings_data_t* data) {
  if (data == NULL) return;

  memset(data, 0, sizeof(*data));
  data->grids.clock_mode = 1u; /* EXT */
  data->grids.bpm = 120u;
  data->grids.map_x = 128u;
  data->grids.map_y = 128u;
  data->grids.chaos = 64u;
  data->grids.prob[0] = 100u;
  data->grids.prob[1] = 100u;
  data->grids.prob[2] = 100u;
  data->grids.prob[3] = 100u;

  data->trigseq.length = 16u;
  data->trigseq.edit_channel = 0u;
  data->trigseq.edit_step = 0u;
  data->trigseq.clock_mode = 1u; /* EXT */
  data->trigseq.bpm = 120u;
  data->trigseq.run = 1u;
  data->trigseq.prob[0] = 100u;
  data->trigseq.prob[1] = 100u;
  data->trigseq.prob[2] = 100u;
  data->trigseq.prob[3] = 100u;
  data->trigseq.pattern[0] = 0x000000000000AAAAull;
  data->trigseq.pattern[1] = 0x000000000000CCCCull;
  data->trigseq.pattern[2] = 0x000000000000F0F0ull;
  data->trigseq.pattern[3] = 0x000000000000FF00ull;

  data->euclid.clock_mode = 1u; /* EXT */
  data->euclid.bpm = 120u;
  data->euclid.steps[0] = 16u;
  data->euclid.steps[1] = 16u;
  data->euclid.steps[2] = 16u;
  data->euclid.steps[3] = 16u;
  data->euclid.hits[0] = 4u;
  data->euclid.hits[1] = 6u;
  data->euclid.hits[2] = 8u;
  data->euclid.hits[3] = 10u;
  data->euclid.prob[0] = 100u;
  data->euclid.prob[1] = 100u;
  data->euclid.prob[2] = 100u;
  data->euclid.prob[3] = 100u;

  data->tr2gate.src[0] = 0u;
  data->tr2gate.src[1] = 1u;
  data->tr2gate.src[2] = 2u;
  data->tr2gate.src[3] = 3u;
  data->tr2gate.gate_time_cs[0] = 50u;
  data->tr2gate.gate_time_cs[1] = 50u;
  data->tr2gate.gate_time_cs[2] = 50u;
  data->tr2gate.gate_time_cs[3] = 50u;
  data->tr2gate.prob[0] = 100u;
  data->tr2gate.prob[1] = 100u;
  data->tr2gate.prob[2] = 100u;
  data->tr2gate.prob[3] = 100u;
  data->tr2gate.level_10v = 1u;

  data->tr2adsr.src[0] = 0u;
  data->tr2adsr.src[1] = 1u;
  data->tr2adsr.src[2] = 2u;
  data->tr2adsr.src[3] = 3u;
  data->tr2adsr.type[0] = 2u;
  data->tr2adsr.type[1] = 2u;
  data->tr2adsr.type[2] = 2u;
  data->tr2adsr.type[3] = 2u;
  data->tr2adsr.total_time_ds[0] = 10u;
  data->tr2adsr.total_time_ds[1] = 10u;
  data->tr2adsr.total_time_ds[2] = 10u;
  data->tr2adsr.total_time_ds[3] = 10u;
  data->tr2adsr.prob[0] = 100u;
  data->tr2adsr.prob[1] = 100u;
  data->tr2adsr.prob[2] = 100u;
  data->tr2adsr.prob[3] = 100u;
  data->tr2adsr.level_10v = 1u;
  data->tr2adsr.a_pct[0] = 25u;
  data->tr2adsr.a_pct[1] = 25u;
  data->tr2adsr.a_pct[2] = 25u;
  data->tr2adsr.a_pct[3] = 25u;
  data->tr2adsr.d_pct[0] = 25u;
  data->tr2adsr.d_pct[1] = 25u;
  data->tr2adsr.d_pct[2] = 25u;
  data->tr2adsr.d_pct[3] = 25u;
  data->tr2adsr.s_pct[0] = 70u;
  data->tr2adsr.s_pct[1] = 70u;
  data->tr2adsr.s_pct[2] = 70u;
  data->tr2adsr.s_pct[3] = 70u;
  data->tr2adsr.r_pct[0] = 25u;
  data->tr2adsr.r_pct[1] = 25u;
  data->tr2adsr.r_pct[2] = 25u;
  data->tr2adsr.r_pct[3] = 25u;

  data->burstgen.selected_channel = 0u;
  data->burstgen.signature_mode = 0u;
  data->burstgen.bpm = 120u;
  data->burstgen.swing_pct = 15u;
  data->burstgen.probability = 100u;
  data->burstgen.level_10v = 1u;

  data->cvgen.algo[0] = 0u;
  data->cvgen.algo[1] = 1u;
  data->cvgen.algo[2] = 2u;
  data->cvgen.algo[3] = 3u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    data->cvgen.clock_mode[i] = 0u;
    data->cvgen.src[i] = i;
    data->cvgen.rate_dhz[i] = 25u;
    data->cvgen.min_mv[i] = 0;
    data->cvgen.max_mv[i] = 10000;
    data->cvgen.param1[i] = 50u;
    data->cvgen.param2[i] = 50u;
  }
  data->cvgen.param1[2] = 30u;
  data->cvgen.param2[2] = 45u;
  data->cvgen.param1[3] = 35u;
  data->cvgen.param2[3] = 0u;
}

bool app_settings_init(app_settings_data_t* data) {
  const app_settings_blob_t* flash_blob =
      (const app_settings_blob_t*)(XIP_BASE + APP_SETTINGS_FLASH_OFFSET);
  const app_settings_blob_v9_t* flash_blob_v9 =
      (const app_settings_blob_v9_t*)(XIP_BASE + APP_SETTINGS_FLASH_OFFSET);
  const app_settings_blob_v8_t* flash_blob_v8 =
      (const app_settings_blob_v8_t*)(XIP_BASE + APP_SETTINGS_FLASH_OFFSET);
  const app_settings_blob_v7_t* flash_blob_v7 =
      (const app_settings_blob_v7_t*)(XIP_BASE + APP_SETTINGS_FLASH_OFFSET);

  if (data == NULL) return false;

  if (blob_valid(flash_blob)) {
    *data = flash_blob->data;
    return true;
  }

  if (blob_valid_v9(flash_blob_v9)) {
    app_settings_set_defaults(data);
    memcpy(&data->grids, &flash_blob_v9->data.grids, sizeof(data->grids));
    memcpy(&data->trigseq, &flash_blob_v9->data.trigseq, sizeof(data->trigseq));
    memcpy(&data->euclid, &flash_blob_v9->data.euclid, sizeof(data->euclid));
    memcpy(&data->tr2gate, &flash_blob_v9->data.tr2gate, sizeof(data->tr2gate));
    memcpy(&data->tr2adsr, &flash_blob_v9->data.tr2adsr, sizeof(data->tr2adsr));
    memcpy(&data->burstgen, &flash_blob_v9->data.burstgen, sizeof(data->burstgen));
    app_settings_save(data);
    return true;
  }

  if (blob_valid_v8(flash_blob_v8)) {
    app_settings_set_defaults(data);
    memcpy(&data->grids, &flash_blob_v8->data.grids, sizeof(data->grids));
    memcpy(&data->trigseq, &flash_blob_v8->data.trigseq, sizeof(data->trigseq));
    memcpy(&data->euclid, &flash_blob_v8->data.euclid, sizeof(data->euclid));
    memcpy(&data->tr2gate, &flash_blob_v8->data.tr2gate, sizeof(data->tr2gate));
    memcpy(&data->tr2adsr, &flash_blob_v8->data.tr2adsr, sizeof(data->tr2adsr));
    app_settings_save(data);
    return true;
  }

  if (blob_valid_v7(flash_blob_v7)) {
    app_settings_set_defaults(data);
    memcpy(&data->grids, &flash_blob_v7->data.grids, sizeof(data->grids));
    memcpy(&data->trigseq, &flash_blob_v7->data.trigseq, sizeof(data->trigseq));
    memcpy(&data->euclid, &flash_blob_v7->data.euclid, sizeof(data->euclid));
    app_settings_save(data);
    return true;
  }

  app_settings_set_defaults(data);
  app_settings_save(data);
  return false;
}

bool app_settings_save(const app_settings_data_t* data) {
  app_settings_blob_t blob;
  static uint8_t sector_buf[FLASH_SECTOR_SIZE];
  uint32_t irq_state;

  if (data == NULL) return false;

  memset(&blob, 0, sizeof(blob));
  blob.magic = APP_SETTINGS_FLASH_MAGIC;
  blob.version = APP_SETTINGS_FLASH_VERSION;
  blob.size = sizeof(app_settings_blob_t);
  blob.data = *data;
  blob.crc32 = blob_crc(&blob);

  memset(sector_buf, 0xFF, sizeof(sector_buf));
  memcpy(sector_buf, &blob, sizeof(blob));

  irq_state = save_and_disable_interrupts();
  flash_range_erase(APP_SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(APP_SETTINGS_FLASH_OFFSET, sector_buf, FLASH_SECTOR_SIZE);
  restore_interrupts(irq_state);

  return true;
}
