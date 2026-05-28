#include "app_settings.h"

#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define APP_SETTINGS_FLASH_MAGIC 0x41505053u /* APPS */
#define APP_SETTINGS_FLASH_VERSION 7u
#define APP_SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - (2u * FLASH_SECTOR_SIZE))

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  app_settings_data_t data;
} app_settings_blob_t;

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
}

bool app_settings_init(app_settings_data_t* data) {
  const app_settings_blob_t* flash_blob =
      (const app_settings_blob_t*)(XIP_BASE + APP_SETTINGS_FLASH_OFFSET);

  if (data == NULL) return false;

  if (blob_valid(flash_blob)) {
    *data = flash_blob->data;
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
