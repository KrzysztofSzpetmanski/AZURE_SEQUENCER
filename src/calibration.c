#include "calibration.h"

#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define CALIBRATION_FLASH_MAGIC 0x43414C32u /* CAL2 */
#define CALIBRATION_FLASH_VERSION 3u
#define CALIBRATION_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  uint16_t pt_code[CALIBRATION_CHANNELS][CAL_POINT_COUNT];
} calibration_blob_t;

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

static uint32_t blob_crc(const calibration_blob_t* blob) {
  calibration_blob_t tmp = *blob;
  tmp.crc32 = 0u;
  return crc32_calc((const uint8_t*)&tmp, sizeof(tmp));
}

static bool blob_valid(const calibration_blob_t* blob) {
  if (blob->magic != CALIBRATION_FLASH_MAGIC) {
    return false;
  }
  if (blob->version != CALIBRATION_FLASH_VERSION) {
    return false;
  }
  if (blob->size != sizeof(calibration_blob_t)) {
    return false;
  }
  return blob->crc32 == blob_crc(blob);
}

static int32_t clamp_voltage_mv(int32_t mv) {
  if (mv < -3000) return -3000;
  if (mv > 6000) return 6000;
  return mv;
}

static uint16_t ideal_code_from_mv_range_neg3_pos6(int32_t mv) {
  int64_t num;
  int64_t code;

  mv = clamp_voltage_mv(mv);
  num = ((int64_t)(mv + 3000) * 4095) + 5000;
  code = num / 9000;
  if (code < 0) code = 0;
  if (code > 4095) code = 4095;
  return (uint16_t)code;
}

static uint16_t prescaled_code_from_output_mv(int32_t mv_out) {
  static const int32_t kOutMv[5] = {-7000, -3000, 0, 3000, 6000};
  static const uint16_t kCode[5] = {0, 1229, 2168, 3107, 4042};
  uint8_t i;

  if (mv_out <= kOutMv[0]) return kCode[0];
  if (mv_out >= kOutMv[4]) return kCode[4];

  for (i = 0; i < 4; ++i) {
    int32_t x0 = kOutMv[i];
    int32_t x1 = kOutMv[i + 1];
    uint16_t y0 = kCode[i];
    uint16_t y1 = kCode[i + 1];
    int32_t den = x1 - x0;
    int64_t num;
    int64_t delta;
    int64_t y;

    if (mv_out < x0 || mv_out > x1) continue;

    num = (int64_t)(mv_out - x0) * ((int32_t)y1 - (int32_t)y0);
    if (num >= 0) {
      delta = (num + (den / 2)) / den;
    } else {
      delta = (num - (den / 2)) / den;
    }
    y = (int64_t)y0 + delta;
    if (y < 0) y = 0;
    if (y > 4095) y = 4095;
    return (uint16_t)y;
  }

  return ideal_code_from_mv_range_neg3_pos6(mv_out);
}

void calibration_set_defaults(calibration_data_t* data) {
  uint8_t ch;

  if (data == NULL) return;

  for (ch = 0; ch < CALIBRATION_CHANNELS; ++ch) {
    data->pt_code[ch][CAL_POINT_NEG3] = 0u;
    data->pt_code[ch][CAL_POINT_0] = 1220u;
    data->pt_code[ch][CAL_POINT_POS3] = 2453u;
    data->pt_code[ch][CAL_POINT_POS6] = 3680u;
  }
}

bool calibration_init(calibration_data_t* data) {
  const calibration_blob_t* flash_blob =
      (const calibration_blob_t*)(XIP_BASE + CALIBRATION_FLASH_OFFSET);

  if (data == NULL) return false;

  if (blob_valid(flash_blob)) {
    memcpy(data->pt_code, flash_blob->pt_code, sizeof(data->pt_code));
    return true;
  }

  calibration_set_defaults(data);
  calibration_save(data);
  return false;
}

bool calibration_save(const calibration_data_t* data) {
  calibration_blob_t blob;
  static uint8_t sector_buf[FLASH_SECTOR_SIZE];
  uint32_t irq_state;

  if (data == NULL) return false;

  memset(&blob, 0, sizeof(blob));
  blob.magic = CALIBRATION_FLASH_MAGIC;
  blob.version = CALIBRATION_FLASH_VERSION;
  blob.size = sizeof(calibration_blob_t);
  memcpy(blob.pt_code, data->pt_code, sizeof(blob.pt_code));
  blob.crc32 = blob_crc(&blob);

  memset(sector_buf, 0xFF, sizeof(sector_buf));
  memcpy(sector_buf, &blob, sizeof(blob));

  irq_state = save_and_disable_interrupts();
  flash_range_erase(CALIBRATION_FLASH_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(CALIBRATION_FLASH_OFFSET, sector_buf, FLASH_SECTOR_SIZE);
  restore_interrupts(irq_state);

  return true;
}

uint16_t calibration_get_code(const calibration_data_t* data, cal_point_t point, uint8_t ch) {
  if (data == NULL) return 0;
  if (ch >= CALIBRATION_CHANNELS) ch = 0;
  if (point < 0 || point >= CAL_POINT_COUNT) point = CAL_POINT_NEG3;
  return data->pt_code[ch][point];
}

void calibration_set_code(calibration_data_t* data, cal_point_t point, uint8_t ch, uint16_t code) {
  if (data == NULL) return;
  if (ch >= CALIBRATION_CHANNELS) ch = 0;
  if (point < 0 || point >= CAL_POINT_COUNT) point = CAL_POINT_NEG3;
  if (code > 4095) code = 4095;
  data->pt_code[ch][point] = code;
}

int32_t calibration_point_millivolts(cal_point_t point) {
  if (point == CAL_POINT_NEG3) return -3000;
  if (point == CAL_POINT_0) return 0;
  if (point == CAL_POINT_POS3) return 3000;
  return 6000;
}

uint16_t calibration_code_for_voltage_mv(const calibration_data_t* data, int32_t millivolts,
                                         uint8_t ch) {
  int32_t x0;
  int32_t x1;
  uint16_t y0;
  uint16_t y1;
  int64_t num;
  int64_t delta;
  int64_t y;
  int32_t den;

  if (data == NULL) return 0;
  if (ch >= CALIBRATION_CHANNELS) ch = 0;

  millivolts = clamp_voltage_mv(millivolts);

  if (millivolts <= 0) {
    x0 = -3000;
    x1 = 0;
    y0 = data->pt_code[ch][CAL_POINT_NEG3];
    y1 = data->pt_code[ch][CAL_POINT_0];
  } else if (millivolts <= 3000) {
    x0 = 0;
    x1 = 3000;
    y0 = data->pt_code[ch][CAL_POINT_0];
    y1 = data->pt_code[ch][CAL_POINT_POS3];
  } else {
    x0 = 3000;
    x1 = 6000;
    y0 = data->pt_code[ch][CAL_POINT_POS3];
    y1 = data->pt_code[ch][CAL_POINT_POS6];
  }

  den = x1 - x0;
  num = (int64_t)(millivolts - x0) * ((int32_t)y1 - (int32_t)y0);
  if (num >= 0) {
    delta = (num + (den / 2)) / den;
  } else {
    delta = (num - (den / 2)) / den;
  }
  y = (int64_t)y0 + delta;
  if (y < 0) y = 0;
  if (y > 4095) y = 4095;
  return (uint16_t)y;
}
