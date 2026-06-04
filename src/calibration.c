#include "calibration.h"

#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define CALIBRATION_FLASH_MAGIC 0x43414C32u /* CAL2 */
#define CALIBRATION_FLASH_VERSION 4u
#define CALIBRATION_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  uint8_t selected_mode;
  uint8_t reserved[3];
  uint16_t pt_code[CAL_RANGE_MODE_COUNT][CALIBRATION_CHANNELS][CAL_POINT_COUNT];
} calibration_blob_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  uint16_t pt_code[CALIBRATION_CHANNELS][CAL_POINT_COUNT];
} calibration_blob_v3_t;

static const int32_t kRangeTargetsMv[CAL_RANGE_MODE_COUNT][CAL_POINT_COUNT] = {
    {-3000, 0, 3000, 7000},
    {0, 3000, 6000, 10000},
};

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

static uint32_t blob_crc_v3(const calibration_blob_v3_t* blob) {
  calibration_blob_v3_t tmp = *blob;
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

static bool blob_valid_v3(const calibration_blob_v3_t* blob) {
  if (blob->magic != CALIBRATION_FLASH_MAGIC) {
    return false;
  }
  if (blob->version != 3u) {
    return false;
  }
  if (blob->size != sizeof(calibration_blob_v3_t)) {
    return false;
  }
  return blob->crc32 == blob_crc_v3(blob);
}

static cal_range_mode_t sanitize_mode(cal_range_mode_t mode) {
  if (mode < 0 || mode >= CAL_RANGE_MODE_COUNT) {
    return CAL_RANGE_MODE_NEG3_POS7;
  }
  return mode;
}

static int32_t point_millivolts_for_mode(cal_range_mode_t mode, cal_point_t point) {
  mode = sanitize_mode(mode);
  if (point < 0 || point >= CAL_POINT_COUNT) {
    point = CAL_POINT_LOW;
  }
  return kRangeTargetsMv[mode][point];
}

static int32_t mode_min_millivolts(cal_range_mode_t mode) {
  return point_millivolts_for_mode(mode, CAL_POINT_LOW);
}

static int32_t mode_max_millivolts(cal_range_mode_t mode) {
  return point_millivolts_for_mode(mode, CAL_POINT_HIGH);
}

static int32_t clamp_voltage_mv(cal_range_mode_t mode, int32_t mv) {
  int32_t min_mv = mode_min_millivolts(mode);
  int32_t max_mv = mode_max_millivolts(mode);
  if (mv < min_mv) return min_mv;
  if (mv > max_mv) return max_mv;
  return mv;
}

static uint16_t extrapolate_code(uint16_t code_a, int32_t mv_a, uint16_t code_b, int32_t mv_b,
                                 int32_t mv_target) {
  int32_t den = mv_b - mv_a;
  int64_t num;
  int64_t delta;
  int64_t y;

  if (den == 0) return code_b;

  num = (int64_t)(mv_target - mv_a) * ((int32_t)code_b - (int32_t)code_a);
  if (num >= 0) {
    delta = (num + (den / 2)) / den;
  } else {
    delta = (num - (den / 2)) / den;
  }
  y = (int64_t)code_a + delta;
  if (y < 0) y = 0;
  if (y > 4095) y = 4095;
  return (uint16_t)y;
}

static uint16_t ideal_code_from_mode_mv(cal_range_mode_t mode, int32_t mv) {
  int32_t min_mv = mode_min_millivolts(mode);
  int32_t max_mv = mode_max_millivolts(mode);
  int32_t span_mv = max_mv - min_mv;
  int64_t num;
  int64_t code;

  mv = clamp_voltage_mv(mode, mv);
  if (span_mv <= 0) return 0u;

  num = ((int64_t)(mv - min_mv) * 4095) + (span_mv / 2);
  code = num / span_mv;
  if (code < 0) code = 0;
  if (code > 4095) code = 4095;
  return (uint16_t)code;
}

void calibration_set_defaults(calibration_data_t* data) {
  uint8_t mode;
  uint8_t ch;
  uint8_t point;

  if (data == NULL) return;

  data->selected_mode = CAL_RANGE_MODE_NEG3_POS7;
  for (mode = 0; mode < CAL_RANGE_MODE_COUNT; ++mode) {
    for (ch = 0; ch < CALIBRATION_CHANNELS; ++ch) {
      for (point = 0; point < CAL_POINT_COUNT; ++point) {
        data->pt_code[mode][ch][point] =
            ideal_code_from_mode_mv((cal_range_mode_t)mode, point_millivolts_for_mode((cal_range_mode_t)mode,
                                                                                       (cal_point_t)point));
      }
    }
  }
}

bool calibration_init(calibration_data_t* data) {
  const calibration_blob_t* flash_blob =
      (const calibration_blob_t*)(XIP_BASE + CALIBRATION_FLASH_OFFSET);
  const calibration_blob_v3_t* flash_blob_v3 =
      (const calibration_blob_v3_t*)(XIP_BASE + CALIBRATION_FLASH_OFFSET);
  uint8_t ch;

  if (data == NULL) return false;

  if (blob_valid(flash_blob)) {
    data->selected_mode = sanitize_mode((cal_range_mode_t)flash_blob->selected_mode);
    memcpy(data->pt_code, flash_blob->pt_code, sizeof(data->pt_code));
    return true;
  }

  calibration_set_defaults(data);

  if (blob_valid_v3(flash_blob_v3)) {
    data->selected_mode = CAL_RANGE_MODE_NEG3_POS7;
    for (ch = 0; ch < CALIBRATION_CHANNELS; ++ch) {
      data->pt_code[CAL_RANGE_MODE_NEG3_POS7][ch][CAL_POINT_LOW] =
          flash_blob_v3->pt_code[ch][0];
      data->pt_code[CAL_RANGE_MODE_NEG3_POS7][ch][CAL_POINT_MID_LOW] =
          flash_blob_v3->pt_code[ch][1];
      data->pt_code[CAL_RANGE_MODE_NEG3_POS7][ch][CAL_POINT_MID_HIGH] =
          flash_blob_v3->pt_code[ch][2];
      data->pt_code[CAL_RANGE_MODE_NEG3_POS7][ch][CAL_POINT_HIGH] =
          extrapolate_code(flash_blob_v3->pt_code[ch][2], 3000, flash_blob_v3->pt_code[ch][3], 6000,
                           7000);
    }
    calibration_save(data);
    return true;
  }

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
  blob.selected_mode = (uint8_t)sanitize_mode(data->selected_mode);
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
  if (point < 0 || point >= CAL_POINT_COUNT) point = CAL_POINT_LOW;
  return data->pt_code[sanitize_mode(data->selected_mode)][ch][point];
}

void calibration_set_code(calibration_data_t* data, cal_point_t point, uint8_t ch, uint16_t code) {
  if (data == NULL) return;
  if (ch >= CALIBRATION_CHANNELS) ch = 0;
  if (point < 0 || point >= CAL_POINT_COUNT) point = CAL_POINT_LOW;
  if (code > 4095) code = 4095;
  data->pt_code[sanitize_mode(data->selected_mode)][ch][point] = code;
}

cal_range_mode_t calibration_get_mode(const calibration_data_t* data) {
  if (data == NULL) return CAL_RANGE_MODE_NEG3_POS7;
  return sanitize_mode(data->selected_mode);
}

void calibration_set_mode(calibration_data_t* data, cal_range_mode_t mode) {
  if (data == NULL) return;
  data->selected_mode = sanitize_mode(mode);
}

const char* calibration_mode_label(cal_range_mode_t mode) {
  mode = sanitize_mode(mode);
  if (mode == CAL_RANGE_MODE_0_POS10) return "0..10V";
  return "-3..+7V";
}

const char* calibration_point_label(const calibration_data_t* data, cal_point_t point) {
  static const char* kLabels[CAL_RANGE_MODE_COUNT][CAL_POINT_COUNT] = {
      {"-3V", "0V", "+3V", "+7V"},
      {"0V", "+3V", "+6V", "+10V"},
  };
  cal_range_mode_t mode = calibration_get_mode(data);
  if (point < 0 || point >= CAL_POINT_COUNT) {
    point = CAL_POINT_LOW;
  }
  return kLabels[mode][point];
}

int32_t calibration_point_millivolts(const calibration_data_t* data, cal_point_t point) {
  return point_millivolts_for_mode(calibration_get_mode(data), point);
}

int32_t calibration_clamp_voltage_mv(const calibration_data_t* data, int32_t millivolts) {
  return clamp_voltage_mv(calibration_get_mode(data), millivolts);
}

int32_t calibration_min_millivolts(const calibration_data_t* data) {
  return mode_min_millivolts(calibration_get_mode(data));
}

int32_t calibration_max_millivolts(const calibration_data_t* data) {
  return mode_max_millivolts(calibration_get_mode(data));
}

uint16_t calibration_code_for_voltage_mv(const calibration_data_t* data, int32_t millivolts,
                                         uint8_t ch) {
  cal_range_mode_t mode;
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
  mode = calibration_get_mode(data);

  millivolts = clamp_voltage_mv(mode, millivolts);

  if (millivolts <= point_millivolts_for_mode(mode, CAL_POINT_MID_LOW)) {
    x0 = point_millivolts_for_mode(mode, CAL_POINT_LOW);
    x1 = point_millivolts_for_mode(mode, CAL_POINT_MID_LOW);
    y0 = data->pt_code[mode][ch][CAL_POINT_LOW];
    y1 = data->pt_code[mode][ch][CAL_POINT_MID_LOW];
  } else if (millivolts <= point_millivolts_for_mode(mode, CAL_POINT_MID_HIGH)) {
    x0 = point_millivolts_for_mode(mode, CAL_POINT_MID_LOW);
    x1 = point_millivolts_for_mode(mode, CAL_POINT_MID_HIGH);
    y0 = data->pt_code[mode][ch][CAL_POINT_MID_LOW];
    y1 = data->pt_code[mode][ch][CAL_POINT_MID_HIGH];
  } else {
    x0 = point_millivolts_for_mode(mode, CAL_POINT_MID_HIGH);
    x1 = point_millivolts_for_mode(mode, CAL_POINT_HIGH);
    y0 = data->pt_code[mode][ch][CAL_POINT_MID_HIGH];
    y1 = data->pt_code[mode][ch][CAL_POINT_HIGH];
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
