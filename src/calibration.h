#ifndef TRIG_GATE_SEQ_CALIBRATION_H
#define TRIG_GATE_SEQ_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CALIBRATION_CHANNELS 4u

typedef enum {
  CAL_POINT_LOW = 0,
  CAL_POINT_MID_LOW = 1,
  CAL_POINT_MID_HIGH = 2,
  CAL_POINT_HIGH = 3,
  CAL_POINT_COUNT = 4,
} cal_point_t;

typedef enum {
  CAL_RANGE_MODE_NEG3_POS7 = 0,
  CAL_RANGE_MODE_0_POS10 = 1,
  CAL_RANGE_MODE_COUNT = 2,
} cal_range_mode_t;

typedef struct {
  cal_range_mode_t selected_mode;
  uint16_t pt_code[CAL_RANGE_MODE_COUNT][CALIBRATION_CHANNELS][CAL_POINT_COUNT];
} calibration_data_t;

bool calibration_init(calibration_data_t* data);
bool calibration_save(const calibration_data_t* data);
void calibration_set_defaults(calibration_data_t* data);

cal_range_mode_t calibration_get_mode(const calibration_data_t* data);
void calibration_set_mode(calibration_data_t* data, cal_range_mode_t mode);
const char* calibration_mode_label(cal_range_mode_t mode);

uint16_t calibration_get_code(const calibration_data_t* data, cal_point_t point, uint8_t ch);
void calibration_set_code(calibration_data_t* data, cal_point_t point, uint8_t ch, uint16_t code);

const char* calibration_point_label(const calibration_data_t* data, cal_point_t point);
int32_t calibration_point_millivolts(const calibration_data_t* data, cal_point_t point);
int32_t calibration_clamp_voltage_mv(const calibration_data_t* data, int32_t millivolts);
int32_t calibration_min_millivolts(const calibration_data_t* data);
int32_t calibration_max_millivolts(const calibration_data_t* data);
uint16_t calibration_code_for_voltage_mv(const calibration_data_t* data, int32_t millivolts, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif
