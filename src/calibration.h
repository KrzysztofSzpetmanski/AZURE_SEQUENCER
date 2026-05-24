#ifndef TRIG_GATE_SEQ_CALIBRATION_H
#define TRIG_GATE_SEQ_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CALIBRATION_CHANNELS 4u

typedef enum {
  CAL_POINT_NEG3 = 0,
  CAL_POINT_0 = 1,
  CAL_POINT_POS3 = 2,
  CAL_POINT_POS6 = 3,
  CAL_POINT_COUNT = 4,
} cal_point_t;

typedef struct {
  uint16_t pt_code[CALIBRATION_CHANNELS][CAL_POINT_COUNT];
} calibration_data_t;

bool calibration_init(calibration_data_t* data);
bool calibration_save(const calibration_data_t* data);
void calibration_set_defaults(calibration_data_t* data);

uint16_t calibration_get_code(const calibration_data_t* data, cal_point_t point, uint8_t ch);
void calibration_set_code(calibration_data_t* data, cal_point_t point, uint8_t ch, uint16_t code);

int32_t calibration_point_millivolts(cal_point_t point);
uint16_t calibration_code_for_voltage_mv(const calibration_data_t* data, int32_t millivolts, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif
