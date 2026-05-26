#pragma once

#include <stdint.h>

#define HRDW_PIN_ENC_L_SW 0u
#define HRDW_PIN_ENC_R_SW 1u
#define HRDW_PIN_SW1 2u
#define HRDW_PIN_SW2 3u
/* Backward-compatible aliases */
#define HRDW_PIN_SW_A HRDW_PIN_SW1
#define HRDW_PIN_SW_B HRDW_PIN_SW2

#define HRDW_PIN_I2C_SDA 4u
#define HRDW_PIN_I2C_SCL 5u

#define HRDW_PIN_TR1_IN 6u
#define HRDW_PIN_TR2_IN 7u
#define HRDW_PIN_TR3_IN 8u
#define HRDW_PIN_TR4_IN 9u

#define HRDW_PIN_ENC_R_A 10u
#define HRDW_PIN_ENC_R_B 11u
#define HRDW_PIN_ENC_L_B 12u
#define HRDW_PIN_ENC_L_A 13u

#define HRDW_PIN_MUX_S0 14u
#define HRDW_PIN_MUX_S1 15u
#define HRDW_PIN_MUX_S2 16u

#define HRDW_PIN_OLED_CS 17u
#define HRDW_PIN_OLED_SCK 18u
#define HRDW_PIN_OLED_MOSI 19u
#define HRDW_PIN_OLED_DC 20u
#define HRDW_PIN_OLED_BLK 21u
#define HRDW_PIN_OLED_RES 22u

#define HRDW_PIN_CV_MUX_ADC_GPIO 26u
#define HRDW_PIN_CV_MUX_ADC_INPUT 0u
