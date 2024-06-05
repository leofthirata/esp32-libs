#ifndef __MAIN_H__
#define __MAIN_H__

#define SOC_WIFI_SUPPORTED  1
#define ESP32	1
#define LIB_DEBUG 0
#define CORE_DEBUG_LEVEL	ARDUHAL_LOG_LEVEL_NONE

#include "Arduino.h"
#include "LedBicolor.hpp"

#define PIN_NUM_SWITCH 4
#define PIN_LED_STATUS_RED  GPIO_NUM_33
#define PIN_LED_STATUS_GREEN GPIO_NUM_25
#define PIN_NUM_SDA         27
#define PIN_NUM_SCL         26

#define PIN_BAT_ADC_CTRL    GPIO_NUM_15

extern LedBicolor *led;
void setup();
void loop();

#endif //__MAIN_H__