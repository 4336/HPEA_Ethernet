#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <Arduino.h>
#include "Variables.h"

void Pressure_INIT(Pressure_t p, uint8_t num);
float GetPressure(Pressure_t p);

#endif