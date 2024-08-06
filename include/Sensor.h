#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <Arduino.h>
#include "Variables.h"

void Pressure_INIT(Pressure_t &p, uint8_t pin1, uint8_t pin2);
float GetPressure(Pressure_t p, uint8_t sensor);

#endif