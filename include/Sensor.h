#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <Arduino.h>
#include "Variables.h"

void Pressure_INIT(Sensor_t &s, uint8_t pin1, uint8_t pin2);
float GetPressure(Sensor_t &s, uint8_t sensor);

void Loadcell_INIT(Sensor_t &s, uint8_t pin1);
float GetLoadcell(Sensor_t &s, uint8_t sensor);

#endif