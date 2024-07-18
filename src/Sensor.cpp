#include "UART.h"

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Pressure_INIT(Pressure_t p, uint8_t num){
    if(num == 0){
        p.pressure_pin = A0;
    }
}

float GetPressure(Pressure_t p)
{
    float pressure;
    pressure = float_map(analogRead(p.pressure_pin), 1240.9, pow(2, ADC_RES_BIT)-1, 0, 750); //kPa
    return pressure;
}
