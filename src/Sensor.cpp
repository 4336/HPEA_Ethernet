#include "UART.h"

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Pressure_INIT(Pressure_t &p, uint8_t pin1, uint8_t pin2){
    p.sensor1_pin = pin1;
    p.sensor2_pin = pin2;
}

float GetPressure(Pressure_t p, uint8_t sensor)
{
    float pressure;
    
#if 1 // with 3.3V volt voltage normalization
    static float in_max = (pow(2, ADC_RES_BIT)-1)/0.99; // 2^bit-1)/(3.3/(5*2/3))
    static float in_min = in_max/3.3*0.666667; // in_min:1V*(2/3) = in_max:3.3V

    switch(sensor){
        case 1:
            pressure = float_map(analogRead(p.sensor1_pin), in_min, in_max, 0, 1000); //kPa; 5V
            break;
        case 2:
            pressure = float_map(analogRead(p.sensor2_pin), in_min, in_max, 0, 1000); //kPa; 5V
            break;
        default:
            pressure = 0;
            break;
    }
#else // without voltage normalization (5V)
    static float in_max = pow(2, ADC_RES_BIT)-1;
    static float in_min = in_max/3.3; // in_min:1V = in_max:3.3V

    switch(sensor){
        case 1:
            pressure = float_map(analogRead(p.sensor1_pin), in_min, in_max, 0, 575); //kPa; 3.3V
            break;
        case 2:
            pressure = float_map(analogRead(p.sensor2_pin), in_min, in_max, 0, 575); //kPa; 3.3V
            break;
        default:
            pressure = 0;
            break;
    }
#endif

    return pressure;
}
