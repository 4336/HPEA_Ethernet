#include "UART.h"

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Pressure_INIT(Sensor_t &s, uint8_t pin1, uint8_t pin2){
    s.pressure1_pin = pin1;
    s.pressure2_pin = pin2;
}

float GetPressure(Sensor_t &s, uint8_t sensor)
{
    float pressure;
    
    // static float in_max = (pow(2, ADC_RES_BIT)-1)/0.99; // (2^bit-1)/(3.3/(5*2/3))
    // static float in_min = in_max/3.3*0.666667; // in_min:1V*(2/3) = in_max:3.3V
    static float in_max = pow(2, ADC_RES_BIT)-1; // (2^bit-1)/(3.3/(5*2/3))
    static float in_min = 1*(3.3/5)*in_max; // in_min:1V*(2/3) = in_max:3.3V

    switch(sensor){
        case 1:
            pressure = float_map(analogRead(s.pressure1_pin), in_min, in_max, 0, 1000); //kPa; 5V
            break;
        case 2:
            pressure = float_map(analogRead(s.pressure2_pin), in_min, in_max, 0, 1000); //kPa; 5V
            break;
        default:
            pressure = 0;
            break;
    }

    return pressure;
}

void Loadcell_INIT(Sensor_t &s, uint8_t pin1){
    s.loadcell_pin = pin1;
}

float GetLoadcell(Sensor_t &s, uint8_t sensor)
{
    float load;
    
    static float in_min = (pow(2, ADC_RES_BIT)-1) * 0.125; // 12bit 512
    static float in_max = (pow(2, ADC_RES_BIT)-1); // 12bit 4095

    switch(sensor){
        case 1:
            load = float_map(analogRead(s.loadcell_pin), in_min, in_max, 0, 7.100861294); //kgf, gain 2033.204058(output 40.66408116)
            break;
        default:
            load = 0;
            break;
    }

    return load;
}
