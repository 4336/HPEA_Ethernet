#ifndef _VARIABLE_H_
#define _VARIABLE_H_

#include <Arduino.h>

// #define UART_ASCII
#define ADC_RES_BIT 12

#define VALVE1 8
#define VALVE2 7

typedef enum{
    STATE_OFF,
    STATE_INIT,
    STATE_READY,
    STATE_RUN,
    STATE_ERROR
}State_e;

typedef struct{
    bool debug = true;

    uint16_t age_cnt = 0;

    uint16_t rx_time, rx_time_prev = 0;

    bool control_timer = false;
    bool debug_timer = false;

    bool uart_rx = false;

    State_e state = STATE_OFF;
    State_e state_prev = STATE_OFF;

}Flag_t;

typedef struct{
    uint8_t pressure_pin = A0;
}Pressure_t;

typedef struct{
    float pos, vel, tau = 0;
}MotorState_t;

typedef struct{
    uint16_t time_stamp = 0;

    uint16_t command_age = 0;

    bool enable = false;
    bool error = false;
    bool reset = false;
    bool zero = false;

    bool valve1 = false;
    bool valve2 = false;
    float tau = 0.0;
    float vel_max = 0.0;
    float pos_min = 0.0;
    float pos_max = 0.0;
}Command_t;

#endif