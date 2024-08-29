#ifndef _UART_H_
#define _UART_H_

#include <Arduino.h>

#include "Variables.h"
#include "Control.h"
#include "Sensor.h"

#define DEBUG_FREQ 10

//ASCII
#define LF 0x0a
#define STAR 0x2a

#define ASCII_RX_LEN 3

typedef struct{
    uint8_t index = 0;
    uint8_t rx_buf[ASCII_RX_LEN] = {0,};
}UART_Vars_t;

void UART_Debug_TX(UART_Vars_t &u, MIT_CAN &m, Sensor_t &s, Flag_t &f, Command_t &c);
uint8_t UART_RX_ASCII(UART_Vars_t &u);

#endif