#ifndef _UART_H_
#define _UART_H_

#include <Arduino.h>

#include "Variables.h"
#include "Control.h"
#include "Sensor.h"

//ASCII
#define STX 0x02
#define ETX 0x03
#define LF 0x0a
#define STAR 0x2a

#define UART_TX_LEN 18
#define UART_RX_LEN 11
#define ASCII_RX_LEN 3

typedef struct{
    uint8_t index = 0;
    uint8_t tx_buf[UART_TX_LEN] = {0,};
    uint8_t rx_buf[UART_RX_LEN] = {0,};
}UART_Vars_t;

void UART_TX(UART_Vars_t &u, MIT_CAN &m, Pressure_t &p, Flag_t &f, Command_t &c);
bool UART_RX(UART_Vars_t &u);
uint8_t UART_RX_ASCII(UART_Vars_t &u);
void UART_Parse(UART_Vars_t &u, Flag_t &f, Command_t &c);
uint16_t MakeCRC16(const uint8_t* data, uint16_t length);

#endif