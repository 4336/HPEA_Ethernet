#include <Arduino.h>

// Local
#include "Variables.h"
#include "Control.h"
#include "UART.h"
#include "MIT_CAN.h"

// Global
Flag_t Flag;
UART_Vars_t UART;

// Control
Command_t Command;
MIT_CAN Motor;
Pressure_t Pressure;

// Timer
IntervalTimer ControlFreq;
IntervalTimer DebugFreq;
void Timer1(){ Flag.control_timer = true; }
void Timer2(){ Flag.debug_timer = true; }

void serialEvent()
{
#ifndef UART_MODE_ASCII
    Flag.uart_rx = UART_RX(UART);
    if(Flag.uart_rx){
        Flag.uart_rx = false;
        UART_Parse(UART, Flag, Command);
        UpdateCommand(Flag, Command);
    }
#else
    uint8_t cmd = UART_RX_ASCII(UART);
    if(cmd){
        Command.command_age = 0;
        
        switch(cmd){
        case 48: // 0
            Flag.state = STATE_OFF;
            break;
        case 49: // 1
            Flag.state = STATE_INIT;
            break;
        case 50: // 2
            Flag.state = STATE_READY;
            break;
        case 51: // 2
            Flag.state = STATE_RUN;
            break;
        case 'a': // off
            Command.valve1 = 0;
            break;
        case 'q': // on
            Command.valve1 = 1;
            break;
        case 's': // off
            Command.valve2 = 0;
            break;
        case 'w': // on
            Command.valve2 = 1;
            break;
        case 'p': // +
            Command.tau += 1;
            break;
        case 'm': // -
            Command.tau -= 1;
            break;
        default: // error
            Flag.state = STATE_ERROR;
        }
    }
#endif
}

void setup() {
    Serial.begin(1152000);

    Pressure_INIT(Pressure, A4, A5);

    pinMode(VALVE1, OUTPUT);
    digitalWrite(VALVE1, 1); // open
    pinMode(VALVE2, OUTPUT);
    digitalWrite(VALVE2, 1); // open
    pinMode(13, OUTPUT);

    analogReadResolution(ADC_RES_BIT);
    analogWriteResolution(DAC_RES_BIT);
    
    ControlFreq.begin(Timer1, 1000/CTRL_FREQ*1000);
    DebugFreq.begin(Timer2, 100*1000);
}

void loop() {
    // CAN RX Update
    Motor.Update_CAN();

    // Main mode
    if(Flag.control_timer)
    {
        Flag.control_timer = false; // clear flag

        switch(Flag.state)
        {
        case STATE_OFF:
            Flag.state = ControlLoopOFF(Motor, Flag);
            break;
        case STATE_INIT:
            Flag.state = ControlLoopINIT(Motor, Flag);
            break;
        case STATE_READY:
            Flag.state = ControlLoopREADY(Motor, Flag, Command);
            break;
        case STATE_RUN:
            Flag.state = ControlLoopRUN(Motor, Flag, Command);
#ifndef UART_MODE_ASCII
            if(Command.command_age++ > 1*CTRL_FREQ) // 1s
            {
                Flag.state = STATE_READY;
            }
#endif
            break;
        default: //STATE_ERROR
            Flag.state = ControlLoopERROR(Motor, Flag);
            break;
        }
        Flag.state_prev = Flag.state;


        UART_TX(UART, Motor, Pressure, Flag, Command);
    }

}
