#include <Arduino.h>

// Native Libraries
#include <IntervalTimer.h>

// Private Libraries
#include "Variables.h"
#include "Ethernet.h"
#include "Control.h"
#include "UART.h"
#include "MIT_CAN.h"

// Global
Flag_t Flag;
UART_Vars_t UART;

// Control
Command_t Command;
MIT_CAN Motor;
Sensor_t Sensor;

// Timer
void Timer1(){ Flag.control_timer = true; }
void Timer2(){ Flag.debug_timer = true; }
IntervalTimer ControlFreq;
IntervalTimer DebugFreq;

void setup() {
    // Hardware Init
    pinMode(VALVE1, OUTPUT);
    pinMode(VALVE2, OUTPUT);
    pinMode(13, OUTPUT); // internal LED
    pinMode(31, OUTPUT); // orange LED

    analogReadResolution(ADC_RES_BIT);
    analogWriteResolution(DAC_RES_BIT);

    Pressure_INIT(Sensor, A0, A1);
    Loadcell_INIT(Sensor, A4);

    digitalWrite(VALVE1, 1); // open
    digitalWrite(VALVE2, 1); // open
    Serial.begin(1152000);

    // Ethernet Init
    InitEthernet();
    
    // UdpRxTimer.begin(receiveData, 1000000 / RX_FREQ); // 수신 주기 설정
    ControlFreq.begin(Timer1, 1000000/CTRL_FREQ);
    DebugFreq.begin(Timer2, 1000000/DEBUG_FREQ);
}

void loop() {
    // CAN RX Update
    Motor.Update_CAN();
    if(UDP_RX(Command)){
        UpdateCommand(Flag, Command);
    }

    // Main mode
    if(Flag.control_timer)
    {
        // Serial.println("c"); //debug
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
            if(Command.manual_mode){
                static float analog_max = pow(2, ADC_RES_BIT)-1;
                Command.tau = analogRead(A0)/analog_max*10-5;
            }else{
                if(Command.command_age++ > 1*CTRL_FREQ) // 1s
                {
                    Flag.state = STATE_READY;
                }
            }
            break;
        default: //STATE_ERROR
            Flag.state = ControlLoopERROR(Motor, Flag);
            break;
        }
        Flag.state_prev = Flag.state;

        UDP_TX(Motor, Sensor, Flag, Command);

        UART_Debug_TX(UART, Motor, Sensor, Flag, Command); // Debug Timer
    }

}

void serialEvent()
{
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
        case 'm': // -
            Command.manual_mode = !Command.manual_mode;
            break;
        default: // error
            Flag.state = STATE_ERROR;
        }
    }
}