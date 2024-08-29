#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <Arduino.h>
#include "Variables.h"
#include "MIT_CAN.h"

#define CTRL_FREQ 1000

#define VEL_ZERO 0.5

void UpdateCommand(Flag_t &f, Command_t &c);
State_e ControlLoopOFF(MIT_CAN &m, Flag_t &f);
State_e ControlLoopINIT(MIT_CAN &m, Flag_t &f);
State_e ControlLoopREADY(MIT_CAN &m, Flag_t &f, Command_t &c);
State_e ControlLoopRUN(MIT_CAN &m, Flag_t &f, Command_t &c);
State_e ControlLoopERROR(MIT_CAN &m, Flag_t &f);

#endif