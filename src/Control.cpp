#include "Control.h"

void UpdateCommand(Flag_t &f, Command_t &c)
{
    c.command_age = 0;

    if(c.error || f.state == STATE_ERROR){
        f.state = STATE_ERROR;
    }else{
        switch(f.state){
        case STATE_OFF:
            if(c.reset){
                f.state = STATE_INIT;
            }
            break;
        case STATE_INIT:
            break;
        case STATE_READY:
            if(c.enable){
                f.state = STATE_RUN;
            }
            break;
        case STATE_RUN:
            if(!c.enable){
                f.state = STATE_READY;
            }
            break;
        default:
            f.state = STATE_ERROR;
            break;
        }
    }
}

State_e ControlLoopOFF(MIT_CAN &m, Flag_t &f)
{
    if(f.state != f.state_prev) m.set_mode(MIT_OFF);
    digitalWrite(13, 0);

    static uint16_t cnt = 0;
    if(++cnt >= 1000){
        cnt = 0;
        m.set_mode(MIT_OFF);
    }
    return STATE_OFF;
}

State_e ControlLoopINIT(MIT_CAN &m, Flag_t &f)
{
    if(f.state != f.state_prev) m.set_mode(MIT_ON);
    digitalWrite(13, 0);

    m.set_tau(-1); //min torque to set zero
    if(abs(m.get_vel()) < VEL_ZERO){
        static uint8_t cnt = 0;
        if(++cnt >= 1*CTRL_FREQ){
            cnt = 0;
            if(abs(m.get_pos()) > 0.001) m.set_mode(MIT_ZERO);
        }
        if(f.age_cnt++ > 1*CTRL_FREQ && m.get_pos() < 0.01){
            f.age_cnt = 0;
            return STATE_READY;
        }
    }else{
        f.age_cnt = 0;
    }
    return STATE_INIT;
}

State_e ControlLoopREADY(MIT_CAN &m, Flag_t &f, Command_t &c)
{
    if(f.state != f.state_prev) m.set_mode(MIT_ON);

    static uint16_t led_cnt = 0;
    static bool led_flag = false;
    if(++led_cnt >= CTRL_FREQ/2){
        led_cnt = 0;
        led_flag = !led_flag;
        digitalWrite(13, led_flag);
    }

    // m.set_mode(MIT_ON);
    m.set_tau(0);
    digitalWrite(VALVE1, c.valve1);
    digitalWrite(VALVE2, c.valve2);
    return STATE_READY;
}

State_e ControlLoopRUN(MIT_CAN &m, Flag_t &f, Command_t &c)
{
    if(f.state != f.state_prev) m.set_mode(MIT_ON);
    digitalWrite(13, 1);
    
    m.set_tau(c.tau);
    digitalWrite(VALVE1, c.valve1);
    digitalWrite(VALVE2, c.valve2);

    return STATE_RUN;
}

State_e ControlLoopERROR(MIT_CAN &m, Flag_t &f)
{
    if(f.state != f.state_prev) m.set_mode(MIT_OFF);

    static uint16_t led_cnt = 0;
    static bool led_flag = false;
    if(++led_cnt >= CTRL_FREQ/10){
        led_cnt = 0;
        led_flag = !led_flag;
        digitalWrite(13, led_flag);
    }

    digitalWrite(VALVE1, 0);
    digitalWrite(VALVE2, 0);

    // check motor stopped over 5sec
    m.set_tau(-0.1);
    if(abs(m.get_vel()) < VEL_ZERO){
        if(f.age_cnt++ > 5*CTRL_FREQ){
            f.age_cnt = 0;
            return STATE_OFF;
        }
    }else{
        f.age_cnt = 0;
    }
    return STATE_ERROR;
}
