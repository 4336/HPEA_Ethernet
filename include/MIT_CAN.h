#ifndef _MIT_CAN_H_
#define _MIT_CAN_H_

#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "Variables.h"

typedef enum{
    MIT_OFF,
    MIT_ON,
    MIT_ZERO
}CtrlMode_e;

// #define AK80_9_V2
#ifdef AK80_9_V2
    #define P_MIN -12.5f
    #define P_MAX 12.5f
    #define V_MIN -50.0f
    #define V_MAX 50.0f
    #define T_MIN -18.0f
    #define T_MAX 18.0f

    #define KP_MIN 0.0f
    #define KP_MAX 500.0f
    #define KD_MIN 0.0f
    #define KD_MAX 5.0f
#else
    #define P_MIN -12.5f
    #define P_MAX 12.5f
    #define V_MIN -50.0f
    #define V_MAX 50.0f
    #define T_MIN -18.0f
    #define T_MAX 18.0f

    #define KP_MIN 0.0f
    #define KP_MAX 500.0f
    #define KD_MIN 0.0f
    #define KD_MAX 5.0f
#endif

class MIT_CAN
{
public:

    MIT_CAN();

    void Update_CAN();

    float get_pos();
    float get_vel();
    float get_tau();

    void set_tau(float tau);
    void set_pos(float pos);
    void set_mode(CtrlMode_e mode);
    void set_gain(float kp, float kd);

private:

    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flex_can;
    CAN_message_t msg_w, msg_r;

    MotorState_t state;
    MotorState_t state_prev;

    int mode;
    float kp, kd;

    void can_pack(float p_des, float v_des, float t_des, float kp_des, float kd_des);
    void can_unpack(CAN_message_t& msg_read);
	
};

#endif
