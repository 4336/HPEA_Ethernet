#include "MIT_CAN.h"

MIT_CAN::MIT_CAN() {
    flex_can.begin(); //begin flex_can (CRX2[0], CTX2[1])
    flex_can.setBaudRate(1000000); //1Mhz Can baurate

    msg_w.len = 8; //length of can message (8byte)
    msg_w.id = 0x00; //address of can device

    memset(msg_w.buf, 0xff, 8); //set initial can message to motor off signal
    memset(msg_r.buf, 0x00, 8); //set initial can message to motor off signal

    state.pos = 0;
    state.vel = 0;
    state.tau = 0;
    state_prev = state;

    kp = 0;
    kd = 0;
    mode = 0;

}

void MIT_CAN::Update_CAN(){
    while(flex_can.read(msg_r)){
        // Serial.println("rx_can");
        can_unpack(msg_r);
    }
    return;
}

float MIT_CAN::get_pos(){
    return state.pos;
}

float MIT_CAN::get_vel(){
    return state.vel;
}

float MIT_CAN::get_tau(){
    return state.tau;
}

void MIT_CAN::set_tau(float tau) {
    can_pack(0, 0, tau, kp, kd); // design can message for left(Master) motor // 2*pi -> 0.133 // 0.021756
    flex_can.write(msg_w);
}

void MIT_CAN::set_pos(float pos) {
    can_pack(pos, 0, 0, kp, kd); // design can message for left(Master) motor // 2*pi -> 0.133 // 0.021756
    flex_can.write(msg_w);
}

void MIT_CAN::set_gain(float _kp, float _kd){
    kp = _kp;
    kd = _kd;
}

void MIT_CAN::set_mode(CtrlMode_e mode) {
    memset(msg_w.buf, 0xff, 8); // set buf[0]~buf[7] to 0xff

    switch (mode) {
    case MIT_OFF:
        msg_w.buf[7] = 0xfd; // Motor Mode Off
        break;
    case MIT_ON:
        msg_w.buf[7] = 0xfc; // Motor Mode On
        break;
    case MIT_ZERO:
        msg_w.buf[7] = 0xfe; // Zero Position
        break;
    }

    flex_can.write(msg_w); // Send can message through can1 port
}

void MIT_CAN::can_pack(float p_des, float v_des, float t_des, float kp_des, float kd_des) {

    // convert float to unsigned int (12 or 16 bit) //
    uint16_t p_int = constrain((p_des / (P_MAX - P_MIN) + 0.5) * 65535.0f, 0, 65535);
    uint16_t v_int = constrain((v_des / (V_MAX - V_MIN) + 0.5) * 4095.0f, 0, 4095);
    uint16_t t_int = constrain((t_des / (T_MAX - T_MIN) + 0.5) * 4095.0f, 0, 4095);
    uint16_t kp_int = constrain(kp_des / KP_MAX * 4095.0f, 0, 4095);
    uint16_t kd_int = constrain(kd_des / KD_MAX * 4095.0f, 0, 4095);

    // convert unsigned int (12 or 16 bit) to hex //
    msg_w.buf[0] = p_int >> 8;   // position 8-H
    msg_w.buf[1] = p_int & 0xFF; // position 8-L
    msg_w.buf[2] = v_int >> 4;   // speed 8-H
    msg_w.buf[3] = (v_int & 0xF) << 4 | (kp_int >> 8); // speed 4-L KP-4H
    msg_w.buf[4] = kp_int & 0xFF; // KP 8-L
    msg_w.buf[5] = kd_int >> 4;   // Kd 8-H
    msg_w.buf[6] = (kd_int & 0xF) << 4 | (t_int >> 8); // KP 4-L torque 4-H
    msg_w.buf[7] = t_int & 0xff; // torque 8-L
}

void MIT_CAN::can_unpack(CAN_message_t& msg_read) {//float &pos, float &vel, float &tau){

    // convert hex to unsigned int (12 or 16 bit) //
    //uint16_t id = msg_read.buf[0];
    uint16_t p_int = (msg_read.buf[1] << 8) | msg_read.buf[2]; // position : 8-H | 8-L
    uint16_t v_int = (msg_read.buf[3] << 4) | (msg_read.buf[4] >> 4); // velocity : 4-H | 4-L
    uint16_t t_int = ((msg_read.buf[4] & 0xF) << 8) | msg_read.buf[5]; // torque : 4-H | 4-L

    // convert float to unsigned int //
    float p = ((float)p_int / 65535.0f - 0.5) * (P_MAX - P_MIN);
    float v = ((float)v_int / 4095.0f - 0.5) * (V_MAX - V_MIN);
    float t = ((float)t_int / 4095.0f - 0.5) * (T_MAX - T_MIN);

    state_prev = state;
    state.pos = p;
    state.vel = v;
    state.tau = t;

}
