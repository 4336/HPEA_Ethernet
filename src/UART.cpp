#include "UART.h"

uint8_t UART_RX_ASCII(UART_Vars_t &u){
    if(u.index == 0){
        if(Serial.available()){
            u.rx_buf[0] = Serial.read();
            if(u.rx_buf[0] == STAR){
                u.index++;
            }
        }
    }else if(Serial.available()>=ASCII_RX_LEN-1){
        for(int i=1; i<ASCII_RX_LEN; i++){
            u.rx_buf[i] = Serial.read();
        }
        if(u.rx_buf[ASCII_RX_LEN-1] == LF){
            u.index = 0;
            Serial.println(u.rx_buf[1]);
            return u.rx_buf[1];
        }else{
            u.index = 0;
        }
    }
    return 0;
}

void UART_Debug_TX(UART_Vars_t &u, MIT_CAN &m, Sensor_t &s, Flag_t &f, Command_t &c){
    if(f.debug_timer && f.debug){
        f.debug_timer = false;
        Serial.print(c.time_stamp); Serial.print(' ');
        Serial.print(c.ping); Serial.print(' ');
        Serial.print(f.state); Serial.print(' ');
        Serial.print("t:"); Serial.print(c.tau); Serial.print(':'); Serial.print(m.get_tau(), 3); Serial.print(' ');
        Serial.print("l:"); Serial.print(GetLoadcell(s, 1)); Serial.print(' ');
        // Serial.print(analogRead(A0)); Serial.println();
        Serial.print("p:"); Serial.print(GetPressure(s, 1)); Serial.print(' ');


        Serial.print(m.get_pos()/PI*180, 3); Serial.print(' ');
        Serial.print(m.get_vel(), 3); Serial.println();
    }
}
