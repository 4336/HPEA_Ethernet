#ifndef _ETHERNET_H_
#define _ETHERNET_H_

#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include "Variables.h"
#include "Control.h"
#include "Sensor.h"

// Ethernet
extern IPAddress ip;
extern unsigned int localPort;
extern IPAddress remoteIp;
extern unsigned int remotePort;

// 송수신 주기 설정
// #define RX_FREQ 1000 // 수신 주기 (Hz)
// #define TX_FREQ 100 // 송신 주기 (Hz)

extern EthernetUDP Udp;

void InitEthernet();
void UDP_TX(MIT_CAN &m, Sensor_t &s, Flag_t &f, Command_t &c);
int UDP_RX(Command_t &c);

typedef struct {
    uint32_t time_stamp;
    uint32_t ping_stamp;
    uint8_t state;
    float pos;
    float vel;
    float tau;
    float pressure1;
    float pressure2;
    float loadcell;
} txPacket_t;

typedef struct {
    uint32_t time_stamp;
    uint32_t ping_stamp;
    uint8_t state;
    float tau;
    uint16_t valve1;
    uint16_t valve2;
} rxPacket_t;

// extern txPacket_t txPacket;
// extern rxPacket_t rxPacket;

#endif // _ETHERNET_H_