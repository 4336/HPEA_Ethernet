#include "Ethernet.h"

// 네트워크 설정

IPAddress ip(192, 168, 64, 128);
unsigned int localPort = 12801;
IPAddress remoteIp(192, 168, 64, 64);
unsigned int remotePort = 6401;

EthernetUDP Udp;

txPacket_t txPacket;
rxPacket_t rxPacket;

void InitEthernet() {
    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
    Ethernet.begin(mac, ip);
    Udp.begin(localPort);
}

int UDP_RX(Command_t &c) {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        Udp.read((uint8_t*)&rxPacket, sizeof(rxPacket));
        txPacket.ping_stamp = rxPacket.time_stamp;
        
        c.time_stamp = rxPacket.time_stamp;
        c.ping = micros() - rxPacket.ping_stamp;
        
        c.enable = (rxPacket.state >> 0) & 0x01;
        c.error = (rxPacket.state >> 1) & 0x01;
        c.reset = (rxPacket.state >> 2) & 0x01;
        c.zero = (rxPacket.state >> 3) & 0x01;

        if(!c.manual_mode){
            c.tau = rxPacket.tau;

            c.valve1 = rxPacket.valve1/65535.0;
            c.valve2 = rxPacket.valve2/65535.0;
        }

        return 1;
    }
    else{
        return 0;
    }
}

void UDP_TX(MIT_CAN &m, Sensor_t &s, Flag_t &f, Command_t &c) {
    txPacket.time_stamp = micros();
    txPacket.ping_stamp = c.time_stamp;
    txPacket.state = f.state;
    txPacket.pos = m.get_pos();
    txPacket.vel = m.get_vel();
    txPacket.tau = m.get_tau();
    txPacket.pressure1 = GetPressure(s, 1);
    txPacket.pressure2 = GetPressure(s, 2);
    txPacket.loadcell = GetLoadcell(s, 1);

    Udp.beginPacket(remoteIp, remotePort);
    Udp.write((uint8_t*)&txPacket, sizeof(txPacket));
    Udp.endPacket();
}