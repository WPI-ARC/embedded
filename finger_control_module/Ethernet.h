#pragma once
#include "mbed.h"
#include "EthernetInterface.h"
#include "UDPSocket.h"

class Ethernet {
private:
    EthernetInterface ethernet;
    UDPSocket udp;
    Endpoint miso;
    Endpoint mosi;
public:
    Ethernet();
    void init(void);
    void send(char* response);
    int recieve(char* buffer);
private:
};