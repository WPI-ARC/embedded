#pragma once
#include "mbed.h"
#include "EthernetInterface.h"
#include "UDPSocket.h"

class Ethernet {
    EthernetInterface ethernet;
    UDPSocket udp;
    Endpoint miso;
    Endpoint mosi;
public:
    Ethernet();
    void init();
    void send();
    int recieve();
private:
};