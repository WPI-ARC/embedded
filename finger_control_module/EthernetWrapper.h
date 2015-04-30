#pragma once
#include "mbed.h"
#include "EthernetInterface.h"
#include "UDPSocket.h"

class EthernetWrapper {
private:
    EthernetInterface ethernet;
    UDPSocket udp;
    Endpoint miso;
    Endpoint mosi;
    int connected;
public:
    EthernetWrapper();
    void init(void);
    void send(char* response);
    int recieve(char* buffer);
    int isConnected(void);
};