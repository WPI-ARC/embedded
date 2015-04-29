#include "Ethernet.h"

Ethernet::Ethernet() {
    connected = 0;
}

void Ethernet::init(void) {
	ethernet.init("192.168.1.3", "255.255.255.0", "0.0.0.0");
    ethernet.connect();

    udp.bind(10001);
    mosi.set_address("192.168.1.2", 10001);
    miso.set_address("192.168.1.2", 10002);
    connected = 1;
}

int Ethernet::isConnected(void) {
    return connected;
}

void Ethernet::send(char* response) {
    udp.sendTo(miso, response, 256);
}

int Ethernet::recieve(char* buffer) {
	return udp.receiveFrom(mosi, buffer, 256);
}

void frameReceivedCB(char* buffer) {
	char eth_buffer[256];
	int eth_buffer_size = 256;
    //Parse info
    uint32_t command = (buffer[0]<<24) + (buffer[1]<<16) + (buffer[2]<<8) + buffer[3]; // first 4 bytes
    uint8_t len = buffer[4];
    char data[len];
    memcpy(data, buffer + 5, len);
    //pc.printf("Recv cmd, data: %d, [%s]\r\n", command, data);
    // Respond
    char response[eth_buffer_size] = {}; // why did I do this?
    switch (command) {
            case 0: // Set finger forces/positions
                break;
            case 1: //
                break;
            case 2: // 
                break;
            default:
                break;
    }
    float afloat = -0.329774;
    char* floatptr = (char *)(&afloat);
    response[4] = 0x08;
    response[5] = floatptr[0];
    response[6] = floatptr[1];
    response[7] = floatptr[2];
    response[8] = floatptr[3];
    afloat = 0.635675;
    response[9] = floatptr[0];
    response[10] = floatptr[1];
    response[11] = floatptr[2];
    response[12] = floatptr[3];
    //udp.sendTo(miso, response, eth_buffer_size);
}