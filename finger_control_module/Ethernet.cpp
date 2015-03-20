#include "Ethernet.h"

Ethernet::Ethernet() {

}

void Ethernet::init(void) {
	ethernet.init("192.168.1.3", "255.255.255.0", "0.0.0.0");
    ethernet.connect();

    udp.bind(10001);
    mosi.set_address("192.168.1.2", 10001);
    miso.set_address("192.168.1.2", 10002);
}

void Ethernet::send(char* response) {
    udp.sendTo(miso, response, 256);
}

int Ethernet::recieve(char* buffer) {
	return udp.receiveFrom(mosi, buffer, 256);
}