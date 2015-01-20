#include "mbed.h"
#include "I2C.h"

//SDA, SCL
I2CSlave slave(P0_5, P0_4);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

AnalogIn topForceEgain(P0_13);
AnalogIn pressureSensor(P0_11);
AnalogIn topStretchEgain(P0_14);
AnalogIn midStretchEgain(P0_16);
AnalogIn botStretchEgain(P0_12);

int main() {
    char buf[10];
    float msg[5] = {0, 0, 0, 0, 0};
    slave.address(0x02);
    slave.frequency(400000);
    int a = 0;
    while (1) {
        led4 = !led4;
        int i = slave.receive();
        switch (i) {
            case I2CSlave::ReadAddressed:
                slave.write((char*)msg, sizeof(float)*5);
                led1 = !led1;
                a = -1;
                break;
            case I2CSlave::WriteGeneral:
                slave.read(buf, 10);
                led2 = !led2;
                break;
            case I2CSlave::WriteAddressed:
                slave.read(buf, 10);
                led3 = !led3;
                break;
            default:
                break;
        }
        for(int i = 0; i < 10; i++) buf[i] = 0;    // Clear buffer
        
        switch (a) {
            case 0:
                msg[0] = topForceEgain.read();
                break;
            case 1:
                msg[1] = pressureSensor.read();
                break;
            case 2:
                msg[2] = topStretchEgain.read();
                break;
            case 3:
                msg[3] = midStretchEgain.read();
                break;
            case 4:
                msg[4] = botStretchEgain.read();
                break;
            default:
                break;
        }
        ++a;
    }
}