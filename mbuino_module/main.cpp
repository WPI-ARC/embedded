#include "mbed.h"
#include "I2C.h"
#include "USBSerial.h"

// USBSerial pc;

//SDA, SCL
// I2CSlave slave(P0_5, P0_4);

DigitalOut led1(P1_19);
DigitalOut led2(P1_15);
DigitalOut led3(P0_23);
DigitalOut led4(P0_20);
DigitalOut led5(P0_2);
DigitalOut led6(P0_17);
DigitalOut led7(P0_18);
DigitalOut led8(P0_19);

AnalogIn topForceEgain(P0_15);
AnalogIn midForceEgain(P0_16);
AnalogIn pressureSensor(P0_14);
AnalogIn topStretchEgain(P0_13);
AnalogIn midStretchEgain(P0_12);
AnalogIn botStretchEgain(P0_11);

int main() {
    while(1) {
        // float a1 = topForceEgain.read();
        // float a2 = midForceEgain.read();
        // float a3 = pressureSensor.read();
        // float a4 = topStretchEgain.read();
        // float a5 = midStretchEgain.read();
        // float a6 = botStretchEgain.read();
        // pc.printf("%f, %f, %f, %f, %f, %f\r\n", a1, a2, a3, a4, a5, a6);

        led1 = !led1;
        led2 = !led2;
        led3 = !led3;
        led4 = !led4;
        led5 = !led5;
        led6 = !led6;
        led7 = !led7;
        led8 = !led8;
    }

    // char buf[10];
    // float msg[5] = {0, 0, 0, 0, 0};
    // slave.address(0x02);
    // slave.frequency(400000);
    // int a = 0;
    // while (1) {
    //     led4 = !led4;
    //     int i = slave.receive();
    //     switch (i) {
    //         case I2CSlave::ReadAddressed:
    //             slave.write((char*)msg, sizeof(float)*5);
    //             led1 = !led1;
    //             a = -1;
    //             break;
    //         case I2CSlave::WriteGeneral:
    //             slave.read(buf, 10);
    //             led2 = !led2;
    //             break;
    //         case I2CSlave::WriteAddressed:
    //             slave.read(buf, 10);
    //             led3 = !led3;
    //             break;
    //         default:
    //             break;
    //     }
    //     for(int i = 0; i < 10; i++) buf[i] = 0;    // Clear buffer
        
    //     switch (a) {
    //         case 0:
    //             msg[0] = topForceEgain.read();
    //             break;
    //         case 1:
    //             msg[1] = pressureSensor.read();
    //             break;
    //         case 2:
    //             msg[2] = topStretchEgain.read();
    //             break;
    //         case 3:
    //             msg[3] = midStretchEgain.read();
    //             break;
    //         case 4:
    //             msg[4] = botStretchEgain.read();
    //             break;
    //         default:
    //             break;
    //     }
    //     ++a;
    // }
}