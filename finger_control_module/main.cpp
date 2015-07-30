#include "mbed.h"
#include "I2C.h"
#include <math.h>
#include "util.h"
#include "ControlModule.h"
#include "EthernetInterface.h"
#include "UDPSocket.h"
#include "Output.h"
#include "Printable.h"

void frameReceivedCB(char* buffer);

/***** Timer Declaration *****/
Ticker fcontroller;
Timer timer;

/***** Set Variable Declarations *****/
static const int addr[4] = {0x02, 0x04, 0x06, 0x08};
ControlModule* controller[4];
PwmOut* solenoid[4];
float dutycycle[4] = {0, 0, 0, 0};
printable printables[4];

// QEI* encoder[4];
// DRV8833* motor[4];
// PID* knucklePID[4];

/***** Communication Declarations *****/
EthernetInterface ethernet;
UDPSocket udp;
Endpoint miso;
Endpoint mosi;

Output pc(printables);
I2C i2c(PTE25, PTE24);

/***** Temporary Variable Declarations *****/
char eth_buffer[256];

char buf[100];
float* fbuf = (float *)buf;

int controlFlag = 0;
int nextController = 0;
void doControl(void) {
    controlFlag = 1;
}

/***** Main *****/
int main(void) {
    /*** Initialize Control Modules ***/
    // encoder[0] = new QEI(PTC5, PTC7, NC, 12, QEI::X2_ENCODING);
    // encoder[1] = new QEI(PTC0, PTC9, NC, 12, QEI::X2_ENCODING);
    // encoder[2] = new QEI(PTB23,PTA2, NC, 12, QEI::X2_ENCODING);
    // encoder[3] = new QEI(PTB9, PTA1, NC, 12, QEI::X2_ENCODING);
    
    // motor[0] = DRV8833(PTD1, PTD3);
    // motor[1] = DRV8833(PTD2, PTD0);
    // motor[2] = DRV8833(PTB19,PTB18);
    // motor[3] = DRV8833(PTC4, PTA0);

    solenoid[0] = new PwmOut(PTC3);
    solenoid[1] = new PwmOut(PTC2);
    solenoid[2] = new PwmOut(PTC11);
    solenoid[3] = new PwmOut(PTC10);

    for(int i = 0; i < 4; i++) {
        controller[i] = new ControlModule(&printables[i]);
        controller[i]->setControlMode(ControlMode::none);
        solenoid[i]->period(0.02);
        solenoid[i]->write(0.0);
    }

    /*** Initialize I/O ***/
    i2c.frequency(400000);
    pc.init();

    pc.serial->printf("Eth Init\n\r");
    ethernet.init("192.168.1.3", "255.255.255.0", "0.0.0.0");
    ethernet.connect();
    udp.set_blocking(false, 1); // timeout of zero breaks things
    udp.bind(10001);
    mosi.set_address("192.168.1.2", 10001);
    miso.set_address("192.168.1.2", 10002);
    pc.serial->printf("Eth Connected\n\r");

    /*** Start Timing of Control ***/
    fcontroller.attach(&doControl, 0.01); // control period in seconds
    timer.start();
    
    while(1) {
        pc.tick();
        // udp.sendTo(miso, eth_buffer, 256);
        // int read = udp.receiveFrom(mosi, eth_buffer, 256);
        // if(read == 256) {
        //     // pc.serial->printf("Frame\r\n");
        //     // frameReceivedCB(eth_buffer);
        // } else if(read > 0) {
        //     // pc.prints("Partial\r\n"); // partial frame, kinda an error
        // } else {
        //     // pc.prints("Empty\r\n"); // nothing recieved
        // }

        if(controlFlag) {
            pc.printp(nextController);
            controlFlag = 0;

            if(i2c.read(addr[nextController], buf, 100, 0) != 0) {
                // pc.prints("Failed\n\r");
            }

            // float actualf = topForceEgain.read(); // fbuf[0];
            // float actualp = (topStretchEgain.read() + midStretchEgain.read() + botStretchEgain.read()) / 3; // (fbuf[2] + fbuf[3] + fbuf[4]) / 3;
            // float actualpre = pressureSensor.read(); //fbuf[1];

            float actualf = fbuf[0];
            float actualp = (fbuf[2] + fbuf[3] + fbuf[4]) / 3;
            float actualpre = fbuf[1];
            solenoid[nextController]->write(controller[nextController]->compute(actualf, actualp, actualpre, timer.read()));

            nextController++;
            if(nextController == 4)
                nextController = 0;

            timer.reset();
        }
    }
}

void frameReceivedCB(char* buffer) {
    //Parse info
    uint32_t command = (buffer[0]<<24) + (buffer[1]<<16) + (buffer[2]<<8) + buffer[3]; // first 4 bytes
    uint8_t len = 96;//buffer[4];
    char* data;
    data = (char *)(buffer + 5);
    // memcpy(data, buffer + 5, len);
    float* floatdata;

    // 4 + 4 + 4 + 4 + 4 + 1 (+3)
    for(int i = 0; i < 4; i++) {
        floatdata = (float *)(data + (i * 24));
        // pc.serial->printf("%d, %d, %f, %f, %f, %f\r\n", i, data[(i * 24) + 20], floatdata[0], floatdata[1], floatdata[2], floatdata[3]);

    //     controller[i]->setDesiredDC(floatdata[0]);
    //     controller[i]->setDesiredPressure(floatdata[1]);
    //     controller[i]->setMaximumForce(floatdata[2]);
    //     controller[i]->setDesiredPosition(floatdata[3]);

    //     ControlMode mode = ControlMode::none;
    //     switch(data[(i * 24) + 20]) {
    //         case 1:
    //             mode = ControlMode::dutycycle;
    //             break;
    //         case 2:
    //             mode = ControlMode::force;
    //             break;
    //         case 3:
    //             mode = ControlMode::position;
    //             break;
    //         case 4:
    //             mode = ControlMode::pressure;
    //             break;
    //         default:
    //             mode = ControlMode::none;
    //             break;
    //     }

    //     controller[i]->setControlMode(mode);
    }
}