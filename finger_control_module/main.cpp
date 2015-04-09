#include "mbed.h"
#include "I2C.h"
#include <math.h>
#include "util.h"
#include "ControlModule.h"
#include "Ethernet.h"
#include "Output.h"
#include "Printable.h"

/***** Timer Declaration *****/
Ticker fcontroller;
Timer timer;

/***** Set Variable Declarations *****/
static const int addr[4] = {0x02, 0x04, 0x06, 0x08};
ControlModule* controller[4];
PwmOut* solenoid[4];
float dutycycle[4] = {0, 0, 0, 0};
printable printables[4];

/***** Communication Declarations *****/
Output pc(printables);
I2C i2c(PTE25, PTE24);
Ethernet ethernet;

/***** Temporary Variable Declarations *****/
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
    controller[0] = new ControlModule(&printables[0]);
    controller[0]->setControlMode(ControlMode::force);
    solenoid[0] = new PwmOut(PTA2);

    /*** Initialize I/O ***/
    solenoid[0]->period(0.02);
    solenoid[0]->write(0.0);
    i2c.frequency(400000);
    pc.init();

    /*** Start Timing of Control ***/
    fcontroller.attach(&doControl, 0.01); // control frequency in seconds
    timer.start();
    
    while(1) {
        pc.tick();

        // int read = udp.receiveFrom(mosi, eth_buffer, eth_buffer_size);
        // if(read == eth_buffer_size) {
        //     printf("Frame received\r\n");
        //     frameReceivedCB(eth_buffer);
        // } else if(read >= 0) {
        //     printf("*** Partial frame received ***\r\n");
        // } else {
        //     ;
        // } 

        if(controlFlag) {
            pc.printp(0);
            controlFlag = 0;

            if(i2c.read(addr[0], buf, 100, 0) != 0) {
                pc.prints("Failed\n\r");
            }

            float actualf = fbuf[0];
            float actualp = (fbuf[2] + fbuf[3] + fbuf[4]) / 3;
            float actualpre = fbuf[1];

            solenoid[nextController]->write(controller[nextController]->compute(actualf, actualp, actualpre, timer.read()));

            // nextController++;
            if(nextController == 4)
                nextController = 0;
        }
    }
}