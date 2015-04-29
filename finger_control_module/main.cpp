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

// QEI* encoder[4];
// DRV8833* motor[4];
// PID* knucklePID[4];

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
        solenoid[i]->write(0.5);
    }

    /*** Initialize I/O ***/
    i2c.frequency(400000);
    ethernet.init();
    pc.init();

    /*** Start Timing of Control ***/
    fcontroller.attach(&doControl, 0.01); // control frequency in seconds
    timer.start();
    
    while(1) {
        pc.tick();
        if(ethernet.isConnected()) {
            pc.prints("Ethernet connected\n\r");
        } else {
            pc.prints("Ethernet disconnected\n\r");
        }
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
            pc.printp(nextController);
            controlFlag = 0;

            if(i2c.read(addr[nextController], buf, 100, 0) != 0) {
                pc.prints("Failed\n\r");
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
        }
    }
}