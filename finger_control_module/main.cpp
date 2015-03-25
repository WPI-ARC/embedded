#include "mbed.h"
#include "I2C.h"
#include <math.h>
#include "util.h"
#include "ControlModule.h"
#include "Ethernet.h"
#include "Output.h"

#define CONTACT 0.05
#define TIMESTEP 0.01


/***** Function Prototypes *****/
void frameReceivedCB(char* buffer);

PwmOut solenoid(PTA2);

Output pc;
I2C i2c(PTE25, PTE24);
Ethernet ethernet;

/***** Timer Declaration *****/
Ticker fcontroller;
Timer timer;

/***** Variable Declarations *****/
static const int addr[4] = {0x02, 0x04, 0x06, 0x08};
ControlModule* controller[4];
float dutycycle[4] = {0, 0, 0, 0};

char buf[100];
float* fbuf = (float *)buf;

/***** Temporary Variable Declarations *****/
float actualf = 0, desiredf = 0;
float desiredp = 0;

float position[5] = {0, 0, 0, 0, 0};
int filter_index = 0;

int controlFlag = 0;
void doControl(void) {
    controlFlag = 1;
}

char eth_buffer[256];
int eth_buffer_size = 256;

/***** Main *****/
int main(void) {
    solenoid.period(0.02);
    solenoid.write(0.0);
    i2c.frequency(400000);

    pc.printf("Begin Main\r\n");

    controller[0] = new ControlModule();

    pc.printf("Control Object Created\r\n");

    fcontroller.attach(&doControl, TIMESTEP);
    timer.start();
    
    pc.printf("Begin Main Loop\r\n");
    while(1) {
        // pc.printf("l");
        // pc.printf("%i, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", timer.read_ms(), actualf, desiredf, actualp, desiredp, dutycycle, forceterm, positionterm, weightf, weightp);
    
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
            controlFlag = 0;

            if(i2c.read(addr[0], buf, 100, 0) != 0) {
                pc.printf("Failed\n\r");
            }

            actualf = fbuf[0];
            position[filter_index] = (fbuf[2] + fbuf[3] + fbuf[4]) / 3;
            filter_index++;
            if(filter_index == 5)
                filter_index = 0;
            float actualp = (position[0] + position[1] + position[2] + position[3] + position[4]) / 5;
            float actualpre = fbuf[1];

            dutycycle[0] = controller[0]->compute(actualf, actualp, actualpre, timer.read());
            solenoid.write(dutycycle[0]);
        }
    }
}

void frameReceivedCB(char* buffer) {
    //Parse info
    uint32_t command = (buffer[0]<<24) + (buffer[1]<<16) + (buffer[2]<<8) + buffer[3]; // first 4 bytes
    uint8_t len = buffer[4];
    char data[len];
    memcpy(data, buffer + 5, len);
    //pc.printf("Recv cmd, data: %d, [%s]\r\n", command, data);
    // Respond
    char response[eth_buffer_size] = {};
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