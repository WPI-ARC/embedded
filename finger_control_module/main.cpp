#include "mbed.h"
#include "I2C.h"
#include <math.h>
#include "util.h"
#include "ControlModule.h"

#define CONTACT 0.05
#define TIMESTEP 0.01


/***** Function Prototypes *****/
void init(void);
void computeControl(void);
void pwmout(void);
void increment(void);
float computeWeight(float force, float max_force, float position, float position_setpoint);


/***** Pin Definitions *****/
AnalogIn topForceEgain(PTC11); //P0_13
AnalogIn pressureSensor(PTC10); //P0_11
AnalogIn topStretchEgain(PTB10); //P0_14
AnalogIn midStretchEgain(PTB3); //P0_16
AnalogIn botStretchEgain(PTB2); //P0_12

PwmOut solenoid(PTA2); //P0_9
// DigitalOut solenoid(PTA2);

Serial pc(USBTX, USBRX);
I2C i2c(PTE25, PTE24);

/***** Timer Declaration *****/
Ticker fcontroller;
Ticker pwm;
Ticker incrementer;
Timer timer;

/***** Variable Declarations *****/
static const int addr[4] = {0x02, 0x04, 0x06, 0x08};
ControlModule* controller[4];
float dutycycle[4] = {0, 0, 0, 0};

int width = 0; // TODO: PWM object
char buf[100];
float* fbuf = (float *)buf;

/***** Temporary Variable Declarations *****/
float actualf = 0, desiredf = 0;
float desiredp = 0;

float position[5] = {0, 0, 0, 0, 0};
int filter_index = 0;

int step = 0;

int controlFlag = 0;
void doControl(void) {
    controlFlag = 1;
}

/***** Main *****/
int main(void) {
    solenoid.period(0.02);
    solenoid.write(0);
    i2c.frequency(400000);
    pc.baud(115200);
    pc.printf("Begin Main\r\n");

    controller[0] = new ControlModule();

    pc.printf("Control Object Created\r\n");

    fcontroller.attach(&doControl, TIMESTEP);
    incrementer.attach(&increment, 0.1);
    timer.start();
    
    pc.printf("Begin Main Loop\r\n");
    while(1) {
        pc.printf("l");
        // pc.printf("%i, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", timer.read_ms(), actualf, desiredf, actualp, desiredp, dutycycle, forceterm, positionterm, weightf, weightp);
    
        if(controlFlag) {
            controlFlag = 0;

            if(i2c.read(addr[0], buf, 100, 0) == 0) {
                pc.printf("%f\r\n", fbuf[0]);
            } else {
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
            // solenoid.write(dutycycle[0]);
            pc.printf("%i, %f\r\n", timer.read_ms(), dutycycle[0]);
        }
    }
}

void increment(void) {
    // if(actualf > CONTACT) {
    //     if(step < 400) {
    //         desiredf = ((float)((int)(step/40)))/10.0;
    //     } else if(step < 800) {
    //         desiredf = ((float)((int)((800-step)/40)))/10.0;
    //     } else if(step < 1000) {
    //         desiredf += 0.005;
    //     } else if(step < 1200) {
    //         desiredf -= 0.005;
    //     } else if(step < 1300) {
    //         desiredf = 0.3;
    //     } else if(step < 1400) {
    //         desiredf = 0.8;
    //     } else if(step < 1500) {
    //         desiredf = 0.3;
    //     } else {
    //         desiredf = 0.1;
    //     }
    //     step++;
    // }

    desiredf = 0.8;
    if(step < 50) {
        desiredp = 0.2;
    } else if(step < 250) {
        desiredp = (((float)((int)((step-50)/40)))/10.0) + 0.2;
    } else if(step < 450) {
        desiredp = 0.6 - (((float)((int)((step-250)/40)))/10.0);
    } else if(step < 500) {
        desiredp = 0.2;
    } else if(step < 600) {
        desiredp += 0.0035;
    } else if(step < 700) {
        desiredp -= 0.0035;
    } else if(step < 750) {
        desiredp = 0.25;
    } else if(step < 850) {
        desiredp = 0.4;
    } else if(step < 950) {
        desiredp = 0.6;
    } else if(step < 1050) {
        desiredp = 0.4;
    } else {
        desiredp = 0.25;
    }
    step++;

    cap(0.20, &desiredp, 1.0);
    cap(0.1, &desiredf, 1.0);
    controller[0]->setMaximumForce(desiredf);
    controller[0]->setDesiredPosition(desiredp);
}