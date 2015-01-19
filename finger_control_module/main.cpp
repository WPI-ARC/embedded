#include "mbed.h"
#include "I2C.h"
#include <math.h>
#include "util.h"
#include "ControlModule.h"

#define EULER 2.71828
#define CONTACT 0.05
#define TIMESTEP 0.01
#define ALPHA 0.994

#define FSLOPE 0.8616
#define FOFFSET 0.041

#define NEUTRAL 0.2


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

DigitalOut solenoid(PTA2); //P0_9

Serial pc(USBTX, USBRX);
I2C i2c(PTE25, PTE24);

/***** Timer Declaration *****/
Ticker fcontroller;
Ticker pwm;
Ticker incrementer;
Timer timer;

/***** Constant Declarations *****/
static const int I2CADDR[4] = {0x2, 0x4, 0x6, 0x8};

static const int POSITION_LENGTH = 7;
static const float POSITION_THRESHOLDS[7] = { 0.6742,  0.5878, 0.3682,  0.3275,  0.2403,  0.1640,  0.0};
static const float POSITION_SLOPES[7]     = {19.6842, 13.3633, 9.5637, 20.6475, 14.4391, 33.0176, 82.7497};
static const float POSITION_OFFSETS[7]    = { 3.3881,  7.6499, 9.8833,  5.8019,  7.8355,  3.3713, -4.7829};

static const int PRESSURE_LENGTH = 5;
static const float PRESSURE_THRESHOLDS[5] = {0.3191,  0.2937, 0.2494,  0.2046, 0.0};
static const float PRESSURE_SLOPES[5]     = {1.6105,  2.9458, 1.6951,  2.7928, 1.2196};
static const float PRESSURE_OFFSETS[5]    = {0.1610, -0.2651, 0.1022, -0.1716, 0.1503};

/***** Variable Declarations *****/
float actualf = 0, desiredf = 0;
float desiredp = 0;
float dutycycle = 0;

float position[5] = {0, 0, 0, 0, 0};
int filter_index = 0;

int step = 0;
int width = 0;

ControlModule* controller;
int controlFlag = 0;
void doControl(void) {
    controlFlag = 1;
}

/***** Main *****/
int main(void) {
    i2c.frequency(400000);
    pc.baud(115200);
    controller = new ControlModule();

    fcontroller.attach(&doControl, TIMESTEP);
    pwm.attach(&pwmout, 0.0002);
    incrementer.attach(&increment, 0.1);
    timer.start();
    
    while(1) {
        // pc.printf("%i, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", timer.read_ms(), actualf, desiredf, actualp, desiredp, dutycycle, forceterm, positionterm, weightf, weightp);
    
        if(controlFlag) {
            actualf = topForceEgain.read();

            position[filter_index] = (botStretchEgain.read() + midStretchEgain.read() + topStretchEgain.read()) / 3;
            filter_index++;
            if(filter_index == 5)
                filter_index = 0;
            float actualp = (position[0] + position[1] + position[2] + position[3] + position[4]) / 5;
            float actualpre = pressureSensor.read();

            dutycycle = controller->compute(actualf, actualp, actualpre, timer.read());
        }
    }
}

void pwmout(void) {
    if((width/100.0) < dutycycle) {
        solenoid = 1;
    } else {
        solenoid = 0;
    }
    
    ++width;
    if(width == 100)
        width = 0;  
}

void increment(void) {
    if(actualf > CONTACT) {
        if(step < 400) {
            desiredf = ((float)((int)(step/40)))/10.0;
        } else if(step < 800) {
            desiredf = ((float)((int)((800-step)/40)))/10.0;
        } else if(step < 1000) {
            desiredf += 0.005;
        } else if(step < 1200) {
            desiredf -= 0.005;
        } else if(step < 1300) {
            desiredf = 0.3;
        } else if(step < 1400) {
            desiredf = 0.8;
        } else if(step < 1500) {
            desiredf = 0.3;
        } else {
            desiredf = 0.1;
        }
        step++;
    }

    // desiredf = 0.8;
    // if(step < 50) {
    //     desiredp = 0.2;
    // } else if(step < 250) {
    //     desiredp = (((float)((int)((step-50)/40)))/10.0) + 0.2;
    // } else if(step < 450) {
    //     desiredp = 0.6 - (((float)((int)((step-250)/40)))/10.0);
    // } else if(step < 500) {
    //     desiredp = 0.2;
    // } else if(step < 600) {
    //     desiredp += 0.0035;
    // } else if(step < 700) {
    //     desiredp -= 0.0035;
    // } else if(step < 750) {
    //     desiredp = 0.25;
    // } else if(step < 850) {
    //     desiredp = 0.4;
    // } else if(step < 950) {
    //     desiredp = 0.6;
    // } else if(step < 1050) {
    //     desiredp = 0.4;
    // } else {
    //     desiredp = 0.25;
    // }
    // step++;

    cap(0.20, &desiredp, 1.0);
    cap(0.1, &desiredf, 1.0);
    controller->setMaximumForce(desiredf);
    controller->setDesiredPosition(desiredp);
}