#include "mbed.h"
#include "I2C.h"
#include <math.h>
#include "util.h"

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
float weightp = 0, weightf = 0;

float actualf = 0, desiredf = 0, errorf = 0, l_errorf = 0;
float f_kp_p, f_ki_p, f_kd_p;
float f_kp_n, f_ki_n, f_kd_n;
float f_pterm = 0, f_iterm = 0, f_dterm = 0;
float forceterm = 0;
float forceff = 0, f_positionff = 0;

float actualp = 0, desiredp = 0, errorp = 0, l_errorp = 0;
float p_kp_p, p_ki_p, p_kd_p;
float p_kp_n, p_ki_n, p_kd_n;
float p_pterm = 0, p_iterm = 0, p_dterm = 0;
float positionterm = 0;
float p_positionff = 0;

float pressure = 0;

float actualpre = 0, desiredpre = 0, errorpre = 0, l_errorpre = 0;
float pre_kp, pre_ki, pre_kd;
float pre_pterm = 0, pre_iterm = 0, pre_dterm = 0;
float pressureff = 0;

float dutycycle = 0;

float position[5] = {0, 0, 0, 0, 0};
int filter_index = 0;

int step = 0;
int width = 0;

int contact = 0;

char i2cbuf[100];

int controlFlag = 0;
int pwmFlag = 0;
void doControl(void) {
    controlFlag = 1;
}

/***** Main *****/
int main(void) {
    init();

    i2c.frequency(400000);

    fcontroller.attach(&doControl, TIMESTEP);
    pwm.attach(&pwmout, 0.0002);
    incrementer.attach(&increment, 0.1);
    timer.start();
    
    while(1) {
        // pc.printf("%i, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", timer.read_ms(), actualf, desiredf, actualp, desiredp, dutycycle, forceterm, positionterm, weightf, weightp);
    
        if(controlFlag) {
            // if(i2c.read(addr, buf, 100, 0) == 0) {
            //     float* fbuf = (float *)buf;
            //     pc.printf("%f\r\n", fbuf[0]);
            // } else {
            //     pc.printf("Failed\n\r");
            // }
        }
    }
}


void init(void) {
    f_kp_p = 80;
    f_ki_p = 90;
    f_kd_p = 0;
    
    f_kp_n = 25;
    f_ki_n = 5;
    f_kd_n = 0;

    p_kp_p = 50;
    p_ki_p = 25;
    p_kd_p = 0;

    p_kp_n = 75;
    p_ki_n = 75;
    p_kd_n = 0;
    
    pre_kp = 5; 
    pre_ki = 1;
    pre_kd = 0.5;
}

float computeWeight(float force, float max_force, float position, float position_setpoint) {
    float y_norm = (force - CONTACT) / (max_force - CONTACT);
    cap(0, &y_norm, 1);
    float x_norm = (position - NEUTRAL) / (position_setpoint - NEUTRAL);
    cap(0, &x_norm, 1);

    float power = (-15) * (0.5 + (x_norm / 2) - y_norm);
    return (1 / (1 + pow(EULER, power)));
}

void computeControl(void) {
    cap(0.1, &desiredf, 1.0);
    actualf = topForceEgain.read();
    l_errorf = errorf;
    errorf = desiredf - actualf;
    
    cap(0.2, &desiredp, 1.0);
    position[filter_index] = (botStretchEgain.read() + midStretchEgain.read() + topStretchEgain.read()) / 3;
    filter_index++;
    if(filter_index == 5)
        filter_index = 0;
    actualp = (position[0] + position[1] + position[2] + position[3] + position[4]) / 5;
    l_errorp = errorp;
    errorp = desiredp - actualp;

    weightp = computeWeight(actualf, desiredf, actualp, desiredp);
    weightf = 1 - weightp;

    /***** FORCE CONTROL *****/
    if(errorf >= 0) {
        f_pterm = f_kp_p * errorf;
        if(fabs(errorf) < 0.075)
            f_iterm = (f_iterm * ALPHA) + (f_ki_p * errorf * TIMESTEP);
        else
            f_iterm = (f_iterm * ALPHA);
        f_dterm = f_kd_p * (errorf - l_errorf) / TIMESTEP;
    } else {
        f_pterm = f_kp_n * errorf;
        if(fabs(errorf) < 0.05)
            f_iterm = (f_iterm * ALPHA) + (f_ki_n * errorf * TIMESTEP);
        else
            f_iterm = (f_iterm * ALPHA);
        f_dterm = f_kd_n * (errorf - l_errorf) / TIMESTEP;
    }
        
    cap(-25.0, &f_pterm, 25);
    cap(-25.0, &f_iterm, 25);
    cap(-25.0, &f_dterm, 25);

    forceff = (FSLOPE * desiredf) + FOFFSET;
    for(int i = 0; i < POSITION_LENGTH; ++i) {
        if(actualp > POSITION_THRESHOLDS[i]) {
            f_positionff = (POSITION_SLOPES[i] * actualp) + POSITION_OFFSETS[i];
            break;
        }
    }
    forceterm = f_pterm + f_iterm + f_dterm + forceff + f_positionff;

    /***** POSITION CONTROL *****/
    if(errorp >= 0) {
        p_pterm = p_kp_p * errorp;
        p_iterm = (p_iterm * ALPHA) + (p_ki_p * errorp * TIMESTEP);
        p_dterm = p_kd_p * (errorp - l_errorp) / TIMESTEP;
    } else {
        p_pterm = p_kp_n * errorp;
        p_iterm = (p_iterm * ALPHA) + (p_ki_n * errorp * TIMESTEP);
        p_dterm = p_kd_n * (errorp - l_errorp) / TIMESTEP;
    }

    cap(-25.0, &p_pterm, 25);
    cap(-25.0, &p_iterm, 25);
    cap(-25.0, &p_dterm, 25);

    for(int i = 0; i < POSITION_LENGTH; ++i) {
        if(desiredp > POSITION_THRESHOLDS[i]) {
            p_positionff = (POSITION_SLOPES[i] * desiredp) + POSITION_OFFSETS[i];
            break;
        }
    }

    positionterm = p_pterm + p_iterm + p_dterm + p_positionff;
    
    /***** COMBINE TERMS *****/
    //TODO: Add weights
    pressure = (weightf * forceterm) + (weightp* positionterm);

    cap(0.0, &pressure, 25.0);
    
    /***** PRESSURE CONTROL *****/
    desiredpre = 0.0122*pressure + 0.1619;
    actualpre = pressureSensor.read();
    l_errorpre = errorpre;
    errorpre = desiredpre - actualpre;
    
    pre_pterm = pre_kp * errorpre;
    pre_iterm = (pre_iterm * ALPHA) + (pre_ki * errorpre * TIMESTEP);
    pre_dterm = pre_kd * (errorpre - l_errorpre);
    
    cap(-1.0, &pre_pterm, 1.0);
    cap(-1.0, &pre_iterm, 1.0);
    cap(-1.0, &pre_dterm, 1.0);

    for(int i = 0; i < PRESSURE_LENGTH; ++i) {
        if(desiredpre > PRESSURE_THRESHOLDS[i]) {
            pressureff = (PRESSURE_SLOPES[i] * desiredpre) + PRESSURE_OFFSETS[i];
            break;
        }
    }
    
    dutycycle = pre_pterm + pre_iterm + pre_dterm + pressureff;
    cap(0.0, &dutycycle, 1.0);
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
    // if(actualf > CONTACT) {
        // if(step < 400) {
        //     desiredf = ((float)((int)(step/40)))/10.0;
        // } else if(step < 800) {
        //     desiredf = ((float)((int)((800-step)/40)))/10.0;
        // } else if(step < 1000) {
        //     desiredf += 0.005;
        // } else if(step < 1200) {
        //     desiredf -= 0.005;
        // } else if(step < 1300) {
        //     desiredf = 0.3;
        // } else if(step < 1400) {
        //     desiredf = 0.8;
        // } else if(step < 1500) {
        //     desiredf = 0.3;
        // } else {
        //     desiredf = 0.1;
        // }
        // step++;
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
}