#include "mbed.h"
#include <math.h>

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
void cap(float min, float* val, float max);
float computeWeight(float force, float max_force, float position, float position_setpoint);


/***** Pin Definitions *****/
AnalogIn topForceEgain(PTC11); //P0_13
AnalogIn pressureSensor(PTC10); //P0_11
AnalogIn topStretchEgain(PTB10); //P0_14
AnalogIn midStretchEgain(PTB3); //P0_16
AnalogIn botStretchEgain(PTB2); //P0_12

// DigitalOut solenoid(PTA2); //P0_9
PwmOut solenoid(PTA2);

Serial pc(USBTX, USBRX);

/***** Timer Declaration *****/
Ticker fcontroller;
Ticker pwm;
Ticker incrementer;
Timer timer;

/***** Constant Declarations *****/
static const int POSITION_LENGTH = 4;
static const float POSITION_THRESHOLDS[4] = {0.2618, 0.2142,  0.1905, 0.0};
static const float POSITION_SLOPES[4]     = {0.2166, 0.6304,  4.2187, 0.8922};
static const float POSITION_OFFSETS[4]    = {0.2433, 0.1349, -0.6338, 0.0};

static const int PRESSURE_LENGTH = 2;
static const float PRESSURE_THRESHOLDS[2] = { 0.1956,  0.1744};
static const float PRESSURE_SLOPES[2]     = { 1.0429,  9.4116};
static const float PRESSURE_OFFSETS[2]    = {-0.0040, -1.6410};

/***** Variable Declarations *****/
float weightp = 0, weightf = 0;

float actualf = 0, desiredf = 0, errorf = 0, l_errorf = 0;
float f_kp_p, f_ki_p, f_kd_p;
float f_kp_n, f_ki_n, f_kd_n;
float f_pterm = 0, f_iterm = 0, f_dterm = 0;
float forceterm = 0;
float forceff = 0, f_positionff = 0;

float stretch0 = 0, stretch1 = 0, stretch2 = 0;
float actualp = 0, desiredp = 0, errorp = 0, l_errorp = 0;
float p_kp_p, p_ki_p, p_kd_p;
float p_kp_n, p_ki_n, p_kd_n;
float p_pterm = 0, p_iterm = 0, p_dterm = 0;
float positionterm = 0;
float p_positionff = 0;

float forcelimiter = 0;
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


/***** Main *****/
int main(void) {
    solenoid.period(0.02);
    solenoid.write(0.0);
    init();
    pc.baud(115200);
    
    fcontroller.attach(&computeControl, TIMESTEP);
    // pwm.attach(&pwmout, 0.0002);
    incrementer.attach(&increment, 0.1);
    timer.start();
    
    while(1) {
        // calibrate pressure and position stepping dutycycle
        // pc.printf("%i, %f, %f, %f, %f, %f\r\n", timer.read_ms(), dutycycle, actualpre, stretch0, stretch1, stretch2);
        // pc.printf("%i, %f, %f, %f, %f\r\n", timer.read_ms(), dutycycle, actualpre, actualp, actualf);

        // debugging force
        // pc.printf("%i, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", timer.read_ms(), actualf, desiredf, dutycycle, f_pterm, f_iterm, f_dterm, f_positionff, actualp, actualpre);

        // debuging prius
        // pc.printf("%i, %f, %f, %f, %f, %f, %f, %f\r\n", timer.read_ms(), actualf, actualp, desiredp, positionterm, p_positionff, f_positionff, forcelimiter);

        // standard print
        pc.printf("%i, %f, %f, %f, %f, %f, %f, %f\r\n", timer.read_ms(), actualf, desiredf, actualp, desiredp, dutycycle, forceterm, positionterm);
    }
}


void init(void) {
    f_kp_p = 80;
    f_ki_p = 90;
    f_kd_p = 0.5;
    
    f_kp_n = 25;
    f_ki_n = 5;
    f_kd_n = 0.5;

    p_kp_p = 75;
    p_ki_p = 100;
    p_kd_p = 0;

    p_kp_n = 75;
    p_ki_n = 175;
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
    return (1 / (1 + pow(exp(1), power)));
}

void computeControl(void) {
    cap(0.1, &desiredf, 1.0);
    actualf = topForceEgain.read();
    l_errorf = errorf;
    errorf = desiredf - actualf;

    if(fabs(errorf) > 0.05) {
        f_iterm = 0;
    }
    if(fabs(errorp) > 0.05) {
        p_iterm = 0;
    }
    
    cap(0.2, &desiredp, 1.0);
    stretch0 = botStretchEgain.read();
    stretch1 = midStretchEgain.read();
    stretch2 = topStretchEgain.read();
    position[filter_index] = (stretch0 + stretch1)/2; // + stretch2)/3; top is dead again...
    filter_index++;
    if(filter_index == 5)
        filter_index = 0;
    actualp = (position[0] + position[1] + position[2] + position[3] + position[4]) / 5;
    l_errorp = errorp;
    errorp = desiredp - actualp;

    /***** FORCE CONTROL *****/
    if(errorf >= 0) {
        f_pterm = f_kp_p * errorf;
        f_iterm = (f_iterm * ALPHA) + (f_ki_p * errorf * TIMESTEP);
        f_dterm = f_kd_p * (errorf - l_errorf) / TIMESTEP;
    } else {
        f_pterm = f_kp_n * errorf;
        f_iterm = (f_iterm * ALPHA) + (f_ki_n * errorf * TIMESTEP);
        f_dterm = f_kd_n * (errorf - l_errorf) / TIMESTEP;
    }
        
    cap(-25.0, &f_pterm, 25);
    cap(-25.0, &f_iterm, 25);
    cap(-25.0, &f_dterm, 25);

    forceff = (FSLOPE * desiredf) + FOFFSET;
    for(int i = 0; i < POSITION_LENGTH; ++i) {
        if(actualp > POSITION_THRESHOLDS[i]) {
            f_positionff = ((POSITION_SLOPES[i] * actualp) + POSITION_OFFSETS[i] - 0.1619)/0.0122;
            break;
        }
    }
    forceterm = f_pterm + f_iterm + f_dterm + forceff + f_positionff;
    cap(-25.0, &forceterm, 25);

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
            p_positionff = ((POSITION_SLOPES[i] * desiredp) + POSITION_OFFSETS[i] - 0.1619)/0.0122;
            break;
        }
    }

    positionterm = p_pterm + p_iterm + p_dterm + p_positionff;
    cap(-25.0, &positionterm, 25);
    
    /***** COMBINE TERMS *****/
    float power = -6.0 * errorf;
    forcelimiter = 1 - pow(EULER, power);
    cap(0.0, &forcelimiter, 1.0);

    pressure = fmin(positionterm, forceterm); //(positionterm * forcelimiter) + ((f_positionff + (FSLOPE*desiredf+FOFFSET)) * 0.95 * (1 - forcelimiter));

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

    solenoid.write(dutycycle);
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
    // calibrate pressure & position
    // if(step < 10000) {
    //     if(((step / 1000) % 2) == 0) {
    //         dutycycle = ((float)(step % 1000) / 1000.0) * 0.15 + 0.2;
    //     } else {
    //         dutycycle = ((1000 - (float)(step % 1000)) / 1000.0) * 0.15 + 0.2;
    //     }
    // } else {
    //     dutycycle = 0;
    // }

    // if(actualf > CONTACT) {
    //     if(step < 440) {
    //         desiredf = ((float)((int)(step/40)))/10.0;
    //     } else if(step < 840) {
    //         desiredf = ((float)((int)((840-step)/40)))/10.0;
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

    desiredf = 0.1;
    desiredp = 0.35;
    // if(step < 50) {
    //     desiredp = 0.25;
    // } else if(step < 690) {
    //     desiredp = (((float)((int)((step-50)/80)))/20.0) + 0.25;
    // } else if(step < 1330) {
    //     desiredp = 0.6 - (((float)((int)((step-690)/80)))/20.0);
    // } else if(step < 1400) {
    //     desiredp = 0.25;
    // } else if(step < 1533) {
    //     desiredp += 0.0030;
    // } else if(step < 1666) {
    //     desiredp -= 0.0030;
    // } else if(step < 1800) {
    //     desiredp = 0.25;
    // } else if(step < 1900) {
    //     desiredp = 0.40;
    // } else if(step < 2000) {
    //     desiredp = 0.55;
    // } else if(step < 2100) {
    //     desiredp = 0.70;
    // } else if(step < 2200) {
    //     desiredp = 0.55;
    // } else if(step < 2300) {
    //     desiredp = 0.40;
    // } else {
    //     desiredp = 0.25;
    // }
    // step++;

    cap(0.20, &desiredp, 1.0);
    cap(0.1, &desiredf, 1.0);
}

void cap(float min, float* val, float max) {
    if(*val < min)
       *val = min;
    if(*val > max)
       *val = max;
}