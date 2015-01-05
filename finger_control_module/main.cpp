#include "mbed.h"

#define CONTACT 0.05
#define TIMESTEP 0.01
#define ALPHA 0.994

#define FSLOPE 0.8616
#define FOFFSET 0.041;


/***** Function Prototypes *****/
void init(void);
void computeControl(void);
void pwmout(void);
void increment(void);
void cap(float min, float* val, float max);


/***** Pin Definitions *****/
AnalogIn topForceEgain(PTC11); //P0_13
AnalogIn pressureSensor(PTC10); //P0_11
AnalogIn topStretchEgain(PTB10); //P0_14
AnalogIn midStretchEgain(PTB3); //P0_16
AnalogIn botStretchEgain(PTB2); //P0_12

DigitalOut solenoid(PTA2); //P0_9

Serial pc(USBTX, USBRX);

/***** Timer Declaration *****/
Ticker fcontroller;
Ticker pwm;
Ticker incrementer;
Timer timer;

/***** Constant Declarations *****/
static const int POSITION_LENGTH = 7;
static const float POSITION_THRESHOLDS[7] = { 0.6742,  0.5878, 0.3682,  0.3275,  0.2403,  0.1640,  0.0};
static const float POSITION_SLOPES[7]     = {19.6842, 13.3633, 9.5637, 20.6475, 14.4391, 33.0176, 82.7497};
static const float POSITION_OFFSETS[7]    = { 3.3881,  7.6499, 9.8833,  5.8019,  7.8355,  3.3713, -4.7829};

static const int PRESSURE_LENGTH = 5;
static const float PRESSURE_THRESHOLDS[5] = {0.3191,  0.2937, 0.2494,  0.2046, 0.0};
static const float PRESSURE_SLOPES[5]     = {1.6105,  2.9458, 1.6951,  2.7928, 1.2196};
static const float PRESSURE_OFFSETS[5]    = {0.1610, -0.2651, 0.1022, -0.1716, 0.1503};

/***** Variable Declarations *****/
float actualf = 0, desiredf = 0, errorf = 0, l_errorf = 0;
float f_kp_p, f_ki_p, f_kd_p;
float f_kp_n, f_ki_n, f_kd_n;
float f_pterm = 0, f_iterm = 0, f_dterm = 0;
float posff = 0, forceff = 0;

float pressure = 0;

float actualp = 0, desiredp = 0, errorp = 0, l_errorp = 0;
float p_kp, p_ki, p_kd;
float p_pterm = 0, p_iterm = 0, p_dterm = 0;
float pressureff = 0;

float dutycycle = 0;

float position[5] = {0, 0, 0, 0, 0};
float positionave = 0;
int filter_index = 0;

int step = 0;
int width = 0;

int contact = 0;


/***** Main *****/
int main(void) {
    init();
    
    fcontroller.attach(&computeControl, TIMESTEP);
    pwm.attach(&pwmout, 0.0002);
    incrementer.attach(&increment, 0.1);
    timer.start();
    
    while(1) {
        pc.printf("%i, %f, %f, %f, %f, %f\r\n", timer.read_ms(), positionave, actualf, desiredf, pressure, dutycycle);
    }
}


void init(void) {
    f_kp_p = 30;
    f_ki_p = 75;
    f_kd_p = 5;
    
    f_kp_n = 0;
    f_ki_n = 0;
    f_kd_n = 5;
    
    p_kp = 5; 
    p_ki = 1;
    p_kd = 0.5;
}

void computeControl(void) {
    cap(0.1, &desiredf, 1.0);
    actualf = topForceEgain.read();
    errorf = desiredf - actualf;
    
    position[filter_index] = (botStretchEgain.read() + midStretchEgain.read() + topStretchEgain.read()) / 3;
    filter_index++;
    if(filter_index == 5)
        filter_index = 0;
    positionave = (position[0] + position[1] + position[2] + position[3] + position[4]) / 5;
    
    if(actualf > CONTACT) {
        if(errorf >= 0) {
            f_pterm = f_kp_p * errorf;
            if(fabs(errorf) < 0.075)
                f_iterm = (f_iterm * ALPHA) + (f_ki_p * errorf * TIMESTEP);
            else
                f_iterm = (f_ki_p * errorf * TIMESTEP);
            f_dterm = f_kd_p * (errorf - l_errorf);
        } else {
            f_pterm = f_kp_n * errorf;
            if(fabs(errorf) < 0.05)
                f_iterm = (f_iterm * ALPHA) + (f_ki_n * errorf * TIMESTEP);
            else
                f_iterm = f_ki_n * errorf * TIMESTEP;
            f_dterm = f_kd_n * errorf * TIMESTEP;
        }
        
        cap(-25.0, &f_pterm, 25);
        cap(-25.0, &f_iterm, 25);
        cap(-25.0, &f_dterm, 25);

        for(int i = 0; i < POSITION_LENGTH; ++i) {
            if(positionave > POSITION_THRESHOLDS[i]) {
                posff = (POSITION_SLOPES[i] * positionave) + POSITION_OFFSETS[i];
                break;
            }
        }
        
        forceff = (FSLOPE * desiredf) + FOFFSET;
        
        pressure = f_pterm + f_iterm + f_dterm + forceff + posff;;
        cap(0.0, &pressure, 25.0);
    } else {
        pressure += 0.01;
    }
    
    /***** PRESSURE CONTROL *****/
    desiredp = 0.0122*pressure + 0.1619;
    actualp = pressureSensor.read();
    l_errorp = errorp;
    errorp = desiredp - actualp;
    
    p_pterm = p_kp * errorp;
    p_iterm = (p_iterm * ALPHA) + (p_ki * errorp * TIMESTEP);
    p_dterm = p_kd * (errorp - l_errorp);
    
    cap(-1.0, &p_pterm, 1.0);
    cap(-1.0, &p_iterm, 1.0);
    cap(-1.0, &p_dterm, 1.0);

    for(int i = 0; i < PRESSURE_LENGTH; ++i) {
        if(desiredp > PRESSURE_THRESHOLDS[i]) {
            pressureff = (PRESSURE_SLOPES[i] * desiredp) + PRESSURE_OFFSETS[i];
            break;
        }
    }
    
    dutycycle = p_pterm + p_iterm + p_dterm + pressureff;
    
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
   if(actualf > CONTACT) {
       if(step < 200) {
           desiredf = ((float)((int)(step/20)))/10.0;
       } else if(step < 400) {
           desiredf = ((float)((int)((400-step)/20)))/10.0;
       } else if(step < 600) {
           desiredf += 0.005;
       } else if(step < 800) {
           desiredf -= 0.005;
       } else if(step < 900) {
           desiredf = 0.3;
       } else if(step < 1000) {
           desiredf = 0.8;
       } else if(step < 1100) {
           desiredf = 0.3;
       } else {
           desiredf = 0.1;
       }
           
       step++;
       
       cap(0.0, &desiredf, 1.0);
   }
}

void cap(float min, float* val, float max) {
    if(*val < min)
       *val = min;
    if(*val > max)
       *val = max;
}