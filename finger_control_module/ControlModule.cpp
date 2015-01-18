#include <math.h>
#include "PID.h"
#include "PiecewiseFit.h"
#include "ControlModule.h"
#include "util.h"

int FORCE_LENGTH = 1;
float FORCE_THRESHOLDS[1] = {0.0};
float FORCE_SLOPES[1]     = {0.8616};
float FORCE_OFFSETS[1]    = {0.041};

int POSITION_LENGTH = 7;
float POSITION_THRESHOLDS[7] = { 0.6742,  0.5878, 0.3682,  0.3275,  0.2403,  0.1640,  0.0};
float POSITION_SLOPES[7]     = {19.6842, 13.3633, 9.5637, 20.6475, 14.4391, 33.0176, 82.7497};
float POSITION_OFFSETS[7]    = { 3.3881,  7.6499, 9.8833,  5.8019,  7.8355,  3.3713, -4.7829};

int PRESSURE_LENGTH = 5;
float PRESSURE_THRESHOLDS[5] = {0.3191,  0.2937, 0.2494,  0.2046, 0.0};
float PRESSURE_SLOPES[5]     = {1.6105,  2.9458, 1.6951,  2.7928, 1.2196};
float PRESSURE_OFFSETS[5]    = {0.1610, -0.2651, 0.1022, -0.1716, 0.1503};

ControlModule::ControlModule() {
    alpha = 0.994;
    forceff = new PiecewiseFit(FORCE_LENGTH, FORCE_THRESHOLDS, FORCE_SLOPES, FORCE_OFFSETS);
    positionff = new PiecewiseFit(POSITION_LENGTH, POSITION_THRESHOLDS, POSITION_SLOPES, POSITION_OFFSETS);
    pressureff = new PiecewiseFit(PRESSURE_LENGTH, PRESSURE_THRESHOLDS, PRESSURE_SLOPES, PRESSURE_OFFSETS);

    forcepid = new PID(alpha, 80, 90, 0, 25, 5, 0, 0.075, (-0.05));
    positionpid = new PID(alpha, 50, 25, 0, 75, 75, 0);
    pressurepid = new PID(alpha, 5, 1, 0.5);

    desiredf = 0.1;
    desiredp = 0.2;
}

ControlModule::~ControlModule() {
    delete forceff;
    delete positionff;
    delete pressureff;

    delete forcepid;
    delete positionpid;
    delete pressurepid;
}

float ControlModule::computeForceTerm(float actualf, float actualp, float time) {
    float pidterm = forcepid->computeStep((desiredf-actualf), time);
    float f_forceff = forceff->getEstimate(desiredf);
    float f_positionff = positionff->getEstimate(actualp);

    return pidterm + f_forceff + f_positionff;
}

float ControlModule::computePositionTerm(float actualp, float time) {
    float pidterm = positionpid->computeStep((desiredp-actualp), time);
    float p_positionff = positionff->getEstimate(desiredp);

    return pidterm + p_positionff;
}

float ControlModule::computeDutycycle(float desiredpre, float actualpre, float time) {
    float pidterm = pressurepid->computeStep((desiredpre-actualpre), time);
    float p_pressureff = pressureff->getEstimate(desiredpre);
    
    return pidterm + p_pressureff;
}

float ControlModule::compute(float actualf, float actualp, float actualpre, float time) {
    float weightp = computeWeight(actualf, desiredf, actualp, desiredp);
    float weightf = 1 - weightp;

    float forceterm = computeForceTerm(actualf, actualp, time);
    float positionterm = computePositionTerm(actualp, time);
    
    float pressure = (weightf * forceterm) + (weightp* positionterm);
    cap(0.0, &pressure, 25.0);
    
    float desiredpre = 0.0122*pressure + 0.1619;
    float dutycycle = computeDutycycle(desiredpre, actualpre, time);
    cap(0.0, &dutycycle, 1.0);
    
    return dutycycle;
}

void ControlModule::setMaximumForce(float force) {
    cap(0.1, &force, 1.0);
    desiredf = force;
}

void ControlModule::setDesiredPosition(float position) {
    cap(0.2, &position, 0.6);
    desiredp = position;
}