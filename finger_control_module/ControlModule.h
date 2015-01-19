#pragma once
#include "PID.h"
#include "PiecewiseFit.h"

class ControlModule {
    float alpha;
    PiecewiseFit* forceff;
    PiecewiseFit* positionff;
    PiecewiseFit* pressureff;

    PID* forcepid;
    PID* positionpid;
    PID* pressurepid;

    float desiredf;
    float desiredp;
public:
    ControlModule();
    ~ControlModule();
    float compute(float actualf, float actualp, float actualpre, float time);
    void setMaximumForce(float force);
    void setDesiredPosition(float position);
private:
    float computeWeight(float actualf, float desiredf, float actualp, float desiredp);
    float computeForceTerm(float actualf, float actualp, float time);
    float computePositionTerm(float actualp, float time);
    float computeDutycycle(float desiredpre, float actualpre, float time);
};