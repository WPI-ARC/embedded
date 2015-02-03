#pragma once
#include "PID.h"
#include "PiecewiseFit.h"

enum class ControlMode {
    none,
    force,
    position,
    hybrid,
    pressure,
    dutycycle
};

class ControlModule {
    ControlMode mode;
    float alpha;
    PiecewiseFit* forceff;
    PiecewiseFit* positionff;
    PiecewiseFit* pressureff;

    PID* forcepid;
    PID* positionpid;
    PID* pressurepid;

    float desiredf;
    float desiredp;
    float desiredpre;
    float desireddc;
public:
    ControlModule();
    ~ControlModule();
    float compute(float actualf, float actualp, float actualpre, float time);
    void setMaximumForce(float force);
    void setDesiredPosition(float position);
    void setDesiredPressure(float pressure);
    void setDesiredDC(float dutycycle);
    void setControlMode(ControlMode mode);
private:
    float computeWeight(float actualf, float desiredf, float actualp, float desiredp);
    float computeForceTerm(float actualf, float actualp, float time);
    float computePositionTerm(float actualp, float time);
    float computeDutycycle(float desiredpre, float actualpre, float time);
};