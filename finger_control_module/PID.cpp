#include "PID.h"

PID::PID(float alpha, float kp, float ki, float kd) {
	init(alpha, kp, ki, kd, kp, ki, kd, 500, (-500)); // TODO: Sentinel values
}

PID::PID(float alpha, float kp_p, float ki_p, float kd_p, float kp_n, float ki_n, float kd_n) {
	init(alpha, kp_p, ki_p, kd_p, kp_n, ki_n, kd_n, 500, (-500)); // TODO: Sentinel values
}

PID::PID(float alpha, float kp_p, float ki_p, float kd_p, float kp_n, float ki_n, float kd_n, float ithreshold_p, float ithreshold_n) {
	init(alpha, kp_p, ki_p, kd_p, kp_n, ki_n, kd_n, ithreshold_p, ithreshold_n);
}

PID::~PID() {

}

void PID::init(float alpha, float kp_p, float ki_p, float kd_p, float kp_n, float ki_n, float kd_n, float ithreshold_p, float ithreshold_n) {
	this->alpha = alpha;
	this->kp_p = kp_p;
	this->ki_p = ki_p;
	this->kd_p = kd_p;
	this->kp_n = kp_n;
	this->ki_n = ki_n;
	this->kd_n = kd_n;
	iterm = 0;
	l_error = 0;
	l_time = 0;
	this->ithreshold_p = ithreshold_p;
	this->ithreshold_n = ithreshold_n;
}

float PID::computeStep(float error, float time) {
	float timestep = time - l_time;
	float pterm = 0;
	float dterm = 0;

	if(error >= 0) {
        pterm = kp_p * error;
        if(error < ithreshold_p)
            iterm = (iterm * alpha) + (ki_p * error * timestep);
        else
            iterm = (iterm * alpha);
        dterm = kd_p * (error - l_error) / timestep;
    } else {
        pterm = kp_n * error;
        if(error > (ithreshold_n))
            iterm = (iterm * alpha) + (ki_n * error * timestep);
        else
            iterm = (iterm * alpha);
        dterm = kd_n * (error - l_error) / timestep;
    }

    l_error = error;
    l_time = time;

    return pterm + iterm + dterm;
}