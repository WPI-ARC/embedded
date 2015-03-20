#pragma once
#include "mbed.h"

class Output {
public:
    int time;
    float actualf;
    float desiredf;
    float actualp;
    float desiredp;
    float actualpre;
    float desiredpre;
    float actualdc;
    float desireddc;
    float forceterm;
    float positionterm;
private:
    // Serial serial;
public:
    Output();
    void init();
    void printpc();
    void printeth();
    void printf(const char* string);
private:
};