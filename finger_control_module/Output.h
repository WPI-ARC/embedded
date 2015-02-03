#pragma once
#include "mbed.h"

class Output {
public:
    int time;
    float actualf;
    float desiredf;
    float actualp;
    float desiredp;
    float dutycycle;
    float forceterm;
    float positionterm;
    float weightf;
    float weightp;
private:
    Serial serial;
public:
    Output();
    void init();
    void printpc();
    void printeth();
    void printf(const char* string);
private:
};