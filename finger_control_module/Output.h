#pragma once
#include "mbed.h"
#include "Printable.h"

class Output {
public:
    printable* printables;
private:
    // Serial serial;
public:
    Output(printable* printables);
    void init();
    void printpc();
    void printeth();
    void printp(int num);
    void prints(const char* string);
private:
};