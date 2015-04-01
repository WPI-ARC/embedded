#pragma once
#include "mbed.h"
#include "Printable.h"

class Output {
public:
    printable* printables;
private:
    char buffer[100];
    int index = 0;
    int b_available = 100;
    Serial* serial;
public:
    Output(printable* printables);
    ~Output();
    void tick();
    void printp(int num);
    int prints(const char* string);
    int prints(const char* string, int length);
    int available();
private:
    void init();
    void printpc();
    void printeth();
};