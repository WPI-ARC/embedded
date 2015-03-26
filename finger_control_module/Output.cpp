#include "Output.h"

Output::Output(printable* printables) {
	this->printables = printables;
	// serial = new Serial(USBTX, USBRX);
}

void Output::init() {
	// serial.baud(115200);
}

void Output::printp(int num) {
	//serial.printf();
}

void Output::prints(const char* string) {

}