#include "Output.h"

Output::Output() {
	serial = new Serial(USBTX, USBRX);
}

void Output::init() {
	serial.baud(115200);
}