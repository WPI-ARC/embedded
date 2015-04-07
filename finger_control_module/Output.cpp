#include "Output.h"

Output::Output(printable* printables) {
	this->printables = printables;
	serial = new Serial(USBTX, USBRX);
}

Output::~Output() {
	delete serial;
}

void Output::init() {
	serial->baud(115200);
	printables[0].string = (char*)malloc(100*sizeof(char));
}

void Output::tick() {
	printpc();
}

int Output::printp(int num) {
	snprintf(printables[num].string, 100, "%f, %f, %f, %f, %f, %f, %f, %f\r\n",
			printables[num].time, printables[num].actualf, printables[num].desiredf,
			printables[num].actualp, printables[num].desiredp, printables[num].actualdc,
			printables[num].forceterm, printables[num].positionterm);
	return prints(printables[num].string);
}

int Output::prints(const char* string) {
	return prints(string, strlen(string));
}

int Output::prints(const char* string, int length) {
	if(length > b_available) {
		return length - b_available;
	} else {
		int i = (index + (100 - b_available)) % 100;
		for(int j = 0; j < length; j++) {
			i++;
			if(i > 100)
				i = 0;

			buffer[i] = string[j];
		}
		b_available -= length;
		return 0;
	}
}

int Output::available(void) {
	return b_available;
}

void Output::printpc() {
	if(serial->writeable() && (b_available < 100)) {
		serial->printf("%c", buffer[index]);

		b_available++;
		index++;
		if(index == 100)
			index = 0;
	}
}