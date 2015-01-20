// Simple I2C responder
#include "mbed.h"
#include "USBSerial.h"

//SDA, SCL
I2CSlave slave(P0_5, P0_4);

USBSerial pc;

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

int main() {
     char buf[10];
     char msg[] = "Slave!";
     pc.printf("Slave!");
 
     slave.address(0x02);
     while (1) {
         led4 = !led4;
         int i = slave.receive();
         switch (i) {
             case I2CSlave::ReadAddressed:
                 slave.write(msg, strlen(msg)+1); // Includes null char
                 led1 = !led1;
                 pc.printf("Wrote: Slave!");
                 break;
             case I2CSlave::WriteGeneral:
                 slave.read(buf, 10);
                 led2 = !led2;
                 pc.printf("Read G: %s\n", buf);
                 break;
             case I2CSlave::WriteAddressed:
                 slave.read(buf, 10);
                 led3 = !led3;
                 pc.printf("Read A: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
                 break;
             default:
//                 pc.printf("Nop");
                 break;
         }
         for(int i = 0; i < 10; i++) buf[i] = 0;    // Clear buffer
     }
 }