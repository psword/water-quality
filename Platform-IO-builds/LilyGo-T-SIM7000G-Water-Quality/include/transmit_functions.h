// transmit_functions.h

#ifndef TRANSMIT_FUNCTIONS_H
#define TRANSMIT_FUNCTIONS_H

#include <Arduino.h>

void transmitSlave(uint8_t messageType, float data);
void transmitSleep(uint8_t messageType);

#endif // TRANSMIT_FUNCTIONS_H
