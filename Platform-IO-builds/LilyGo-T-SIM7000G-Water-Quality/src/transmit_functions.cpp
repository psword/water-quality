// transmit_functions.cpp

#include "transmit_functions.h"
#include "sensor_config.h"
#include <Wire.h>  // Include any necessary libraries

const uint8_t messageLength = 8;     // Length of message to transmit
const uint8_t slaveAddress = 0x08;   // I2C Slave address

void transmitSlave(uint8_t messageType, float data) {
    Wire.beginTransmission(slaveAddress); // Start I2C transmission with slave
    Wire.write(messageType);              // Send message type
    Wire.write(messageLength);            // Send message length

    uint8_t byteArray[sizeof(float)];        // Create byte array to store data
    memcpy(byteArray, &data, sizeof(float)); // Copy data value to byte array

    for (int i = 0; i < sizeof(float); i++) {
        Wire.write(byteArray[i]); // Send data byte by byte
    }

    uint8_t error = Wire.endTransmission(); // End transmission and capture error

    // Debugging print
    Serial.print("Transmitted message type ");
    Serial.print(messageType);
    Serial.print(" with data: ");
    Serial.print(data);
    Serial.print(" Error: ");
    Serial.println(error);
}

void transmitSleep(uint8_t messageType) {
    Wire.beginTransmission(slaveAddress); // Start I2C transmission with slave
    Wire.write(messageType);              // Send message type
    Wire.write((uint8_t)0);               // Send zero as message length for trigger

    uint8_t error = Wire.endTransmission(); // End transmission and capture error

    // Debugging print
    Serial.print("Transmitted message type ");
    Serial.print(messageType);
    Serial.print(" Error: ");
    Serial.println(error);
}