// temperature_sensor.cpp

#include "temperature_sensor.h"
#include <Arduino.h>

/**
 * Constructor for TemperatureSensor, initializes the OneWire bus and DallasTemperature sensor.
 * @param oneWirePin The pin connected to the OneWire bus.
 * @param iterations The number of measurements to take (default is 10).
 */
TemperatureSensor::TemperatureSensor(int oneWirePin, int iterations) 
    : oneWire(oneWirePin), sensors(&oneWire), tempSenseIterations(iterations), analogBufferIndex(0) 
{
    // Allocate memory for the analog buffer
    analogBuffer = new float[tempSenseIterations];
    if (analogBuffer == nullptr) 
    {
        Serial.println("Failed to allocate memory for analog buffer");
    }
}

/**
 * Destructor for TemperatureSensor, deallocates memory for the analog buffer.
 */
TemperatureSensor::~TemperatureSensor() {
    // Deallocate memory for the analog buffer
    delete[] analogBuffer;
}

/**
 * Initializes the DallasTemperature sensor.
 */
void TemperatureSensor::beginSensors() {
    sensors.begin();
}

/**
 * Reads temperature data from the sensor and updates the buffer.
 */
void TemperatureSensor::analogReadAction() {
    sensors.requestTemperatures();
    float sensorValue = sensors.getTempCByIndex(0);
    analogBuffer[analogBufferIndex] = sensorValue;
    analogBufferIndex = (analogBufferIndex + 1) % tempSenseIterations;
}

/**
 * Computes the median temperature from the buffer.
 * @return The median temperature.
 */
float TemperatureSensor::computeMedian() {
    float sortedBuffer[tempSenseIterations]; // Temporary array for sorting
    std::copy(analogBuffer, analogBuffer + tempSenseIterations, sortedBuffer);
    std::sort(sortedBuffer, sortedBuffer + tempSenseIterations);

    if (tempSenseIterations % 2 == 0) {
        float median = (sortedBuffer[tempSenseIterations / 2 - 1] + sortedBuffer[tempSenseIterations / 2]) / 2.0f;
        return median;
    } else {
        return sortedBuffer[tempSenseIterations / 2];
    }
}

/**
 * Reads and adjusts the temperature data.
 * @return The adjusted temperature.
 */
float TemperatureSensor::read(float temperature) {
    analogReadAction();
    return computeMedian();
}

/**
 * Initializes the sensor, calls beginSensors().
 */
void TemperatureSensor::init() {
    beginSensors(); // Initialize the sensors object
}

/**
 * Shuts down the sensor, no operation.
 */
void TemperatureSensor::shutdown() {
    // No shutdown code needed for this sensor
}

/**
 * Stabilizes the sensor with a timer, possible for future operation.
 */
void TemperatureSensor::stabilize() {
    // Stabilization code if necessary
}
