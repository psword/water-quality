// temperature_sensor.cpp

#include "temperature_sensor.h"
#include "debug.h"
#include <Arduino.h>

/**
 * Constructor for TemperatureSensor, initializes the OneWire bus and DallasTemperature sensor.
 * @param oneWirePin The pin connected to the OneWire bus.
 * @param iterations The number of measurements to take (default is 10).
 */
TemperatureSensor::TemperatureSensor(int oneWirePin, int iterations) 
    : oneWire(oneWirePin), sensors(&oneWire), tempSenseIterations(iterations), analogBufferIndex(0) 
{
    DEBUG_PRINTLN("TemperatureSensor constructor called");
    DEBUG_PRINT("OneWire Pin: "); DEBUG_PRINTLN(oneWirePin);
    DEBUG_PRINT("Iterations: "); DEBUG_PRINTLN(tempSenseIterations);

    // Allocate memory for the analog buffer
    analogBuffer = new float[tempSenseIterations];
    if (analogBuffer == nullptr) 
    {
        DEBUG_PRINTLN("ERROR: Failed to allocate memory for analog buffer");
    }
    else 
    {
        DEBUG_PRINTLN("Memory allocated for analog buffer");
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
void TemperatureSensor::init() {
    DEBUG_PRINTLN("Initializing temperature sensor...");
    sensors.begin(); 
    DEBUG_PRINTLN("Temperature sensor setup complete.");
}


/**
 * Reads temperature data from the sensor and updates the buffer.
 */
void TemperatureSensor::analogReadAction() {
    sensors.requestTemperatures();
    float sensorValue = sensors.getTempCByIndex(0);
    
    DEBUG_PRINT("Raw Temperature: "); DEBUG_PRINTLN(sensorValue);

    // Check for error value (-127.00°C)
    if (sensorValue != -127.00) {
        analogBuffer[analogBufferIndex] = sensorValue;
        DEBUG_PRINT("Stored Temperature at Index "); 
        DEBUG_PRINT(analogBufferIndex); 
        DEBUG_PRINT(": "); 
        DEBUG_PRINTLN(sensorValue);

        analogBufferIndex = (analogBufferIndex + 1) % tempSenseIterations;
    } else {
        DEBUG_PRINTLN("WARNING: Temperature sensor returned -127.00°C (invalid reading). Keeping last valid reading.");
    }
}



/**
 * Computes the median temperature from the buffer.
 * @return The median temperature.
 */
float TemperatureSensor::computeMedian() {
    float sortedBuffer[tempSenseIterations]; // Temporary array for sorting
    std::copy(analogBuffer, analogBuffer + tempSenseIterations, sortedBuffer);
    std::sort(sortedBuffer, sortedBuffer + tempSenseIterations);

    DEBUG_PRINTLN("Sorted temperature buffer:");
    for (int i = 0; i < tempSenseIterations; i++) {
        DEBUG_PRINT(sortedBuffer[i]); DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN("");

    float medianValue;
    if (tempSenseIterations % 2 == 0) {
        medianValue = (sortedBuffer[tempSenseIterations / 2 - 1] + sortedBuffer[tempSenseIterations / 2]) / 2.0f;
    } else {
        medianValue = sortedBuffer[tempSenseIterations / 2];
    }

    DEBUG_PRINT("Computed Median Temperature: "); DEBUG_PRINTLN(medianValue);
    return medianValue;
}


/**
 * Reads and adjusts the temperature data.
 * @return The adjusted temperature.
 */
float TemperatureSensor::read(float temperature) {
    DEBUG_PRINTLN("Starting temperature reading...");
    analogReadAction();
    
    float medianTemperature = computeMedian();
    
    DEBUG_PRINT("Final Adjusted Temperature: "); 
    DEBUG_PRINTLN(medianTemperature);

    return medianTemperature;
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
