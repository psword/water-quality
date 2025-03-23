// temperature_sensor.cpp

#include "temperature_sensor.h"
#include "debug.h"
#include "sensor_config.h"
#include <Arduino.h>
#include <cmath> // For isnan() and isinf()

// Constructor for TemperatureSensor, initializes the OneWire bus and DallasTemperature sensor.
TemperatureSensor::TemperatureSensor(int oneWirePin, int iterations)
    : oneWire(oneWirePin), sensors(&oneWire), tempSenseIterations(iterations), analogBufferIndex(0)
{
    analogBuffer = new float[tempSenseIterations];

    if (analogBuffer == nullptr)
    {
        DEBUG_PRINTLN("ERROR: Failed to allocate memory for analog buffer");
    }
    else
    {
        DEBUG_PRINTLN("Memory allocated for analog buffer");
        for (int i = 0; i < tempSenseIterations; i++)
        {
            analogBuffer[i] = 0.0; // Initialize to a safe default
        }
    }
}

// Destructor for TemperatureSensor, deallocates memory for the analog buffer.
TemperatureSensor::~TemperatureSensor()
{
    // Deallocate memory for the analog buffer
    delete[] analogBuffer;
}

// Initializes the DallasTemperature sensor.
void TemperatureSensor::init()
{
    DEBUG_PRINTLN("Initializing temperature sensor...");
    sensors.begin();
    DEBUG_PRINTLN("Temperature sensor setup complete.");
}

// Reads temperature data from the sensor and updates the buffer.
void TemperatureSensor::analogReadAction()
{
    sensors.requestTemperatures();
    float sensorValue = sensors.getTempCByIndex(0);

    DEBUG_PRINT("Raw Temperature: ");
    DEBUG_PRINTLN(sensorValue);

    if (sensorValue != -127.00 && !isnan(sensorValue) && !isinf(sensorValue))
    {
        analogBuffer[analogBufferIndex] = sensorValue;
        DEBUG_PRINT("Stored Temperature at Index ");
        DEBUG_PRINT(analogBufferIndex);
        DEBUG_PRINT(": ");
        DEBUG_PRINTLN(sensorValue);

        analogBufferIndex = (analogBufferIndex + 1) % tempSenseIterations;
    }
    else
    {
        DEBUG_PRINTLN("WARNING: Invalid temperature reading, keeping last valid value.");
    }
}

// Computes the median temperature from the buffer.
float TemperatureSensor::computeMedian()
{
    float sortedBuffer[tempSenseIterations]; // Temporary array for sorting
    std::copy(analogBuffer, analogBuffer + tempSenseIterations, sortedBuffer);
    std::sort(sortedBuffer, sortedBuffer + tempSenseIterations);

    DEBUG_PRINTLN("Sorted temperature buffer:");
    for (int i = 0; i < tempSenseIterations; i++)
    {
        if (isnan(sortedBuffer[i]) || isinf(sortedBuffer[i]))
        {
            DEBUG_PRINT("ERROR: Invalid value detected in buffer: ");
            DEBUG_PRINTLN(sortedBuffer[i]);
        }
        else
        {
            DEBUG_PRINT(sortedBuffer[i]);
            DEBUG_PRINT(" ");
        }
    }
    DEBUG_PRINTLN("");

    float medianValue;
    if (tempSenseIterations % 2 == 0)
    {
        medianValue = (sortedBuffer[tempSenseIterations / 2 - 1] + sortedBuffer[tempSenseIterations / 2]) / 2.0f;
    }
    else
    {
        medianValue = sortedBuffer[tempSenseIterations / 2];
    }

    DEBUG_PRINT("Computed Median Temperature: ");
    DEBUG_PRINTLN(medianValue);
    return medianValue;
}

// Reads and adjusts the temperature data.
float TemperatureSensor::read(float temperature)
{
    DEBUG_PRINTLN("Starting temperature reading...");
    analogReadAction();

    float medianTemperature = computeMedian();
    float finalTemp = medianTemperature;
    DEBUG_PRINT("Final Adjusted Temperature: ");
    DEBUG_PRINTLN(medianTemperature);

    return medianTemperature;
}

// Shuts down the sensor, no operation.
void TemperatureSensor::shutdown()
{
    // No shutdown code needed for this sensor
}

// Stabilizes the sensor with a timer, possible for future operation.
void TemperatureSensor::stabilize()
{
    // Stabilization code if necessary
}
