// temperature_sensor.h

#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "sensor.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <algorithm>

/**
 * TemperatureSensor class, derived from Sensor, to read temperature data from a DallasTemperature sensor.
 */
class TemperatureSensor : public Sensor 
{
private:
    const int tempSenseIterations; // Number of measurements to retain for buffer
    OneWire oneWire;
    DallasTemperature sensors;
    float* analogBuffer; // Dynamic array for buffer
    int analogBufferIndex; // Index for the circular buffer

public:
    /**
     * Constructor for TemperatureSensor, initializes the OneWire bus and DallasTemperature sensor.
     * @param oneWirePin The pin connected to the OneWire bus.
     * @param iterations The number of measurements to retain for buffer (default is 10).
     */
    TemperatureSensor(int oneWirePin, int iterations = 10);

    /**
     * Destructor for TemperatureSensor, deallocates memory for the analog buffer.
     */
    ~TemperatureSensor();

    /**
     * Initializes the DallasTemperature sensor.
     */
    void beginSensors();

    /**
     * Reads temperature data from the sensor and updates the buffer.
     */
    void analogReadAction();

    /**
     * Computes the median temperature from the buffer.
     * @return The median temperature.
     */
    float computeMedian();

    /**
     * Reads and adjusts the temperature data.
     * @return The adjusted temperature.
     */
    float read(float temperature = 0.0) override;

    /**
     * Initializes the sensor, calls beginSensors().
     */
    void init() override;

    /**
     * Shuts down the sensor, no operation.
     */
    void shutdown() override;

    /**
     * Stabilizes the sensor with a timer, possible for future operation.
     */
    void stabilize() override;
};

#endif // TEMPERATURE_SENSOR_H
