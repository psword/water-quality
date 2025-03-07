// ph_sensor.cpp

#include "ph_sensor.h"
#include <algorithm>        // Include for std::copy and std::sort
#include <Arduino.h>

// Define the ADS1115 object
ADS1115_WE adcPH(I2C_ADDRESS_PH);

float acidVoltage    = 1.02513;    //buffer solution 4.0 at 25C
float neutralVoltage = .75525;     //buffer solution 7.0 at 25C
float baseVoltage    = .50250;     //buffer solution 10.0 at 25C
float probeVoltage;         //voltage from probe

pHSensor::pHSensor(ADS1115_MUX inputMux, int iterations)
    : sensorInputMux(inputMux), pHSenseIterations(iterations), analogBufferIndex(0),
    acidVoltage(1.02513), neutralVoltage(.75525), baseVoltage(.50250), probeVoltage(0.0)
{
    // Allocate memory for the analog buffer
    analogBuffer = new float[pHSenseIterations];
    if (analogBuffer == nullptr)
    {
        Serial.println("Failed to allocate memory for analog buffer");
    }
}

pHSensor::~pHSensor()
{
    // Deallocate memory for the analog buffer
    delete[] analogBuffer;
}

float pHSensor::getAverageVoltage() const
{
    return averageVoltage;
}

/**
 * Initializes the PH sensor (sets up ADS1115).
 */
void pHSensor::init()
{
    if (!adcPH.init())
    {
      Serial.println("ADS1115 No 2 (pH Sensor) not connected!");
    }
    adcPH.setVoltageRange_mV(ADS1115_RANGE_6144);
    adcPH.setMeasureMode(ADS1115_CONTINUOUS);
    adcPH.setCompareChannels(sensorInputMux);
}

void pHSensor::analogReadAction()
{
    float voltage = adcPH.getResult_V(); // 10000 Ohm Resistor Cuts Voltage in half
    probeVoltage = voltage*2; // multiply by 2 to get the correct value
    float sensorValue = probeVoltage;
    Serial.println(sensorValue);
    // Store the voltage value in the circular buffer
    analogBuffer[analogBufferIndex] = sensorValue;

    // Update the buffer index
    analogBufferIndex = (analogBufferIndex + 1) % pHSenseIterations;
}

float pHSensor::computeMedian()
{
    float sortedBuffer[pHSenseIterations]; // Temporary array for sorting
    std::copy(analogBuffer, analogBuffer + pHSenseIterations, sortedBuffer);
    std::sort(sortedBuffer, sortedBuffer + pHSenseIterations);

    if (pHSenseIterations % 2 == 0)
    {
        float median = (sortedBuffer[pHSenseIterations / 2 - 1] + sortedBuffer[pHSenseIterations / 2]) / 2.0f;
        return median;
    }
    else
    {
        return sortedBuffer[pHSenseIterations / 2];
    }
}

float pHSensor::adjustpH(float voltage, float temperature)
{
    Serial.print("Probe Voltage: ");
    Serial.println(probeVoltage, 10);
    Serial.print("Temperature: ");
    Serial.println(temperature);

    // Check for potential division by zero
    if (neutralVoltage * 2 == acidVoltage * 2) {
        Serial.println("Error: Division by zero in slope calculation (neutral - acid).");
        return NAN;
    }
    if (neutralVoltage * 2 == baseVoltage * 2) {
        Serial.println("Error: Division by zero in slope calculation (neutral - base).");
        return NAN;
    }

    double slope = ((7.0 - 4.01) / (neutralVoltage * 2 - acidVoltage * 2) + (7 - 10.0) / (neutralVoltage * 2 - baseVoltage * 2)) / 2;

    Serial.print("Slope: ");
    Serial.println(slope, 10);

    if (std::isnan(slope)) {
        Serial.println("Error: Slope is NaN!");
        return NAN;
    }

    double intercept = 7.0 - slope * (neutralVoltage * 2);
    Serial.print("Intercept: ");
    Serial.println(intercept, 10);

    if (std::isnan(intercept)) {
        Serial.println("Error: Intercept is NaN!");
        return NAN;
    }

    double compensatedSlope = slope * ((temperature + 273.15) / (25.0 + 273.15));
    Serial.print("Compensated Slope: ");
    Serial.println(compensatedSlope, 10);

    if (std::isnan(compensatedSlope)) {
        Serial.println("Error: Compensated slope is NaN!");
        return NAN;
    }

    double numerator = compensatedSlope * (probeVoltage);
    double pHValue = numerator + intercept;

    Serial.print("Calculated pH: ");
    Serial.println(pHValue, 10);

    return pHValue;
}


float pHSensor::read(float temperature)
{
    analogReadAction();
    float medianSensorValue = computeMedian();
    averageVoltage = medianSensorValue;
    return adjustpH(averageVoltage, temperature);
}

void pHSensor::shutdown()
{
    // No operation
}

void pHSensor::stabilize()
{
    // Possible for future operation
}