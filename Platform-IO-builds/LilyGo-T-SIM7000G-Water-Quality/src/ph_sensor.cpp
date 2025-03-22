// ph_sensor.cpp

#include "ph_sensor.h"
#include "debug.h"
#include "sensor_config.h"
#include <algorithm> // Include for std::copy and std::sort
#include <Arduino.h>
#include <cmath> // For isnan() and isinf()

// Define the ADS1115 object
ADS1115_WE adcPH(I2C_ADDRESS_PH);

float voltagePh = 0.0; // voltage for pH sensor

pHSensor::pHSensor(ADS1115_MUX inputMux, int iterations)
    : sensorInputMux(inputMux), pHSenseIterations(iterations), analogBufferIndex(0),
      acidVoltage(1.02513), neutralVoltage(.75525), baseVoltage(.50250), probeVoltage(0.0)
{
    DEBUG_PRINTLN("pHSensor constructor called");
    DEBUG_PRINT("Input MUX: ");
    DEBUG_PRINTLN(sensorInputMux);
    DEBUG_PRINT("Iterations: ");
    DEBUG_PRINTLN(pHSenseIterations);
    DEBUG_PRINT("Acid Voltage: ");
    DEBUG_PRINTLN(acidVoltage);
    DEBUG_PRINT("Neutral Voltage: ");
    DEBUG_PRINTLN(neutralVoltage);
    DEBUG_PRINT("Base Voltage: ");
    DEBUG_PRINTLN(baseVoltage);

    // Allocate memory for analog buffer
    analogBuffer = new float[pHSenseIterations];
    if (analogBuffer == nullptr)
    {
        DEBUG_PRINTLN("ERROR: Failed to allocate memory for analog buffer");
    }
    else
    {
        DEBUG_PRINTLN("Memory allocated for analog buffer");

        // Initialize buffer with 0.0 to prevent uninitialized values
        for (int i = 0; i < pHSenseIterations; i++)
        {
            analogBuffer[i] = 0.0;
        }
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

void pHSensor::init()
{
    DEBUG_PRINTLN("Initializing pH sensor...");
    if (!adcPH.init())
    {
        DEBUG_PRINTLN("ERROR: ADS1115 No 2 (pH Sensor) not connected!");
    }
    else
    {
        DEBUG_PRINTLN("ADS1115 initialized successfully.");
    }
    adcPH.setVoltageRange_mV(ADS1115_RANGE_6144);
    adcPH.setMeasureMode(ADS1115_CONTINUOUS);
    adcPH.setCompareChannels(sensorInputMux);
    DEBUG_PRINTLN("pH sensor setup complete.");
}

void pHSensor::analogReadAction()
{
    float voltage = adcPH.getResult_V();
    probeVoltage = voltage * resistorFactor; // Compensate for 10kOhm resistor

    // Check for NaN or infinite values before storing
    if (!isnan(probeVoltage) && !isinf(probeVoltage))
    {
        analogBuffer[analogBufferIndex] = probeVoltage;
        analogBufferIndex = (analogBufferIndex + 1) % pHSenseIterations;

        DEBUG_PRINT("Raw ADC Voltage: ");
        DEBUG_PRINTLN(voltage);
        DEBUG_PRINT("Probe Voltage (adjusted): ");
        DEBUG_PRINTLN(probeVoltage);
    }
    else
    {
        DEBUG_PRINTLN("WARNING: Invalid ADC reading, skipping storage.");
    }
}

float pHSensor::computeMedian()
{
    float sortedBuffer[pHSenseIterations];
    std::copy(analogBuffer, analogBuffer + pHSenseIterations, sortedBuffer);
    std::sort(sortedBuffer, sortedBuffer + pHSenseIterations);

    DEBUG_PRINTLN("Sorted analog buffer:");
    for (int i = 0; i < pHSenseIterations; i++)
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
    if (pHSenseIterations % 2 == 0)
    {
        medianValue = (sortedBuffer[pHSenseIterations / 2 - 1] + sortedBuffer[pHSenseIterations / 2]) / 2.0f;
    }
    else
    {
        medianValue = sortedBuffer[pHSenseIterations / 2];
    }

    DEBUG_PRINT("Computed Median: ");
    DEBUG_PRINTLN(medianValue);
    return medianValue;
}

float pHSensor::adjustpH(float voltage, float temperature)
{
    DEBUG_PRINT("Adjusting pH with voltage: ");
    DEBUG_PRINT(voltage);
    voltagePh = voltage; // Store the voltage for logging
    DEBUG_PRINT(" and temperature: ");
    DEBUG_PRINTLN(temperature);

    if (isnan(voltage) || isinf(voltage))
    {
        DEBUG_PRINTLN("ERROR: Voltage is NaN or infinite!");
        return NAN;
    }

    if (neutralVoltage * resistorFactor == acidVoltage * resistorFactor)
    {
        DEBUG_PRINTLN("ERROR: Division by zero in slope calculation (neutral - acid).");
        return NAN;
    }
    if (neutralVoltage * resistorFactor == baseVoltage * resistorFactor)
    {
        DEBUG_PRINTLN("ERROR: Division by zero in slope calculation (neutral - base).");
        return NAN;
    }

    double slope = ((7.0 - 4.01) / (neutralVoltage * resistorFactor - acidVoltage * resistorFactor) +
                    (7 - 10.0) / (neutralVoltage * resistorFactor - baseVoltage * resistorFactor)) /
                   2;

    if (isnan(slope) || isinf(slope))
    {
        DEBUG_PRINTLN("ERROR: Slope is NaN or infinite!");
        return NAN;
    }

    double intercept = 7.0 - slope * (1.50);

    if (isnan(intercept) || isinf(intercept))
    {
        DEBUG_PRINTLN("ERROR: Intercept is NaN or infinite!");
        return NAN;
    }

    double compensatedSlope = slope * ((temperature + 273.15) / (25.0 + 273.15));

    if (isnan(compensatedSlope) || isinf(compensatedSlope))
    {
        DEBUG_PRINTLN("ERROR: Compensated slope is NaN or infinite!");
        return NAN;
    }

    double pHValue = compensatedSlope * voltage + intercept;

    if (isnan(pHValue) || isinf(pHValue))
    {
        DEBUG_PRINTLN("ERROR: Computed pH is NaN or infinite!");
        return NAN;
    }

    DEBUG_PRINT("Final Calculated pH: ");
    DEBUG_PRINTLN(pHValue);
    return pHValue;
}

float pHSensor::read(float temperature)
{
    DEBUG_PRINTLN("Starting pH reading...");
    analogReadAction();

    float medianSensorValue = computeMedian();
    if (isnan(medianSensorValue) || isinf(medianSensorValue))
    {
        DEBUG_PRINTLN("ERROR: Median voltage is invalid, returning NaN.");
        return NAN;
    }

    averageVoltage = medianSensorValue;
    float pHValue = adjustpH(averageVoltage, temperature);
    float finalpH = pHValue;

    DEBUG_PRINT("Final pH Value: ");
    DEBUG_PRINTLN(pHValue);
    return pHValue;
}

void pHSensor::shutdown()
{
    // No operation
}

void pHSensor::stabilize()
{
    // Possible for future operation
}
