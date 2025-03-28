// tds_sensor.cpp

#include "tds_sensor.h"
#include "debug.h"
#include "sensor_config.h"
#include <Arduino.h>
#include <algorithm> // Include for std::copy and std::sort
#include <cmath>     // For isnan() and isinf()

// Define the ADS1115 object
ADS1115_WE adcTDS(I2C_ADDRESS_TDS);

float rawTds = 0.0; // Raw TDS value
float voltageTds = 0.0; // Voltage for TDS sensor

// Constructor for TdsSensor
TdsSensor::TdsSensor(float kCoeff, float refTemp, ADS1115_MUX inputMux, int iterations)
    : kCoefficient(kCoeff), referenceTemp(refTemp), sensorInputMux(inputMux), tdsSenseIterations(iterations), analogBufferIndex(0)
{
    DEBUG_PRINTLN("TdsSensor constructor called");
    DEBUG_PRINT("kCoefficient: ");
    DEBUG_PRINTLN(kCoeff);
    DEBUG_PRINT("referenceTemp: ");
    DEBUG_PRINTLN(refTemp);
    DEBUG_PRINT("sensorInputMux: ");
    DEBUG_PRINTLN(sensorInputMux);
    DEBUG_PRINT("tdsSenseIterations: ");
    DEBUG_PRINTLN(tdsSenseIterations);

    analogBuffer = new float[tdsSenseIterations];
    if (analogBuffer == nullptr)
    {
        DEBUG_PRINTLN("Failed to allocate memory for analog buffer");
    }
    else
    {
        DEBUG_PRINTLN("Memory allocated for analog buffer");
        for (int i = 0; i < tdsSenseIterations; i++)
        {
            analogBuffer[i] = 0.0;
        }
    }
}

// Destructor for TdsSensor
TdsSensor::~TdsSensor()
{
    delete[] analogBuffer;
}

// Initializes the TDS sensor (sets up ADS1115)
void TdsSensor::init()
{
    DEBUG_PRINTLN("Initializing TDS sensor...");
    if (!adcTDS.init())
    {
        DEBUG_PRINTLN("ERROR: ADS1115 No 1 (Tds Sensor) not connected!");
    }
    else
    {
        DEBUG_PRINTLN("ADS1115 initialized successfully.");
    }
    adcTDS.setVoltageRange_mV(ADS1115_RANGE_6144);
    adcTDS.setMeasureMode(ADS1115_CONTINUOUS);
    adcTDS.setCompareChannels(sensorInputMux);
    DEBUG_PRINTLN("TDS sensor setup complete.");
}

// Reads analog value from sensor and stores in buffer
void TdsSensor::analogReadAction()
{
    double voltage = adcTDS.getResult_V();
    double probeVoltage = voltage * 3.33457692; // Adjust for 10kOhm resistor

    // Check for NaN or infinite values before storing
    if (!isnan(probeVoltage) && !isinf(probeVoltage))
    {
        analogBuffer[analogBufferIndex] = probeVoltage;
        analogBufferIndex = (analogBufferIndex + 1) % tdsSenseIterations; // Circular buffer

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

// Computes the median reading from the buffer
float TdsSensor::computeMedian()
{
    float sortedBuffer[tdsSenseIterations];
    std::copy(analogBuffer, analogBuffer + tdsSenseIterations, sortedBuffer);
    std::sort(sortedBuffer, sortedBuffer + tdsSenseIterations);

    DEBUG_PRINTLN("Sorted analog buffer:");
    for (int i = 0; i < tdsSenseIterations; i++)
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
    if (tdsSenseIterations % 2 == 0)
    {
        medianValue = (sortedBuffer[tdsSenseIterations / 2 - 1] + sortedBuffer[tdsSenseIterations / 2]) / 2.0f;
    }
    else
    {
        medianValue = sortedBuffer[tdsSenseIterations / 2];
    }

    DEBUG_PRINT("Computed Median: ");
    DEBUG_PRINTLN(medianValue);
    return medianValue;
}

// Adjusts the TDS reading based on temperature
float TdsSensor::adjustTds(float voltage, float temperature)
{
    DEBUG_PRINT("Adjusting TDS with voltage: ");
    DEBUG_PRINT(voltage);
    DEBUG_PRINT(" and temperature: ");
    DEBUG_PRINTLN(temperature);

    if (isnan(voltage) || isinf(voltage))
    {
        DEBUG_PRINTLN("ERROR: Voltage is NaN or infinite!");
        return NAN;
    }

    voltageTds = voltage; // Store the voltage for logging
    float rawEC = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage);

    if (isnan(rawEC) || isinf(rawEC))
    {
        DEBUG_PRINTLN("ERROR: rawEC is NaN or infinite!");
        return NAN;
    }
    
    float tempCorrection = 1.0 + kCoefficient * (temperature - referenceTemp);

    if (isnan(tempCorrection) || isinf(tempCorrection))
    {
        DEBUG_PRINTLN("ERROR: tempCorrection is NaN or infinite!");
        return NAN;
    }
    
    float compensatedEC = rawEC / tempCorrection;

    if (isnan(compensatedEC) || isinf(compensatedEC))
    {
        DEBUG_PRINTLN("ERROR: compensatedEC is NaN or infinite!");
        return NAN;
    }
    
    rawTds = compensatedEC * 0.5;

    if (measuredConductivityStandard == measuredDeionizedWater)
    {
        DEBUG_PRINTLN("ERROR: Division by zero in slope calculation (conductivityStandard - diWater).");
        return NAN;
    }
    
    float slope = (706.5 - 2.5) / (measuredConductivityStandard - measuredDeionizedWater);

    if (isnan(slope) || isinf(slope))
    {
        DEBUG_PRINTLN("ERROR: Slope is NaN or infinite!");
        return NAN;
    }
    
    float intercept = 0 - slope * measuredDeionizedWater;

    if (isnan(intercept) || isinf(intercept))
    {
        DEBUG_PRINTLN("ERROR: Intercept is NaN or infinite!");
        return NAN;
    }
    
    float correctedTds = slope * rawTds + intercept;

    DEBUG_PRINT("Temperature Correction Factor: ");
    DEBUG_PRINTLN(tempCorrection);
    DEBUG_PRINT("Compensated EC: ");
    DEBUG_PRINTLN(compensatedEC);
    DEBUG_PRINT("Raw TDS: ");
    DEBUG_PRINTLN(rawTds);
    DEBUG_PRINTLN("Calibration Adjustment:");
    DEBUG_PRINT("Slope: ");
    DEBUG_PRINTLN(slope);
    DEBUG_PRINT("Intercept: ");
    DEBUG_PRINTLN(intercept);
    DEBUG_PRINT("Corrected TDS: ");
    DEBUG_PRINTLN(correctedTds);

    return correctedTds;
}

// Reads and adjusts the TDS value
float TdsSensor::read(float temperature)
{
    DEBUG_PRINTLN("Starting TDS reading...");
    analogReadAction();

    float medianSensorValue = computeMedian();
    float voltage = medianSensorValue;
    float tdsValue = adjustTds(voltage, temperature);

    DEBUG_PRINT("Final TDS Value: ");
    DEBUG_PRINTLN(tdsValue);
    return tdsValue;
}

// Shuts down the sensor
void TdsSensor::shutdown()
{
    // No specific shutdown needed
}

// Stabilizes the sensor
void TdsSensor::stabilize()
{
    // Possible future implementation for stabilization
}
