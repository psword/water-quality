// ph_sensor.cpp

#include "ph_sensor.h"
#include <algorithm>        // Include for std::copy and std::sort
#include <Arduino.h>
#include <DFRobot_ESP_PH.h> // DFRobot pH Library v2.0
#include <EEPROM.h>         // EEPROM library

// Define the global ADS1115 object (must be declared `extern` in the header)
ADS1115_WE adcPH(I2C_ADDRESS_PH);

pHSensor::pHSensor(float voltageConstant, float refTemp, float maxADC, ADS1115_MUX inputMux, int iterations)
    : VC(voltageConstant), referenceTemp(refTemp), maxADCValue(maxADC), sensorInputMux(inputMux), pHSenseIterations(iterations), analogBufferIndex(0)
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

void pHSensor::beginSensors()
{
    ph.begin();
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
    Wire.begin();

    // if (!adcPH.init())
    // {
    //   Serial.println("ADS1115 No 2 (pH Sensor) not connected!");
    // }
    adcPH.setVoltageRange_mV(ADS1115_RANGE_6144);
    // adcPH.setMeasureMode(ADS1115_SINGLE);
    adcPH.setMeasureMode(ADS1115_CONTINUOUS);
    adcPH.setCompareChannels(sensorInputMux);
}

void pHSensor::analogReadAction()
{

    adcPH.setCompareChannels(sensorInputMux); // Ensure correct channel is selected; uncomment if needed
    float sensorValue = adcPH.getRawResult();
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
    float pHValue = ph.readPH(voltage, temperature); // Convert voltage to pH with temp adjustment
    // Serial.println(temperature);
    // Serial.println(voltage);
    // Serial.println(pHValue);
    return pHValue;
}

float pHSensor::read(float temperature)
{
    analogReadAction();
    float medianSensorValue = computeMedian();
    averageVoltage = medianSensorValue * VC / maxADCValue;
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