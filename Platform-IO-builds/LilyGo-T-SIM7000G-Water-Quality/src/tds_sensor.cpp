#include "tds_sensor.h"
#include <Arduino.h>
#include <algorithm> // Include for std::copy and std::sort

// Define the global ADS1115 object (must be declared `extern` in the header)
ADS1115_WE adcTDS(I2C_ADDRESS_TDS);

/**
 * Constructor for TdsSensor.
 */
TdsSensor::TdsSensor(float voltageConstant, float kCoeff, float refTemp, float maxADC, ADS1115_MUX inputMux, int iterations)
    : VC(voltageConstant), kCoefficient(kCoeff), referenceTemp(refTemp), maxADCValue(maxADC), sensorInputMux(inputMux), tdsSenseIterations(iterations), analogBufferIndex(0)
{
    // Allocate memory for the analog buffer
    analogBuffer = new float[tdsSenseIterations];
    if (analogBuffer == nullptr)
    {
        Serial.println("Failed to allocate memory for analog buffer");
    }
}

/**
 * Destructor for TdsSensor.
 */
TdsSensor::~TdsSensor()
{
    delete[] analogBuffer;
}

/**
 * Initializes the TDS sensor (sets up ADS1115).
 */
void TdsSensor::init()
{
    Wire.begin();
    // if (!adcTDS.init())
    // {
    //   Serial.println("ADS1115 No 1 (Tds Sensor) not connected!");
    // }
    adcTDS.setVoltageRange_mV(ADS1115_RANGE_6144);
    // adcTDS.setMeasureMode(ADS1115_SINGLE);
    adcTDS.setMeasureMode(ADS1115_CONTINUOUS);
    adcTDS.setCompareChannels(sensorInputMux);
}

/**
 * Reads analog value from sensor and stores in buffer.
 */
void TdsSensor::analogReadAction()
{
    adcTDS.setCompareChannels(sensorInputMux); // Ensure correct channel is selected; uncomment if needed
    float sensorValue = adcTDS.getRawResult();
    // Serial.println(sensorValue);
    analogBuffer[analogBufferIndex] = sensorValue;
    analogBufferIndex = (analogBufferIndex + 1) % tdsSenseIterations; // Circular buffer
}

/**
 * Computes the median reading from the buffer.
 */
float TdsSensor::computeMedian()
{
    float sortedBuffer[tdsSenseIterations]; // Temporary array for sorting
    std::copy(analogBuffer, analogBuffer + tdsSenseIterations, sortedBuffer);
    std::sort(sortedBuffer, sortedBuffer + tdsSenseIterations);

    if (tdsSenseIterations % 2 == 0)
    {
        return (sortedBuffer[tdsSenseIterations / 2 - 1] + sortedBuffer[tdsSenseIterations / 2]) / 2.0f;
    }
    else
    {
        return sortedBuffer[tdsSenseIterations / 2];
    }
}

/**
 * Adjusts the TDS reading based on temperature.
 */
float TdsSensor::adjustTds(float voltage, float temperature)
{
    float tempCorrection = 1.0 + kCoefficient * (temperature - referenceTemp);
    float compensationVoltage = voltage / tempCorrection;

    return (133.42 / compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
}

/**
 * Reads and adjusts the TDS value.
 */
float TdsSensor::read(float temperature)
{
    analogReadAction();
    float medianSensorValue = computeMedian();
    float voltage = (medianSensorValue / maxADCValue) * VC;
    return adjustTds(voltage, temperature);
}

/**
 * Shuts down the sensor.
 */
void TdsSensor::shutdown()
{
    // No specific shutdown needed
    // add 'sensor_.shutdown(); // Ensure ADS1115 is properly shut down'
    // in the shutdown method of the SensorStateMachine as first statement
}

/**
 * Stabilizes the sensor.
 */
void TdsSensor::stabilize()
{
    // Possible future implementation for stabilization
    // add 'sensor_.stabilize(); // Ensure ADS1115 is properly stabilized'
    // in the stabilize method of the SensorStateMachine as first statement
}
