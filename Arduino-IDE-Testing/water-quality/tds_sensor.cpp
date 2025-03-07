#include "tds_sensor.h"
#include <Arduino.h>
#include <algorithm> // Include for std::copy and std::sort

// Define the ADS1115 object
ADS1115_WE adcTDS(I2C_ADDRESS_TDS);

/**
 * Constructor for TdsSensor.
 */
TdsSensor::TdsSensor(float kCoeff, float refTemp, ADS1115_MUX inputMux, int iterations)
    : kCoefficient(kCoeff), referenceTemp(refTemp), sensorInputMux(inputMux), tdsSenseIterations(iterations), analogBufferIndex(0)
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
    if (!adcTDS.init())
    {
      Serial.println("ADS1115 No 1 (Tds Sensor) not connected!");
    }
    adcTDS.setVoltageRange_mV(ADS1115_RANGE_6144);
    adcTDS.setMeasureMode(ADS1115_CONTINUOUS);
    adcTDS.setCompareChannels(sensorInputMux);
}

/**
 * Reads analog value from sensor and stores in buffer.
 */
void TdsSensor::analogReadAction()
{
    float voltage = adcTDS.getResult_V(); // 10000 Ohm Resistor Cuts Voltage in half
    float probeVoltage = voltage*2;
    float sensorValue = probeVoltage; // Result in V
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
    float rawTds = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * 0.5;
    float correctedTds = 1.1297 * rawTds - 9.52; // apply calibration scaler
    return correctedTds;
}

/**
 * Reads and adjusts the TDS value.
 */
float TdsSensor::read(float temperature)
{
    analogReadAction();
    float medianSensorValue = computeMedian();
    float voltage = medianSensorValue;
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
