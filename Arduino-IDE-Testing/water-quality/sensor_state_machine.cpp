// sensor_state_machine.cpp

#include "sensor_state_machine.h"
#include "sensor_config.h"
#include "transmit_functions.h"
#include <Arduino.h>

float adjustedTemp = 0;              // Variable to store temperature
const unsigned long readDelay = 650; // Delay between reads (milliseconds)

template <typename SensorType>
SensorStateMachine<SensorType>::SensorStateMachine(SensorType &sensor, int powerPin)
    : sensor_(sensor), state_(SENSOR_OFF), stateStartTime_(0), powerPin_(powerPin), totalReadTime_(0) {}

template <typename SensorType>
void SensorStateMachine<SensorType>::start()
{
    state_ = SENSOR_INIT;
    totalReadTime_ = 0; // Reset the total read time
}

template <typename SensorType>
void SensorStateMachine<SensorType>::operate()
{
    switch (state_)
    {
    case SENSOR_OFF:
        // Do nothing until instructed to turn on the sensor
        break;
    case SENSOR_INIT:
        init();
        break;
    case SENSOR_STABILIZE:
        stabilize();
        break;
    case SENSOR_READ:
        timer_.setInterval(readDelay); // Set the timer interval
        state_ = SENSOR_WAIT;
        timer_.reset(); // Reset the timer
        break;
    case SENSOR_WAIT:
        if (timer_.isReady())
        { // Check if the timer is ready
            read(); // Call the read function
            totalReadTime_ += readDelay; // Increment the total read time
            if (totalReadTime_ >= READING_DURATION)
            { // Check if the total read time exceeds the reading duration
                state_ = SENSOR_SHUTDOWN;
            }
            else
            {
                state_ = SENSOR_READ; // Go back to the read state
            }
        }
        break;
    case SENSOR_SHUTDOWN:
        shutdown();
        break;
    }
}

template <typename SensorType>
void SensorStateMachine<SensorType>::init()
{
    stateStartTime_ = millis();
    digitalWrite(powerPin_, HIGH); // Power on the sensor
    state_ = SENSOR_STABILIZE;
}

template <typename SensorType>
void SensorStateMachine<SensorType>::stabilize()
{
    sensor_.init(); // Initialize the sensor (ADS1115 setup for TdsSensor and pHSensor)
    if (millis() - stateStartTime_ >= 4000)
    { // Wait for 4 seconds
        Serial.println("Sensor initialized."); // Uncomment for debugging
        state_ = SENSOR_READ;
    }
}

template <typename SensorType>
void SensorStateMachine<SensorType>::read()
{
    float sensorValue = sensor_.read(adjustedTemp);
    Serial.println(sensorValue); // Uncomment for debugging
}

// TemperatureSensor specialization
template <>
void SensorStateMachine<TemperatureSensor>::shutdown()
{
    digitalWrite(powerPin_, LOW); // Power off the sensor
    // if (sendMessageFlag_)
    // {
    //     sendMessageFlag_ = false;
    // }
    state_ = SENSOR_OFF;
}

// TdsSensor shutdown specialization
template <>
void SensorStateMachine<TdsSensor>::shutdown()
{
    digitalWrite(powerPin_, LOW); // Power off the sensor
    // if (sendMessageFlag_)
    // {
    //     sendMessageFlag_ = false;
    // }
    state_ = SENSOR_OFF;
}

// pHSensor shutdown specialization
template <>
void SensorStateMachine<pHSensor>::shutdown()
{
    digitalWrite(powerPin_, LOW); // Power off the sensor
    // if (sendMessageFlag_)
    // {
    //     sendMessageFlag_ = false;
    // }
    state_ = SENSOR_OFF;
}

template <typename SensorType>
bool SensorStateMachine<SensorType>::isOff() const
{
    return state_ == SENSOR_OFF;
}

// Instantiate the templates for each sensor type
template class SensorStateMachine<TemperatureSensor>;
template class SensorStateMachine<TdsSensor>;
template class SensorStateMachine<pHSensor>;
