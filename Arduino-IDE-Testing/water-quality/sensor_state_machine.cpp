// sensor_state_machine.cpp

#include "sensor_state_machine.h"
#include "sensor_config.h"
#include "transmit_functions.h"
#include "debug.h"
#include <Arduino.h>

float adjustedTemp = 0;              // Variable to store temperature
const unsigned long readDelay = 100; // Delay between reads (milliseconds)

template <typename SensorType>
SensorStateMachine<SensorType>::SensorStateMachine(SensorType &sensor, int powerPin)
    : sensor_(sensor), state_(SENSOR_OFF), stateStartTime_(0), powerPin_(powerPin), totalReadTime_(0) 
{
    DEBUG_PRINTLN("SensorStateMachine constructor called");
    DEBUG_PRINT("Power Pin: "); DEBUG_PRINTLN(powerPin_);
}


template <typename SensorType>
void SensorStateMachine<SensorType>::start()
{
    state_ = SENSOR_INIT;
    totalReadTime_ = 0; // Reset the total read time
}

template <typename SensorType>
void SensorStateMachine<SensorType>::operate()
{
    DEBUG_PRINT("Current State: "); DEBUG_PRINTLN(state_);

    switch (state_)
    {
    case SENSOR_OFF:
        DEBUG_PRINTLN("State: SENSOR_OFF - Waiting for activation.");
        break;
    case SENSOR_INIT:
        DEBUG_PRINTLN("State: SENSOR_INIT - Initializing sensor.");
        init();
        break;
    case SENSOR_STABILIZE:
        DEBUG_PRINTLN("State: SENSOR_STABILIZE - Stabilizing sensor.");
        stabilize();
        break;
    case SENSOR_READ:
        DEBUG_PRINTLN("State: SENSOR_READ - Starting read cycle.");
        stateStartTime_ = millis();
        state_ = SENSOR_WAIT;
        break;
    case SENSOR_WAIT:
        DEBUG_PRINTLN("State: SENSOR_WAIT - Waiting for sensor read delay.");
        delay(readDelay);
        totalReadTime_ += millis() - stateStartTime_;

        DEBUG_PRINT("Total Read Time (ms): "); DEBUG_PRINTLN(totalReadTime_);
        
        read();

        if (totalReadTime_ >= READING_DURATION)
        {
            DEBUG_PRINTLN("Reading duration exceeded. Transitioning to shutdown.");
            state_ = SENSOR_SHUTDOWN;
        }
        else
        {
            DEBUG_PRINTLN("Continuing sensor readings.");
            state_ = SENSOR_READ;
        }
        break;
    case SENSOR_SHUTDOWN:
        DEBUG_PRINTLN("State: SENSOR_SHUTDOWN - Shutting down sensor.");
        shutdown();
        break;
    }
}


template <typename SensorType>
void SensorStateMachine<SensorType>::init()
{
    DEBUG_PRINTLN("Initializing sensor: Powering ON.");
    stateStartTime_ = millis();
    digitalWrite(powerPin_, HIGH);
    state_ = SENSOR_STABILIZE;
}

template <typename SensorType>
void SensorStateMachine<SensorType>::stabilize()
{
    DEBUG_PRINTLN("Stabilizing sensor...");
    sensor_.init(); // Initialize the sensor

    if (millis() - stateStartTime_ >= 4000)
    {
        DEBUG_PRINTLN("Sensor initialized. Proceeding to read state.");
        state_ = SENSOR_READ;
    }
    else
    {
        DEBUG_PRINT("Stabilization in progress: "); 
        DEBUG_PRINTLN(millis() - stateStartTime_);
    }
}


template <typename SensorType>
void SensorStateMachine<SensorType>::read()
{
    float sensorValue = sensor_.read(adjustedTemp);
    DEBUG_PRINT("Sensor Read Value: "); DEBUG_PRINTLN(sensorValue);
}


template <>
void SensorStateMachine<TemperatureSensor>::read()
{
    adjustedTemp = sensor_.read(0);
    DEBUG_PRINT("Updated Temperature: "); DEBUG_PRINTLN(adjustedTemp);
}

template <typename SensorType>
void SensorStateMachine<SensorType>::shutdown()
{
    DEBUG_PRINTLN("Shutting down sensor...");
    digitalWrite(powerPin_, LOW);
    state_ = SENSOR_OFF;
}

// TemperatureSensor specialization
template <>
void SensorStateMachine<TemperatureSensor>::shutdown()
{
    DEBUG_PRINTLN("Shutting down TemperatureSensor...");
    digitalWrite(powerPin_, LOW);
    state_ = SENSOR_OFF;
}

// TdsSensor shutdown specialization
template <>
void SensorStateMachine<TdsSensor>::shutdown()
{
    DEBUG_PRINTLN("Shutting down TdsSensor...");
    digitalWrite(powerPin_, LOW);
    state_ = SENSOR_OFF;
}

// pHSensor shutdown specialization
template <>
void SensorStateMachine<pHSensor>::shutdown()
{
    DEBUG_PRINTLN("Shutting down pHSensor...");
    digitalWrite(powerPin_, LOW);
    state_ = SENSOR_OFF;
}

template <typename SensorType>
bool SensorStateMachine<SensorType>::isOff() const
{
    DEBUG_PRINT("Sensor isOff check: "); 
    DEBUG_PRINTLN(state_ == SENSOR_OFF ? "YES" : "NO");
    return state_ == SENSOR_OFF;
}


// Instantiate the templates for each sensor type
template class SensorStateMachine<TemperatureSensor>;
template class SensorStateMachine<TdsSensor>;
template class SensorStateMachine<pHSensor>;
