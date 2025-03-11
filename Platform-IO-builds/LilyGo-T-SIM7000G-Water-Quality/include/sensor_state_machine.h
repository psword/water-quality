// sensor_state_machine.h

#ifndef SensorStateMachine_h
#define SensorStateMachine_h

#include "temperature_sensor.h"
#include "tds_sensor.h"
#include "ph_sensor.h"

enum SensorState
{
    SENSOR_OFF,
    SENSOR_INIT,
    SENSOR_STABILIZE,
    SENSOR_READ,
    SENSOR_WAIT,
    SENSOR_SHUTDOWN
};

template <typename SensorType>
class SensorStateMachine {
public:
    SensorStateMachine(SensorType& sensor, int powerPin);
    void start();
    void operate();
    bool isOff() const; // Check if the state machine is off
private:
    void init();
    void stabilize();
    void read();
    void shutdown();

    SensorType& sensor_;
    SensorState state_;
    unsigned long stateStartTime_;
    int powerPin_;

    // Member variables for sending data
    // float sendFloatValue_;
    unsigned long totalReadTime_;
};

#endif // SensorStateMachine_h