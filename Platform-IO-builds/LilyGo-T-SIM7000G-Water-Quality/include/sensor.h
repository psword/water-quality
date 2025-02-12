// sensor.h

#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
public:
    virtual void init() = 0;        // Pure virtual function to initialize the sensor
    virtual void shutdown() = 0;    // Pure virtual function to shut down the sensor
    virtual void stabilize() = 0;   // Pure virtual function to stabilize the sensor
    virtual float read(float temperature = 0.0) = 0; // Pure virtual function to read the sensor; default parameter for temperature
    virtual ~Sensor() {} // Virtual destructor to ensure proper cleanup
};

#endif // SENSOR_H
