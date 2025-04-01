#ifndef TDSSENSOR_H
#define TDSSENSOR_H

#include "sensor.h"
#include <ADS1115_WE.h> // ADS1115 Library for TDS and pH sensors

#define I2C_ADDRESS_TDS 0x4A // I2C address for the TDS sensor through the ADS1115

// TdsSensor class to read TDS (Total Dissolved Solids) data.
class TdsSensor : public Sensor
{
private:
    const float kCoefficient;         // 2% Coefficient calculation
    const float referenceTemp;        // Reference temperature for the TDS sensor
    const int tdsSenseIterations;     // Number of measurements to retain for buffer
    const ADS1115_MUX sensorInputMux; // ADC multiplexer input for TDS sensor

    float *analogBuffer;                         // Dynamic array for buffer
    int analogBufferIndex;                       // Index for the circular buffer
    float measuredConductivityStandard = 665.76; // Measured conductivity standard for calibration
    float measuredDeionizedWater = 16.23;         // Measured deionized water for calibration
    
public:
    // Constructor for TdsSensor.
    TdsSensor(float kCoeff, float refTemp, ADS1115_MUX inputMux, int iterations = 10);

    // Destructor for TdsSensor.
    ~TdsSensor();

    // Reads analog value from sensor and stores in buffer.
    void analogReadAction();

    // Computes the median reading from the buffer.
    // @return The median value.
    float computeMedian();

    // Adjusts the TDS reading based on temperature.
    float adjustTds(float voltage, float temperature);

    // Reads and adjusts the TDS value.
    float read(float temperature) override;

    // Initializes the sensor.
    void init() override;

    // Shuts down the sensor.
    void shutdown() override;

    // Stabilizes the sensor.
    void stabilize() override;
};

#endif // TDSSENSOR_H
