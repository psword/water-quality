// ph_sensor.h

#ifndef pHSensor_h
#define pHSensor_h

#include "sensor.h"
#include <ADS1115_WE.h> // ADS1115 Library for TDS and pH sensors

#define I2C_ADDRESS_PH 0x4B // I2C address for PH sensor through the ADS1115

/**
 * pHSensor class, derived from Sensor, to handle pH sensor operations.
 */
class pHSensor : public Sensor
{
private:
    const ADS1115_MUX sensorInputMux; // ADC multiplexer input for TDS sensor
    const int pHSenseIterations;      // Number of pH measurements to retain for buffer

    float *analogBuffer;   // Dynamic array for buffer
    int analogBufferIndex; // Index for circular buffer
    float averageVoltage;  // Store the average voltage

    float acidVoltage = 1.01662;   // buffer solution 4.01 at 25C
    float neutralVoltage = .76050; // buffer solution 7.0 at 25C
    float baseVoltage = .53462;    // buffer solution 10.0 at 25C
    float probeVoltage;            // voltage from probe

public:
    /**
     * Constructor for ph sensor.
     */
    pHSensor(ADS1115_MUX inputMux, int iterations = 10);

    /**
     * Destructor to clean up resources.
     */
    ~pHSensor();

    /**
     * Returns the average voltage.
     * @return The average voltage.
     */
    float getAverageVoltage() const;

    /**
     * Reads analog value from pH sensor and stores in buffer.
     */
    void analogReadAction();

    /**
     * Computes the median reading from the buffer.
     * @return The median reading.
     */
    float computeMedian();

    /**
     * Adjusts pH based on temperature.
     * @param voltage The voltage to adjust.
     * @param temperature The temperature to adjust with.
     * @return The adjusted pH value.
     */
    float adjustpH(float voltage, float temperature);

    /**
     * Computes the pH value.
     * @param temperature The temperature to compute with.
     * @return The computed pH value.
     */
    float read(float temperature) override;

    /**
     * Initializes the sensor, if needed.
     */
    void init() override;

    /**
     * Shuts down the sensor, no operation.
     */
    void shutdown() override;

    /**
     * Stabilizes the sensor with a timer, possible for future operation.
     */
    void stabilize() override;
};

#endif // pHSensor_h