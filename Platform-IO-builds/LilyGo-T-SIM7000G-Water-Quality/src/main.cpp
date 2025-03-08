/*
 * Project: ESP32 Sensor Monitoring with Deep Sleep
 * Description: This code reads data from temperature, TDS, and pH sensors connected to an ESP32,
 *              manages sensor operations using a state machine, and transmits data.
 *              The ESP32 enters deep sleep to conserve power between readings.
 * Author: Philip
 * Written for: ESP32 Dev Board
 */

#include <Wire.h>               // Wire Library for I2C communication
#include <OneWire.h>            // Library for OneWire protocol (used by DS18B20 temperature sensor)
#include <DallasTemperature.h>  // Library for DS18B20 temperature sensor
#include <ADS1115_WE.h>         // ADS1115 Library for TDS and pH sensors
#include "temperature_sensor.h" // Custom Temperature Sensor Class
#include "tds_sensor.h"         // Custom TDS Sensor Class
#include "ph_sensor.h"          // Custom pH Sensor Class
#include "esp_sleep.h"          // ESP32 deep sleep library
#include "sensor_config.h"      // Configuration file with parameters, variables, and constants
#include "sensor_state_machine.h" // State machine to control each sensor
#include "transmit_functions.h"  // I2C transmission functions
#include "debug.h"              // Debugging functions

// Global flags and constants
bool codeExecuted = false;          // Flag to track if code has executed previously
unsigned long sleep_PARAMETER = TIME_TO_SLEEP * S_TO_MIN_FACTOR * uS_TO_S_FACTOR; // Sleep duration in microseconds

// Instantiate sensor objects

// Temperature Sensor: Specify OneWire PIN and the number of samples in the buffer
TemperatureSensor tempSensorInstance(ONE_WIRE_BUS);

// TDS Sensor: Specify kCoefficient, reference temperature, ADC resolution, ADS1115 MUX channel, and buffer size
TdsSensor tdsSensorInstance(0.02, 25.0, TDS_SENSOR_MUX, 15);

// pH Sensor: reference temperature, ADC resolution, ADS1115 MUX channel, and buffer size
pHSensor pHSensorInstance(PH_SENSOR_MUX, 15);

// Create state machine instances for each sensor
SensorStateMachine<TemperatureSensor> tempStateMachine(tempSensorInstance, TEMP_SENSOR_POWER_PIN);
SensorStateMachine<TdsSensor> tdsStateMachine(tdsSensorInstance, TDS_SENSOR_POWER_PIN);
SensorStateMachine<pHSensor> pHStateMachine(pHSensorInstance, PH_SENSOR_POWER_PIN);

void setup() {
    Serial.begin(9600);
    while (!Serial) {;}

    DEBUG_PRINTLN("Initializing ESP32 Sensor Monitoring System...");

    esp_sleep_enable_timer_wakeup(sleep_PARAMETER);

    // Configure GPIOs
    pinMode(TEMP_SENSOR_POWER_PIN, OUTPUT);
    pinMode(TDS_SENSOR_POWER_PIN, OUTPUT);
    pinMode(PH_SENSOR_POWER_PIN, OUTPUT);
    pinMode(ONE_WIRE_BUS, INPUT);

    Wire.begin();  // Initialize I2C

    // Initialize sensors
    tempSensorInstance.init();
    tdsSensorInstance.init();
    pHSensorInstance.init();

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        DEBUG_PRINTLN("Waking up from deep sleep.");
        codeExecuted = true;
    }

    if (!codeExecuted) {
        DEBUG_PRINTLN("Setup Complete.");
        codeExecuted = true;
    }
}

void loop() {
    static bool tempStarted = false;
    static bool tdsStarted = false;
    static bool pHStarted = false;

    tempStateMachine.operate();
    tdsStateMachine.operate();
    pHStateMachine.operate();

    if (!tempStarted) {
        DEBUG_PRINTLN("Starting Temperature Sensor...");
        tempStateMachine.start();
        tempStarted = true;
    }

    if (tempStateMachine.isOff() && !tdsStarted) {
        DEBUG_PRINTLN("Temperature Sensor done. Starting TDS Sensor...");
        tdsStateMachine.start();
        tdsStarted = true;
    }

    if (tempStateMachine.isOff() && tdsStateMachine.isOff() && !pHStarted) {
        DEBUG_PRINTLN("TDS Sensor done. Starting pH Sensor...");
        pHStateMachine.start();
        pHStarted = true;
    }

    if (tempStateMachine.isOff() && tdsStateMachine.isOff() && pHStateMachine.isOff()) {
        DEBUG_PRINTLN("All sensors have finished. Entering deep sleep.");
        DEBUG_PRINT("Uptime (seconds): "); DEBUG_PRINTLN(millis() / 1000);
        delay(2000);
        esp_deep_sleep_start();
    }
}
