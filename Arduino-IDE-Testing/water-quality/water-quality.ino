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
#include <DFRobot_ESP_PH.h>     // DFRobot pH Sensor Library v2.0
#include <EEPROM.h>             // EEPROM library for non-volatile storage
#include "temperature_sensor.h" // Custom Temperature Sensor Class
#include "tds_sensor.h"         // Custom TDS Sensor Class
#include "ph_sensor.h"          // Custom pH Sensor Class
#include "esp_sleep.h"          // ESP32 deep sleep library
#include "sensor_config.h"      // Configuration file with parameters, variables, and constants
#include "sensor_state_machine.h" // State machine to control each sensor
#include "transmit_functions.h"  // I2C transmission functions

// Global flags and constants
bool codeExecuted = false;          // Flag to track if code has executed previously
const uint8_t messageType4 = 0x04;  // Message type identifier for sleep command

// Function prototype for deep sleep
void goToDeepSleep();

// Instantiate sensor objects

// Temperature Sensor: Specify OneWire PIN and the number of samples in the buffer
TemperatureSensor tempSensorInstance(ONE_WIRE_BUS);

// TDS Sensor: Specify voltage, kCoefficient, reference temperature, ADC resolution, ADS1115 MUX channel, and buffer size
TdsSensor tdsSensorInstance(3.3, 0.02, 25.0, ADC_BITS, TDS_SENSOR_MUX, 15);

// pH Sensor: Specify voltage (in mV), reference temperature, ADC resolution, ADS1115 MUX channel, and buffer size
pHSensor pHSensorInstance(3300, 25.0, ADC_BITS, PH_SENSOR_MUX, 15);

// Create state machine instances for each sensor
SensorStateMachine<TemperatureSensor> tempStateMachine(tempSensorInstance, TEMP_SENSOR_POWER_PIN);
SensorStateMachine<TdsSensor> tdsStateMachine(tdsSensorInstance, TDS_SENSOR_POWER_PIN);
SensorStateMachine<pHSensor> pHStateMachine(pHSensorInstance, PH_SENSOR_POWER_PIN);

void setup() {
    Serial.begin(9600); // Initialize serial communication for debugging
    // Wait for the serial port to connect
    while (!Serial) {
        ;
    }

    // Set GPIO pins as OUTPUTs for controlling sensor power
    pinMode(TEMP_SENSOR_POWER_PIN, OUTPUT);
    pinMode(TDS_SENSOR_POWER_PIN, OUTPUT);
    pinMode(PH_SENSOR_POWER_PIN, OUTPUT);

    // Set GPIO pins as INPUTs for reading sensor data
    pinMode(ONE_WIRE_BUS, INPUT);

    EEPROM.begin(32);          // Initialize EEPROM with a size of 32 bytes
    Wire.begin();              // Initialize I2C bus as master

    // Initialize sensors
    tempSensorInstance.beginSensors(); // Initialize the temperature sensor library
    pHSensorInstance.beginSensors();   // Initialize the pH sensor library

    // Check if the system is waking up from deep sleep
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("Waking up from deep sleep.");
        codeExecuted = true; // Set flag indicating code has been executed
    }

    // If the code has not been executed yet, enter deep sleep initially
    if (!codeExecuted) {
        Serial.println("Going to deep sleep for the first time.");
        // goToDeepSleep(); // Enter deep sleep mode
    }

    // Note: TDS sensor does not require any additional initialization
    // Additional custom initialization can be added here if needed
    /*
    if (!codeExecuted) {
        delay(10000);
        Serial.println("Setup Complete."); // Print setup complete message to serial monitor
        codeExecuted = true; // Set flag indicating code execution
    }
    */
}

void loop() {
    static bool tempStarted = false;  // Track whether the temperature sensor has started
    static bool tdsStarted = false;   // Track whether the TDS sensor has started
    static bool pHStarted = false;    // Track whether the pH sensor has started

    // Operate each state machine
    tempStateMachine.operate(); // Handle temperature sensor state
    tdsStateMachine.operate();  // Handle TDS sensor state
    pHStateMachine.operate();   // Handle pH sensor state
    
    // Start the temperature sensor state machine if not already started
    if (!tempStarted) {
        tempStateMachine.start();
        tempStarted = true;
    }

    // Transition to the TDS sensor state machine after temperature sensor completes
    if (tempStateMachine.isOff() && !tdsStarted) {
        tdsStateMachine.start();
        tdsStarted = true;
    }

    // Transition to the pH sensor state machine after TDS sensor completes
    if (tempStateMachine.isOff() && tdsStateMachine.isOff() && !pHStarted) {
        pHStateMachine.start();
        pHStarted = true;
    }

    // After all sensors have completed, transmit data and go to deep sleep
    if (tempStateMachine.isOff() && tdsStateMachine.isOff() && pHStateMachine.isOff()) {
        Serial.println("All sensors have completed. This is where I sleep.");
        goToDeepSleep();             // Enter deep sleep mode
    }
}

// Function to enter deep sleep for a specified duration
void goToDeepSleep() {
    // Set the deep sleep timer (e.g., 10 minutes)
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * S_TO_MIN_FACTOR * uS_TO_S_FACTOR); // Define the sleep condition
    Serial.println("Going to deep sleep now.");
    esp_deep_sleep_start(); // Enter deep sleep mode
}
