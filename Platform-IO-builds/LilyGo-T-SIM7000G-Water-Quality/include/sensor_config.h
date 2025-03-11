/*
 * File: sensor_config.h
 * Description: Header file containing pin definitions, global variables, and configuration constants
 *              for the sensor monitoring system using ESP32.
 * Author: Philip
 */

#ifndef SENSOR_CONFIG
#define SENSOR_CONFIG

// Pin Definitions

#define ONE_WIRE_BUS 5                      // GPIO pin for the OneWire data line (DS18B20 temperature sensor)
#define TEMP_SENSOR_POWER_PIN 18             // GPIO pin for controlling power to the temperature sensor
#define TDS_SENSOR_MUX ADS1115_COMP_0_GND   // ADS1115 multiplexer channel for TDS sensor
#define TDS_SENSOR_POWER_PIN 19             // GPIO pin for controlling power to the TDS sensor
#define PH_SENSOR_POWER_PIN 23              // GPIO pin for controlling power to the pH sensor
#define PH_SENSOR_MUX ADS1115_COMP_0_GND    // GPIO pin for the pH sensor data line

// Global Variables

extern float adjustedTemp;               // Global variable to store the adjusted temperature value
extern bool codeExecuted;                // Flag to indicate whether the code has executed before
extern const unsigned long readDelay;    // Global constant for the delay between sensor reads (milliseconds)

// Time and Sleep Configuration

#define READING_DURATION 4000           // Duration in milliseconds for sensor sampling
#define uS_TO_S_FACTOR 1000000           // Conversion factor from microseconds to seconds
#define S_TO_MIN_FACTOR 60               // Conversion factor from seconds to minutes
#define TIME_TO_SLEEP 2                  // Duration in minutes for deep sleep between readings

#endif // SENSOR_CONFIG
