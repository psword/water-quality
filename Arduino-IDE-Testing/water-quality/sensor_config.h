/*
 * File: sensor_config.h
 * Description: Header file containing pin definitions, global variables, and configuration constants
 *              for the sensor monitoring system using ESP32.
 * Author: Philip
 */

#ifndef SENSOR_CONFIG
#define SENSOR_CONFIG

// Serial Configuration
#define SerialMon Serial // Set serial for debug console (to the Serial Monitor)
#define SerialAT Serial1 // Communicate with SIM7000G via Serial1 (AT Commands)

// Sensor Pin Definitions

static constexpr int ONE_WIRE_BUS = 5;           // GPIO pin for the OneWire data line (DS18B20 temperature sensor)
static constexpr int TEMP_SENSOR_POWER_PIN = 18; // GPIO pin for controlling power to the temperature sensor
#define TDS_SENSOR_MUX ADS1115_COMP_0_GND        // ADS1115 multiplexer channel for TDS sensor
static constexpr int TDS_SENSOR_POWER_PIN = 19;  // GPIO pin for controlling power to the TDS sensor
static constexpr int PH_SENSOR_POWER_PIN = 23;   // GPIO pin for controlling power to the pH sensor
#define PH_SENSOR_MUX ADS1115_COMP_0_GND         // GPIO pin for the pH sensor data line

// Global Variables

// SD Card Configuration
static constexpr int SD_CS = 13;   // Chip Select pin for the SD card module
static constexpr int SD_MOSI = 15; // SPI MOSI pin for the SD card module
static constexpr int SD_MISO = 2;  // SPI MISO pin for the SD card module
static constexpr int SD_SCLK = 14; // SPI SCLK pin for the SD card module

// LilyGO T-SIM7000G Pinout
static constexpr int UART_BAUD = 115200; // Baud rate for UART communication
static constexpr int PIN_DTR = 25;       // DTR pin for SIM7000G
static constexpr int PIN_TX = 27;        // TX pin for SIM7000G
static constexpr int PIN_RX = 26;        // RX pin for SIM7000G
static constexpr int PWR_PIN = 4;        // Power pin for SIM7000G

// Others
extern float finalTemp;               // Global variable to store the final temperature value (write to SD)
extern float finalTds;                // Global variable to store the final TDS value (write to SD)
extern float rawTds;                  // Global variable to store the raw TDS value (write to SD)
extern float voltageTds;              // Global variable to store the voltage value for TDS sensor (write to SD)
extern float finalpH;                 // Global variable to store the final pH value (write to SD)
extern float voltagePh;               // Global variable to store the voltage value for pH sensor
extern float adjustedTemp;            // Global variable to store the adjusted temperature value
extern bool codeExecuted;             // Flag to indicate whether the code has executed before
extern const unsigned long readDelay; // Global constant for the delay between sensor reads (milliseconds)

// Time and Sleep Configuration

#define READING_DURATION 4000                        // Duration in milliseconds for sensor sampling
static const unsigned long uS_TO_S_FACTOR = 1000000; // Conversion factor from microseconds to seconds
static const int S_TO_MIN_FACTOR = 60;               // Conversion factor from seconds to minutes
static const int TIME_TO_SLEEP = 2;                 // Duration in minutes for deep sleep between readings

#endif // SENSOR_CONFIG
