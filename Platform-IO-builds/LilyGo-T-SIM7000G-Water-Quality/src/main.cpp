/*
 * Project: ESP32 Sensor Monitoring with Deep Sleep
 * Description: This code reads data from temperature, TDS, and pH sensors connected to an ESP32,
 *              manages sensor operations using a state machine, and transmits data.
 *              The ESP32 enters deep sleep to conserve power between readings.
 * Author: Philip
 * Written for: LilyGo-T-SIM7000G
 */

#include <Wire.h>                 // Wire Library for I2C communication
#include <OneWire.h>              // Library for OneWire protocol (used by DS18B20 temperature sensor)
#include <DallasTemperature.h>    // Library for DS18B20 temperature sensor
#include <ADS1115_WE.h>           // ADS1115 Library for TDS and pH sensors
#include "temperature_sensor.h"   // Custom Temperature Sensor Class
#include "tds_sensor.h"           // Custom TDS Sensor Class
#include "ph_sensor.h"            // Custom pH Sensor Class
#include "esp_sleep.h"            // ESP32 deep sleep library
#include "sensor_config.h"        // Configuration file with parameters, variables, and constants
#include "sensor_state_machine.h" // State machine to control each sensor
#include "debug.h"                // Debugging functions
#include "secrets.h"              // File containing sensitive information (e.g., credentials)
#include <uRTCLib.h>              // Micro Real-Time Clock Library
#include <SPI.h>                  // SPI Library for SD Card
#include <SD.h>                   // SD Card Library
#include <FS.h>                   // File System Library

// Modem Configuration
#define TINY_GSM_MODEM_SIM7000 // Define the modem
// #define TINY_GSM_USE_GPRS true // Use GPRS for data connection
#define SIM_PIN ""             // SIM card PIN (leave empty if not defined)
#include <TinyGsmClient.h>     // TinyGSM Library for GSM communication
#include <PubSubClient.h>      // MQTT Library for ESP32
#include <ArduinoJson.h>       // JSON Library for ESP32
TinyGsm modem(Serial1);
TinyGsmClient client(modem); // TinyGSM client

PubSubClient mqtt(client); // MQTT client

// SD Variables
char daysOfTheWeek[7][12] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"}; // Array of days of the week
const char *directory = "/logs";                                                                            // Directory path for sensor data logs
const char *filename = "/logs/sensor_log.txt";                                                              // File path for sensor data log

// Global flags and constants
bool codeExecuted = false;                                                        // Flag to track if code has executed previously
bool dataLogged = false;                                                          // Flag to track if data has been logged
RTC_DATA_ATTR u_int32_t bootCounter = 0;                                          // Counter to track number of boots
static const uint64_t sleep_PARAMETER = (uint64_t)TIME_TO_SLEEP * S_TO_MIN_FACTOR * uS_TO_S_FACTOR; // Sleep duration in microseconds

// Instantiate sensor objects
// Temperature Sensor: Specify OneWire PIN and the number of samples in the buffer
TemperatureSensor tempSensorInstance(ONE_WIRE_BUS);

// TDS Sensor: Specify kCoefficient, reference temperature, ADS1115 MUX channel, and buffer size
TdsSensor tdsSensorInstance(0.019, 25.0, TDS_SENSOR_MUX, 10);

// pH Sensor: ADS1115 MUX channel, and buffer size
pHSensor pHSensorInstance(PH_SENSOR_MUX, 10);

// Create state machine instances for each sensor
SensorStateMachine<TemperatureSensor> tempStateMachine(tempSensorInstance, TEMP_SENSOR_POWER_PIN);
SensorStateMachine<TdsSensor> tdsStateMachine(tdsSensorInstance, TDS_SENSOR_POWER_PIN);
SensorStateMachine<pHSensor> pHStateMachine(pHSensorInstance, PH_SENSOR_POWER_PIN);

// Instantiate uRTCLib object

// RTC object: Specify I2C address,  // Comment out below line once you set the date & time - place in setup().
// Following line sets the RTC with an explicit date & time
// to set January 13 2022 at 12:56 you would call:
// example -> rtc.set(second, minute, hour, dayOfWeek, dayOfMonth, month, year)
// OR rtc.set(0, 56, 12, 5, 13, 1, 22);
// set day of week (1=Monday, 7=Sunday)
uRTCLib rtc(0x68);                    // RTC object (default I2C address)
byte rtcModel = URTCLIB_MODEL_DS3231; // RTC model

// Basic logging function to SD Card
void logData(fs::FS &fs, String filename, const char *data)
{
    File file = fs.open(filename, FILE_APPEND);
    if (file)
    {
        file.print(data); // Write the data to the file
        file.flush();     // Ensure data is written before closing
        file.close();     // Close the file
        DEBUG_PRINT("Data Written: ");
        DEBUG_PRINTLN(data);
    }
    else
    {
        DEBUG_PRINTLN("Failed to open file for writing!");
    }
}

// Function to create a directory
void createDir(fs::FS &fs, const char *path)
{
    DEBUG_PRINTF("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
        DEBUG_PRINTLN("Dir created");
    }
    else
    {
        DEBUG_PRINTLN("mkdir failed");
    }
}

// Function to log data with timestamp, call logData() to write to file
void logDataWithTimestamp(fs::FS &fs, String filename, uRTCLib rtc)
{

    char logEntry[256]; // Buffer to store formatted log string

    rtc.refresh();

    // Time Format: YY/MM/DD Day HH:MM:SS
    sprintf(logEntry, "%02d/%02d/%02d %s %02d:%02d:%02d; finalTemp: %.2f; finalTDS: %.2f; rawTDS: %.2f; voltageTDS: %.2f; finalpH: %.2f; voltagePh: %.2f\n",
            rtc.year(), rtc.month(), rtc.day(),
            daysOfTheWeek[rtc.dayOfWeek() - 1], // Adjust day index (1-7 → 0-6)
            rtc.hour(), rtc.minute(), rtc.second(), finalTemp, finalTds, rawTds, voltageTds, finalpH, voltagePh);
    DEBUG_PRINTF("Buffer Length: %d\n", strlen(logEntry)); // Check if exceeding 128 bytes
    // Write the formatted log entry to the file
    logData(fs, filename, logEntry);
}

// Function to initialize SD Card
void intializeSDCard(const int SD_SCLK, const int SD_MISO, const int SD_MOSI, const int SD_CS)
{
    DEBUG_PRINTLN("========SDCard Detect.======");
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);
    if (!SD.begin(SD_CS))
    {
        DEBUG_PRINTLN("SDCard MOUNT FAIL");
    }
    else
    {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        String str = "SDCard Size: " + String(cardSize) + "MB";
        DEBUG_PRINTLN(str);
    }
    DEBUG_PRINTLN("===========================");
}

// RTC Verify
String rtcVerify(uRTCLib rtc)
{
    char timeStamp[128]; // Buffer to store timeStamp test
    sprintf(timeStamp, "%02d/%02d/%02d %s %02d:%02d:%02d",
            rtc.year(), rtc.month(), rtc.day(),
            daysOfTheWeek[rtc.dayOfWeek() - 1], // Adjust day index (1-7 → 0-6)
            rtc.hour(), rtc.minute(), rtc.second());
    return String(timeStamp);
}

// Functions for Modem
void modemPowerOn()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1500);
    digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(3000);
    digitalWrite(PWR_PIN, HIGH);
}

void modemRestart()
{
    modemPowerOff();
    delay(1000);
    modemPowerOn();
}

bool verifyGPRS()
{
    if (!modem.isGprsConnected())
    {
        DEBUG_PRINTLN("GPRS disconnected! Attempting reconnection...");
        if (!modem.gprsConnect(apn, gprsUser, gprsPass))
        {
            DEBUG_PRINTLN("GPRS connection failed");
            return false;
        }
        DEBUG_PRINTLN("GPRS reconnected");
    }
    return true;
}

// Functions to send data via MQTT
void sendMQTT()
{
    JsonDocument doc; // Allocate JSON document

    doc["Device"] = "LilyGo-T-SIM7000G";
    doc["Timestamp"] = rtcVerify(rtc);
    doc["Temp"] = finalTemp;
    doc["TDS"] = finalTds;
    doc["rawTDS"] = rawTds;
    doc["voltageTDS"] = voltageTds;
    doc["pH"] = finalpH;
    doc["voltagePh"] = voltagePh;
    doc["bootCounter"] = bootCounter;
    doc["cycleTime"] = millis() / 1000 + 7; // Add 7 seconds to account for delays

    char jsonBuffer[256];           // Buffer to hold the JSON string
    serializeJson(doc, jsonBuffer); // Convert the JSON object to string
    DEBUG_PRINTLN("Publishing JSON:");
    DEBUG_PRINTLN(jsonBuffer); // Debug print
    mqtt.publish(topicInit, jsonBuffer);
}

boolean mqttConnect()
{
    DEBUG_PRINT("Connecting to ");
    DEBUG_PRINT(broker);

    // Authenticating MQTT:
    boolean status = mqtt.connect(clientMQTT, username, password);

    if (status == false)
    {
        DEBUG_PRINTLN(" fail");
        return false;
    }
    DEBUG_PRINTLN(" success");
    sendMQTT();
    return mqtt.connected();
}

// Setup function
void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        ;
    }

    DEBUG_PRINTLN("Initializing ESP32 Sensor Monitoring System...");

    esp_sleep_enable_timer_wakeup(sleep_PARAMETER);
    DEBUG_PRINTF("sleep_PARAMETER: %llu\n", sleep_PARAMETER);

    // Configure GPIOs
    pinMode(TEMP_SENSOR_POWER_PIN, OUTPUT);
    pinMode(TDS_SENSOR_POWER_PIN, OUTPUT);
    pinMode(PH_SENSOR_POWER_PIN, OUTPUT);
    pinMode(ONE_WIRE_BUS, INPUT);

    mqtt.setServer(broker, 1883); // MQTT Broker setup

    Wire.begin(); // Initialize I2C

    // Configure RTC
    URTCLIB_WIRE.begin();
    rtc.set_model(rtcModel);
    rtc.refresh();
    rtcVerify(rtc);
    // Set the RTC time, comment out once set and reload the code
    // rtc.set(0, 1, 18, 2, 1, 4, 25);

    intializeSDCard(SD_SCLK, SD_MISO, SD_MOSI, SD_CS); // Initialize SD Card

    createDir(SD, directory); // Create directory if it doesn't exist

    // Initialize sensors
    tempSensorInstance.init();
    tdsSensorInstance.init();
    pHSensorInstance.init();

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
    {
        bootCounter++; // Increment boot counter
        DEBUG_PRINT("Boot Counter: ");
        DEBUG_PRINTLN(bootCounter);
        DEBUG_PRINTLN("Waking up from deep sleep.");
        codeExecuted = true;
    }

    if (!codeExecuted)
    {
        bootCounter++; // Increment boot counter
        DEBUG_PRINTLN("Setup Complete.");
        codeExecuted = true;
    }
}

void loop()
{
    int counterNetworkRetries = 0;
    const int MAX_RETRIES = 4;
    rtc.refresh();
    DEBUG_PRINTLN("========TimeStamp Print.=======");
    String currentTime = rtcVerify(rtc);
    DEBUG_PRINTLN(currentTime);
    DEBUG_PRINTLN("===========================");

    static bool tempStarted = false;
    static bool tdsStarted = false;
    static bool pHStarted = false;

    tempStateMachine.operate();
    tdsStateMachine.operate();
    pHStateMachine.operate();

    if (!tempStarted)
    {
        DEBUG_PRINTLN("> Starting Temperature Sensor... <");
        tempStateMachine.start();
        tempStarted = true;
    }

    if (tempStateMachine.isOff() && !tdsStarted)
    {
        DEBUG_PRINTLN("> Temperature Sensor done. Starting TDS Sensor... <");
        tdsStateMachine.start();
        tdsStarted = true;
    }

    if (tempStateMachine.isOff() && tdsStateMachine.isOff() && !pHStarted)
    {
        DEBUG_PRINTLN("> TDS Sensor done. Starting pH Sensor... <");
        pHStateMachine.start();
        pHStarted = true;
    }

    if (tempStateMachine.isOff() && tdsStateMachine.isOff() && pHStateMachine.isOff())
    {
        if (!dataLogged)
        {
            DEBUG_PRINTLN("> All sensors have finished. Writing data to SD Card. <");
            DEBUG_PRINTLN("========Datalogging Print.=======");
            DEBUG_PRINTLN("========SD Card.=======");
            logDataWithTimestamp(SD, filename, rtc);
            DEBUG_PRINTLN("========SD Write.=======");
            delay(500);
            if (bootCounter % 1 == 0)
            {
                String res;
                Serial1.flush();
                delay(100);
                DEBUG_PRINTLN("========GSM Print.=======");
                // Initialize Modem
                Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
                DEBUG_PRINTLN("Wait for modem...setting GSM module baud rate to 9600");
                Serial1.begin(9600);
                delay(6000);
                modemPowerOn();

                // Unlock your SIM card with a PIN if needed
                if (SIM_PIN && modem.getSimStatus() != 3)
                {
                    modem.simUnlock(SIM_PIN);
                }

                DEBUG_PRINTLN("========SIMCOMATI======");
                modem.sendAT("+SIMCOMATI");
                modem.waitResponse(1000L, res);
                res.replace(AT_NL "OK" AT_NL, "");
                DEBUG_PRINTLN(res);
                res = "";
                DEBUG_PRINTLN("=======================");
                
                DEBUG_PRINTLN("=====Preferred mode selection=====");
                modem.sendAT("+CNMP?");
                if (modem.waitResponse(1000L, res) == 1) {
                  res.replace(AT_NL "OK" AT_NL, "");
                  DEBUG_PRINTLN(res);
                }
                res = "";
                DEBUG_PRINTLN("=======================");
              
              
                DEBUG_PRINTLN("=====Preferred selection between CAT-M and NB-IoT=====");
                modem.sendAT("+CMNB?");
                if (modem.waitResponse(1000L, res) == 1) {
                  res.replace(AT_NL "OK" AT_NL, "");
                  DEBUG_PRINTLN(res);
                }
                res = "";
                DEBUG_PRINTLN("=======================");

                DEBUG_PRINT("Waiting for network...");
                while (!modem.waitForNetwork() && counterNetworkRetries < MAX_RETRIES)
                {
                    DEBUG_PRINT(".");
                    delay(1000);
                    DEBUG_PRINTLN(" fail");
                    delay(8000);
                    return;
                }
                DEBUG_PRINTLN(" success");
                
                for (int i = 0; i <= 4; i++) {
                    uint8_t network[] = {
                        2,  /*Automatic*/
                        13, /*GSM only*/
                        38, /*LTE only*/
                        51  /*GSM and LTE only*/
                    };
                    DEBUG_PRINTF("Try %d method\n", network[i]);
                    modem.setNetworkMode(network[i]);
                    delay(3000);
                    bool isConnected = false;
                    int tryCount = 60;
                    while (tryCount--) {
                      int16_t signal =  modem.getSignalQuality();
                      DEBUG_PRINT("Signal: ");
                      DEBUG_PRINT(signal);
                      DEBUG_PRINTLN(" ");
                      DEBUG_PRINT("isNetworkConnected: ");
                      isConnected = modem.isNetworkConnected();
                      DEBUG_PRINTLN( isConnected ? "CONNECT" : "NO CONNECT");
                      if (isConnected) {
                        break;
                        }
                    }
                    if (isConnected) {
                        break;
                    }
                    delay(1000);  
                }
                if (modem.isNetworkConnected())
                {
                    DEBUG_PRINTLN("Network connected");
                }

                if (!verifyGPRS())
                {
                    delay(8000);
                    return;
                }

                String name = modem.getModemName();
                DEBUG_PRINTLN("Modem Name: " + name);

                DEBUG_PRINTLN("========MQTT Print.=======");
                while (!mqtt.connected() && counterNetworkRetries < MAX_RETRIES)
                {
                    DEBUG_PRINTLN("Attempting MQTT connection...");
                    if (mqttConnect())
                    {
                        DEBUG_PRINTLN("MQTT Connected!");
                    }
                    else
                    {
                        DEBUG_PRINTLN("MQTT Connection Failed.");
                        counterNetworkRetries++;
                    }
                }
                DEBUG_PRINTLN("===========================");

                // Disconnect network
                modemPowerOff();
                DEBUG_PRINTLN("========GSM Print Finished.=======");
            }
            dataLogged = true; // Ensure logging only happens once
            DEBUG_PRINTLN("========Datalogging Print Finished.=======");
        
        }

        // Delay for 3 seconds before sleeping (non-blocking)
        static unsigned long sleepDelayStart = 0;
        if (sleepDelayStart == 0)
        {
            sleepDelayStart = millis();
        }

        if (millis() - sleepDelayStart >= 3000)
        {
            DEBUG_PRINTLN("> Data logging completed. Entering deep sleep. <");
            DEBUG_PRINT("Cycle Time (seconds): ");
            DEBUG_PRINTLN(millis() / 1000);
            DEBUG_PRINTLN("Entering deep sleep...");
            DEBUG_PRINTF("Sleep duration: %llu µs\n", sleep_PARAMETER);
            DEBUG_FLUSH();
            esp_deep_sleep_start();
        }
    }
} 
// End of loop
