#include <WiFi.h>
#include "time.h"
#define uS_TO_S_FACTOR 1000000           // Conversion factor from microseconds to seconds
#define S_TO_MIN_FACTOR 60               // Conversion factor from seconds to minutes
#define TIME_TO_SLEEP 2                  // Duration in minutes for deep sleep between readings
RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint16_t sleepCount = 0;
int pollTime = 30000;
int startTime = 0;

const char* ssid     = "DNA-WIFI-58F4";
const char* password = "31033885";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200;
const int   daylightOffset_sec = 3600;

void setup()
{
  Serial.begin(9600); // Initialize serial communication for debugging
  // Wait for the serial port to connect
  while (!Serial) {
      ;
  }
  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * S_TO_MIN_FACTOR * uS_TO_S_FACTOR); // Define the sleep condition
  Serial.println("Setup Complete");
  bootCount++;
}

void loop()
{
// delay(2000);
// Serial.print("Times Booted: ");
// Serial.println(bootCount);
// Serial.println("Looping....");
// sleepCount++;
// Serial.print("Times Asleep: ");
// Serial.println(sleepCount);
if (millis() - startTime > pollTime)
  {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  Serial.println("");
  Serial.println("WiFi connected.");
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println(millis());
  Serial.print(millis()/1000);
  Serial.println(" seconds");
  startTime = millis();
  }
// if (millis() > upTime)
//   {
//   Serial.println("Going to deep sleep now.");
//   Serial.flush();
//   esp_deep_sleep_start();
//   }
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();
}