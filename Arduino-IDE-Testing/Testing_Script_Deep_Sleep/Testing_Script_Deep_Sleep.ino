#define uS_TO_S_FACTOR 1000000           // Conversion factor from microseconds to seconds
#define S_TO_MIN_FACTOR 60               // Conversion factor from seconds to minutes
#define TIME_TO_SLEEP 2                  // Duration in minutes for deep sleep between readings
RTC_DATA_ATTR int16_t bootCount = 0;
RTC_DATA_ATTR uint16_t sleepCount = 0;   

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
delay(2000);
Serial.print("Times Booted: ");
Serial.println(bootCount);
Serial.println("Looping....");
sleepCount++;
Serial.print("Times Asleep: ");
Serial.println(sleepCount);
Serial.println("Going to deep sleep now.");
Serial.flush();
esp_deep_sleep_start();
}
