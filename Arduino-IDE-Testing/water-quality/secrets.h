#ifndef SECRETS
#define SECRETS

// Define how we want to connect to the internet
const char apn[] = "internet"; // APN (Access Point Name)
const char gprsUser[] = "";    // GPRS User
const char gprsPass[] = "";    // GPRS Password

// MQTT Configuration
const char *broker = "backend.thinger.io";  // MQTT broker address
const char *topicInit = "esp32/water1";     // MQTT topic for initialization
const char *clientMQTT = "esp32-monitor-1"; // MQTT client name
const char *username = "Betaman6";          // MQTT username
const char *password = "lZColKNAmsZO4WLP";  // MQTT password

#endif // SECRETS