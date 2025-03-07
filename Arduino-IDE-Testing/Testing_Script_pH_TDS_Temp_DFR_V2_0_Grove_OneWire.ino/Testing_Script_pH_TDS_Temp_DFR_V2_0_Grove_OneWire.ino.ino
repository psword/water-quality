/***************************************************************************
* Example sketch for the ADS1115_WE library
*
* This sketch shows how to use two ADS1115 modules. In order to set the address, 
* connect the address pin to:
* 
* GND -> 0x48 (or leave unconnected)
* VCC -> 0x49
* SDA -> 0x4A
* SCL -> 0x4B
* 
* When you have understood how it works you can easily add two additional ADS1115.
* Of course there is potential to shorten the code, e.g. by setting up the ADCs 
* as array.
* 
* If you need up to eight ADS1115 modules you can use an ESP32 with its two I2C 
* interfaces:
* https://wolles-elektronikkiste.de/en/how-to-use-the-i2c-interfaces-of-the-esp32 
* 
* If you need up to 32 ADS1115 modules you can use a multiplexer like the TSCA9548A:
* https://wolles-elektronikkiste.de/en/tca9548a-i2c-multiplexer
*  
* Or you combine both and control up to 64 ADS1115 modules.
*  
* Further information about the ADS1115 can be found on:
* https://wolles-elektronikkiste.de/ads1115 (German)
* https://wolles-elektronikkiste.de/en/ads1115-a-d-converter-with-amplifier (English)
* 
***************************************************************************/

#include <OneWire.h>
#include <DallasTemperature.h>
#include<ADS1115_WE.h> 
#include<Wire.h>
#define I2C_ADDRESS_PH  0x4B
#define I2C_ADDRESS_TDS  0x4A

// GPIO where the DS18B20 is connected to
const int oneWireBus = 5;  

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

ADS1115_WE adc_ph = ADS1115_WE(I2C_ADDRESS_PH);
ADS1115_WE adc_tds = ADS1115_WE(I2C_ADDRESS_TDS);

float acidVoltage    = 1025.13;    //buffer solution 4.0 at 25C
float neutralVoltage = 755.25;     //buffer solution 7.0 at 25C
float baseVoltage    = 502.50;     //buffer solution 10.0 at 25C
float neutralCompensationVoltage = neutralVoltage*2;
float acidCompensationVoltage = acidVoltage*2;
float baseCompensationVoltage = baseVoltage*2;
float factoryNeutralVoltage = 1500; //ph 7.0
int phDistance = 3; //distance from neutral ph 7.0


void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  if(!adc_ph.init()){
    Serial.print("ADS1115 No 1 not connected!");
  }
  adc_ph.setVoltageRange_mV(ADS1115_RANGE_4096);
  adc_ph.setMeasureMode(ADS1115_CONTINUOUS); 
  adc_ph.setCompareChannels(ADS1115_COMP_0_GND);
  
  if(!adc_tds.init()){
    Serial.print("ADS1115 No 2 not connected!");
  }
  adc_tds.setVoltageRange_mV(ADS1115_RANGE_4096);
  adc_tds.setMeasureMode(ADS1115_CONTINUOUS); 
  adc_tds.setCompareChannels(ADS1115_COMP_0_GND);

  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  sensors.begin();
}

void loop() {
  float voltage = 0.0;
  
  voltage = adc_ph.getResult_mV(); // 10000 Ohm Resistor Cuts Voltage in half
  float probeVoltage = voltage*2;
  Serial.println("Voltage [mV], ADS1115 PH: ");
  Serial.println(voltage);
  Serial.println(probeVoltage);

  float slope = ((7.0-4.01)/((neutralCompensationVoltage - factoryNeutralVoltage)/phDistance - (acidCompensationVoltage - factoryNeutralVoltage)/phDistance) + (7-10.0)/((neutralCompensationVoltage - factoryNeutralVoltage)/phDistance - (baseCompensationVoltage - factoryNeutralVoltage)/phDistance))/2;  // three point: (neutralVoltage,7.0),(acidVoltage,4.0),(baseVoltage,10.0)
  float intercept =  7.0 - slope*(neutralCompensationVoltage - factoryNeutralVoltage)/3;
  Serial.print("slope:");
  Serial.print(slope);
  Serial.print(",intercept:");
  Serial.println(intercept);
  float phValue = slope*(probeVoltage - factoryNeutralVoltage)/3+intercept;  //y = k*x + b
  Serial.println(phValue);

  voltage = adc_tds.getResult_V(); // 10000 Ohm Resistor Cuts Voltage in half
  probeVoltage = voltage*2;
  Serial.println("Voltage [V], ADS1115 No 2: ");
  Serial.println(voltage);
  Serial.println(probeVoltage);
  float rawTds = (133.42 * probeVoltage * probeVoltage * probeVoltage - 255.86 * probeVoltage * probeVoltage + 857.39 * probeVoltage) * 0.5;
  float correctedTds = 1.1297 * rawTds - 9.52; // apply calibration scaler
  Serial.print("tds:");
  Serial.println(correctedTds);
 
  Serial.println("****************************");  
  
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");
  delay(1000);
}
