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

float acidVoltage    = 1020.94;    //buffer solution 4.01 at 25C
float neutralVoltage = 784.12;     //buffer solution 7.0 at 25C
float baseVoltage    = 552.00;     //buffer solution 10.0 at 25C
float referenceTemp = 25.0; // reference temperature of 25 deg C
float measuredConductivityStandard = 667.39; // Measured conductivity standard for calibration
float measuredDeionizedWater = 5.52476;         // Measured deionized water for calibration
int iterations = 50;
float *analogBuffer;   // Dynamic array for buffer, NOTE: there is no destructor for the buffer
int analogBufferIndex; // Index for circular buffer

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
  adc_ph.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc_ph.setMeasureMode(ADS1115_CONTINUOUS); 
  adc_ph.setCompareChannels(ADS1115_COMP_0_GND);
  
  if(!adc_tds.init()){
    Serial.print("ADS1115 No 2 not connected!");
  }
  adc_tds.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc_tds.setMeasureMode(ADS1115_CONTINUOUS); 
  adc_tds.setCompareChannels(ADS1115_COMP_0_GND);

  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  sensors.begin();

  // Allocate memory for analog buffer
  analogBuffer = new float[iterations];
  for (int i = 0; i < iterations; i++)
    {
      analogBuffer[i] = 0.0;
    }
}

void loop() {
  float voltage = 0.0;
  float probeVoltage = 0.0;
  
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");
  
  
  // Serial.println("****************BEGIN PH**********************");
  // BEGIN PH
  // voltage = adc_ph.getResult_mV(); // 10kOhm resistor present
  // probeVoltage = voltage*2;  // multiply by 10kOhm factor
  // Serial.println("Voltage [mV], ADS1115 PH: ");
  // Serial.println(voltage);
  // Serial.println("Adjusted Voltage [mV], ADS1115 PH: ");
  // Serial.println(probeVoltage, 12);

  // analogBuffer[analogBufferIndex] = probeVoltage;
  // analogBufferIndex = (analogBufferIndex + 1) % iterations;
  // float sortedBuffer[iterations];
  // std::copy(analogBuffer, analogBuffer + iterations, sortedBuffer);
  // std::sort(sortedBuffer, sortedBuffer + iterations);
  // float medianValue;
  // if (iterations % 2 == 0)
  // {
  //   medianValue = (sortedBuffer[iterations / 2 - 1] + sortedBuffer[iterations / 2]) / 2.0f;
  // }
  // else
  // {
  //   medianValue = sortedBuffer[iterations / 2];
  // }
  // for (int i = 0; i < iterations; i++)
  //   {
  //     Serial.print(sortedBuffer[i]);
  //     Serial.print( " ");
  //   }
  // Serial.println("");
  // Serial.print("Median Value: ");
  // Serial.println(medianValue);

  // probeVoltage = medianValue;
  // // Build pH calibration code
  // double slopePH = ((7.0 - 4.01) / (neutralVoltage * 2 - acidVoltage * 2) + (7 - 10.0) / (neutralVoltage * 2 - baseVoltage * 2)) / 2;
  // double interceptPH = 7.0 - slopePH * (neutralVoltage * 2);
  // double compensatedSlope = slopePH * ((temperatureC + 273.15) / (25.0 + 273.15));
  // double pHValue = compensatedSlope * probeVoltage + interceptPH; //y = k*x + b
  // // double pHValue = slopePH * probeVoltage + interceptPH; //y = k*x + b
  // Serial.print("slope:");
  // Serial.print(slopePH,12);
  // Serial.print(",intercept:");
  // Serial.println(interceptPH,12);
  // Serial.print("pHValue: ");
  // Serial.println(pHValue);
	
	Serial.println("****************BEGIN TDS**********************");
  // BEGIN TDS
  // Build Tds Calibration code
  voltage = adc_tds.getResult_V(); // 10kOhm resistor present
  probeVoltage = voltage * 3.33457692; // multiply by 10kOhm factor, 1413 uS/cm factor
  Serial.print("Voltage [V], ADS1115 No 2: ");
  Serial.println(voltage, 12);
  Serial.print("Adjusted Voltage [V] using 1413 uS/cm factor, ADS1115 No 2: ");
  Serial.println(probeVoltage,12);
  
  analogBuffer[analogBufferIndex] = probeVoltage;
  analogBufferIndex = (analogBufferIndex + 1) % iterations;
  float sortedBuffer[iterations];
  std::copy(analogBuffer, analogBuffer + iterations, sortedBuffer);
  std::sort(sortedBuffer, sortedBuffer + iterations);
  float medianValue;
  if (iterations % 2 == 0)
  {
    medianValue = (sortedBuffer[iterations / 2 - 1] + sortedBuffer[iterations / 2]) / 2.0f;
  }
  else
  {
    medianValue = sortedBuffer[iterations / 2];
  }
  for (int i = 0; i < iterations; i++)
    {
      Serial.print(sortedBuffer[i]);
      Serial.print( " ");
    }
  Serial.println("");
  Serial.print("Median Value: ");
  Serial.println(medianValue, 12);
  probeVoltage = medianValue;

  float kCoefficient = 0.019;
  float rawEC = (133.42 * probeVoltage * probeVoltage * probeVoltage - 255.86 * probeVoltage * probeVoltage + 857.39 * probeVoltage);
  Serial.print("TDS before Temp Correction and fitting function: ");
  Serial.println(rawEC * 0.5);
  float tempCorrection = 1.0 + kCoefficient * (temperatureC - referenceTemp);
  float compensatedEC = rawEC / tempCorrection;
  float rawTds = compensatedEC * 0.5;
  Serial.print("TDS after Temp Correction and before fitting function: ");
  Serial.println(rawTds);
  float slopeTDS = (706.5 - 5) / (measuredConductivityStandard - measuredDeionizedWater);
  float interceptTDS = 0 - slopeTDS * measuredDeionizedWater;
  float correctedTds = slopeTDS * rawTds + interceptTDS; //y = k*x + b
  Serial.print("slope: ");
  Serial.println(slopeTDS);
  Serial.print("intercept:");
  Serial.println(interceptTDS);
  Serial.print("After Temp Correction and fitting function: correctedTds = ");
  Serial.println(correctedTds);
 
  Serial.println("****************************");  
  
  delay(100);
}