#include <Arduino.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <ADS1115_WE.h>
#include <Wire.h>
#define I2C_ADDRESS_PH 0x4B
#define I2C_ADDRESS_TDS 0x4A

// GPIO where the DS18B20 is connected to
const int oneWireBus = 5;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

ADS1115_WE adc_ph = ADS1115_WE(I2C_ADDRESS_PH);
ADS1115_WE adc_tds = ADS1115_WE(I2C_ADDRESS_TDS);

float acidVoltage = 2063.5501807228916;                                  // buffer solution 4.01 at 25C
float neutralVoltage = 1537.376356427379;                               // buffer solution 7.0 at 25C
float baseVoltage = 1078.2311577652847;                                  // buffer solution 10.0 at 25C
float referenceTemp = 25.0;                             // reference temperature of 25 deg C
float measuredConductivityStandard = 725.9212190526141; // Measured conductivity standard for calibration
float measuredDeionizedWater = 16.42684950747122;      // Measured deionized water for calibration
int iterations = 50;
float *analogBufferPH;
float *analogBufferTDS;
int analogBufferIndexPH = 0;
int analogBufferIndexTDS = 0;
int analogBufferCountPH = 0;
int analogBufferCountTDS = 0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  if (!adc_ph.init())
  {
    Serial.print("ADS1115 No 1 not connected!");
  }
  adc_ph.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc_ph.setMeasureMode(ADS1115_CONTINUOUS);
  adc_ph.setCompareChannels(ADS1115_COMP_0_GND);

  if (!adc_tds.init())
  {
    Serial.print("ADS1115 No 2 not connected!");
  }
  adc_tds.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc_tds.setMeasureMode(ADS1115_CONTINUOUS);
  adc_tds.setCompareChannels(ADS1115_COMP_0_GND);

  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  sensors.begin();

  // Allocate memory for analog buffer
  analogBufferPH = new float[iterations];
  analogBufferTDS = new float[iterations];
  for (int i = 0; i < iterations; i++)
  {
    analogBufferPH[i] = 0.0;
    analogBufferTDS[i] = 0.0;
  }
}

void loop()
{
  sensors.requestTemperatures(); // Send the command to get temperatures

  // Read temperature in Celsius

  float temperatureC = sensors.getTempCByIndex(0);

  // Read PH
  float voltagePH = adc_ph.getResult_mV();
  float probeVoltagePH = voltagePH * 2;
  analogBufferPH[analogBufferIndexPH] = probeVoltagePH;
  analogBufferIndexPH = (analogBufferIndexPH + 1) % iterations;
  if (analogBufferCountPH < iterations)
    analogBufferCountPH++;

  // Read TDS
  float voltageTDS = adc_tds.getResult_V();
  float probeVoltageTDS = voltageTDS * 4.46112;
  analogBufferTDS[analogBufferIndexTDS] = probeVoltageTDS;
  analogBufferIndexTDS = (analogBufferIndexTDS + 1) % iterations;
  if (analogBufferCountTDS < iterations)
    analogBufferCountTDS++;

  if (analogBufferCountPH >= iterations && analogBufferCountTDS >= iterations)
  {
    // Median filtering
    float sortedPH[iterations], sortedTDS[iterations];
    memcpy(sortedPH, analogBufferPH, sizeof(float) * iterations);
    memcpy(sortedTDS, analogBufferTDS, sizeof(float) * iterations);
    std::sort(sortedPH, sortedPH + iterations);
    std::sort(sortedTDS, sortedTDS + iterations);
    float medianPH = sortedPH[iterations / 2];
    float medianTDS = sortedTDS[iterations / 2];

    // pH calculation
    double slopePH = ((7.0 - 4.0) / (neutralVoltage - acidVoltage) +
                      (7 - 10.0) / (neutralVoltage - baseVoltage)) /
                     2;
    double interceptPH = 7.0 - slopePH * (neutralVoltage);
    double compensatedSlopePH = slopePH * ((temperatureC + 273.15) / (referenceTemp + 273.15));
    double pHValue = compensatedSlopePH * medianPH + interceptPH;

    // TDS calculation
    float kCoefficient = 0.019;
    double rawEC = (133.42 * medianTDS * medianTDS * medianTDS - 255.86 * medianTDS * medianTDS + 857.39 * medianTDS);
    double tempCorrection = 1.0 + kCoefficient * (temperatureC - referenceTemp);
    double compensatedEC = rawEC / tempCorrection;
    double tdsValue = compensatedEC * 0.5;
    // double slopeTDS = (706.5 - 2.5) / (measuredConductivityStandard - measuredDeionizedWater);
    // double interceptTDS = 0 - slopeTDS * measuredDeionizedWater;
    // double tdsValue = slopeTDS * rawTds + interceptTDS; //y = k*x + b

    // Output as CSV-style line
    Serial.print(medianPH, 4);
    Serial.print(";");
    Serial.print(medianPH / 2, 4);
    Serial.print(";");
    Serial.print(pHValue, 4);
    Serial.print(";");
    Serial.print(medianTDS, 4);
    Serial.print(";");
    Serial.print(medianTDS / 4.46112, 4);
    Serial.print(";");
    Serial.print(tdsValue, 4);
    Serial.print(";");
    Serial.print(temperatureC, 2);
    Serial.print(";");

    if (0.001 < medianTDS && medianTDS < 0.09)
      Serial.print("Water:Deionized;");
    else if (1.5 < medianTDS && medianTDS < 1.9)
      Serial.print("Water:Standard;");
    else
      Serial.print("Water:Unknown;");

    if (1450 < medianPH && medianPH < 1600)
      Serial.print("PH:Neutral;");
    else if (900 < medianPH && medianPH < 1050)
      Serial.print("PH:Basic;");
    else if (1950 < medianPH && medianPH < 2100)
      Serial.print("PH:Acidic;");
    else
      Serial.print("PH:Unknown;");

    Serial.println();
  }

  delay(100);
}
