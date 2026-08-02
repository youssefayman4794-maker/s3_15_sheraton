#include <Arduino.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "sensors.h"

// These variables are defined by Arduino Cloud in thingProperties.h, which is
// included by the main .ino file.  Do not include thingProperties.h here:
// Arduino Cloud injects the secret definitions only for the main sketch.
extern float battery_voltage;
extern float humidity;
extern float Temp;
extern float water_temperature;
extern float tds_ppm;
extern float UltrasonicSensor;
extern bool soilMoisture;
extern bool state_led_A;
extern bool state_led_B;
extern bool state_led_H;

#define DHT_PIN 37
#define DHT_TYPE DHT22

// RCWL-1670 default GPIO mode (not UART mode):
// sensor RX is the trigger INPUT; sensor TX is the echo OUTPUT.
#define ULTRASONIC_TRIG_PIN 44  // ESP32 output -> RCWL-1670 RX
#define ULTRASONIC_ECHO_PIN 43  // ESP32 input  <- RCWL-1670 TX
#define ULTRASONIC_TIMEOUT_US 30000UL

// Soil-moisture module: use its AO pin only.
#define SOIL_MOISTURE_PIN 10
// ADC range is 0-4095. Change this after measuring your dry and wet readings.
#define SOIL_MOISTURE_THRESHOLD 2000
// 1 = wet and 0 = dry when raw value is below the threshold.
#define SOIL_IS_WET_WHEN_BELOW_THRESHOLD true

#define DS18B20_PIN 14
#define TDS_PIN 12

static DHT dht(DHT_PIN, DHT_TYPE);
OneWire oneWire(DS18B20_PIN);
DallasTemperature waterSensor(&oneWire);
static float ultrasonicDistanceCm = NAN;
static uint16_t soilMoistureRaw = 0;
static uint8_t soilMoistureState = 0;
static float waterTemperature = NAN;
static float tdsValue = NAN;

void initSensors() {
  initDHT22();
  initUltrasonicSensor();
  initSoilMoistureSensor();
  initWaterTemperatureSensor();
  initTDSSensor();
}

void initDHT22() {
  dht.begin();
}

void readDHT22() {
  float newHumidity = dht.readHumidity();
  float newTemperature = dht.readTemperature();

  // Preserve the previous Cloud values if the DHT22 read fails.
  if (!isnan(newHumidity) && !isnan(newTemperature)) {
    humidity = newHumidity;
    Temp = newTemperature;
  }
}

void initUltrasonicSensor() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
}

void readUltrasonicDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long echoUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  ultrasonicDistanceCm = echoUs == 0 ? NAN : (echoUs * 0.0343f) / 2.0f;
  UltrasonicSensor = ultrasonicDistanceCm;  // Single assignment only
}

void initSoilMoistureSensor() {
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_MOISTURE_PIN, ADC_11db);
}

void readSoilMoisture() {
  soilMoistureRaw = analogRead(SOIL_MOISTURE_PIN);
  bool isBelowThreshold = soilMoistureRaw < SOIL_MOISTURE_THRESHOLD;
  bool isWet = SOIL_IS_WET_WHEN_BELOW_THRESHOLD ? isBelowThreshold : !isBelowThreshold;
  soilMoistureState = isWet ? 1 : 0;
  soilMoisture = isWet;
}

void initWaterTemperatureSensor() {
  waterSensor.begin();
  Serial.print("DS18B20 devices = ");
  Serial.println(waterSensor.getDeviceCount());
}

void readWaterTemperature() {
  waterSensor.requestTemperatures();
  float temp = waterSensor.getTempCByIndex(0);
  Serial.print("Raw DS18B20 = ");
  Serial.println(temp);
  if (temp != DEVICE_DISCONNECTED_C) {
    waterTemperature = temp;
    water_temperature = temp;
  }
}

void initTDSSensor() {
  pinMode(TDS_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(TDS_PIN, ADC_11db);
}

void readTDSSensor() {
  long adcSum = 0;
  for (int i = 0; i < 30; i++) {
    adcSum += analogRead(TDS_PIN);
    delay(2);
  }
  int adc = adcSum / 30;
  Serial.print("ADC = ");
  Serial.println(adc);
  
  float voltage = adc * 3.3 / 4095.0;
  Serial.print("Voltage = ");
  Serial.println(voltage, 3);
  
  float coefficient = 1.0 + 0.02 * (waterTemperature - 25.0);
  float compensatedVoltage = voltage / coefficient;
  
  // Calculate TDS for all voltage values (removed the >0.1V check)
  float tds = (133.42 * compensatedVoltage * compensatedVoltage * compensatedVoltage
               - 255.86 * compensatedVoltage * compensatedVoltage
               + 857.39 * compensatedVoltage) * 0.5;
  
  tdsValue = tds;
  tds_ppm = tds;
}

void printMonitorData() {
  Serial.print("Temperature: "); Serial.print(Temp, 1); Serial.print(" C | Humidity: ");
  Serial.print(humidity, 1); Serial.print(" % | Battery: ");
  Serial.print(battery_voltage, 2); Serial.print(" V");
  Serial.print(" | A: "); Serial.print(state_led_A ? "ON" : "OFF");
  Serial.print(" | B: "); Serial.print(state_led_B ? "ON" : "OFF");
  Serial.print(" | H: "); Serial.print(state_led_H ? "ON" : "OFF");
  Serial.print(" | Distance: ");
  if (isnan(ultrasonicDistanceCm)) Serial.print("no echo");
  else { Serial.print(ultrasonicDistanceCm, 1); Serial.print(" cm"); }
  Serial.print(" | Soil raw: "); Serial.print(soilMoistureRaw);
  Serial.print(" | Soil wet: "); Serial.print(soilMoistureState);
  Serial.print(" | Water Temp: ");
  Serial.print(waterTemperature);
  Serial.print(" C");
  Serial.print(" | TDS: ");
  Serial.print(tdsValue);
  Serial.println(" ppm");
}