#include <Arduino.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "sensors.h"

// These variables are defined by Arduino Cloud in thingProperties.h
extern float battery_voltage;
extern float humidity;
extern float Temp;
extern float water_temperature;
extern float tds_ppm;
extern float UltrasonicSensor;
extern float tof_distance_cm;
extern bool soilMoisture;
extern bool state_led_A;
extern bool state_led_B;
extern bool state_led_H;
extern bool tankWarning;
extern bool pumpSafetyStop;

// Sensor Pin Definitions
#define DHT_PIN 37
#define DHT_TYPE DHT22

// RCWL-1670 Ultrasonic
#define ULTRASONIC_TRIG_PIN 12
#define ULTRASONIC_ECHO_PIN 14
#define ULTRASONIC_TIMEOUT_US 30000UL

// EMA Filter for Ultrasonic
#define ULTRASONIC_ALPHA 0.25f  // Smoothing factor (0.1=very smooth, 0.5=fast response)

// Tank level thresholds (sensor at TOP pointing DOWN)
// When tank is FULL:  distance is SMALL (water close to sensor)
// When tank is EMPTY: distance is LARGE (water far from sensor)
#define WARNING_START_CM     15.0f   // Start warning when distance > 15cm
#define PUMP_STOP_CM         25.0f   // Stop pump when distance > 25cm (tank empty)
#define PUMP_RESTART_CM      22.0f   // Restart pump when distance < 22cm (hysteresis)

// Soil moisture
#define SOIL_MOISTURE_PIN 10
#define SOIL_MOISTURE_THRESHOLD 2000
#define SOIL_IS_WET_WHEN_BELOW_THRESHOLD true

// Water temperature and TDS
#define DS18B20_PIN 44
#define TDS_PIN 43

// Sensor Objects
static DHT dht(DHT_PIN, DHT_TYPE);
OneWire oneWire(DS18B20_PIN);
DallasTemperature waterSensor(&oneWire);
static VL53L1X tofSensor;

// Sensor Data Storage
static float ultrasonicDistanceCm = NAN;
static float ultrasonicFilteredCm = NAN;  // EMA filtered value
static float tofDistanceCm = NAN;
static uint16_t soilMoistureRaw = 0;
static uint8_t soilMoistureState = 0;
static float waterTemperature = NAN;
static float tdsValue = NAN;
static bool tofOK = false;

// Ultrasonic Alarm Variables with Hysteresis
static bool lowWater = false;        // Current state (true = low water / tank empty)
static bool ultrasonicAlarm = false;
static bool ultrasonicPumpStop = false;

//======================================================
// Sensor Initialization
//======================================================

void initSensors() {
  Serial.println("Initializing sensors...");
  initDHT22();
  initUltrasonicSensor();
  initTOFSensor();
  initSoilMoistureSensor();
  initWaterTemperatureSensor();
  initTDSSensor();
  Serial.println("Sensor initialization complete");
}

//======================================================
// DHT22
//======================================================

void initDHT22() {
  dht.begin();
  Serial.println("DHT22 initialized");
}

void readDHT22() {
  float newHumidity = dht.readHumidity();
  float newTemperature = dht.readTemperature();

  if (!isnan(newHumidity) && !isnan(newTemperature)) {
    humidity = newHumidity;
    Temp = newTemperature;
  }
}

//======================================================
// RCWL-1670 Ultrasonic with EMA Filtering + Three-State Logic
//======================================================

void initUltrasonicSensor() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  Serial.println("Ultrasonic sensor initialized");
}

void readUltrasonicDistanceCm() {
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    unsigned long echoUs =
        pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);

    if (echoUs == 0) {
        ultrasonicDistanceCm = NAN;
        UltrasonicSensor = NAN;

        // If no echo, assume tank has water (safe default)
        lowWater = false;
        ultrasonicAlarm = false;
        ultrasonicPumpStop = false;

        tankWarning = ultrasonicAlarm;
        pumpSafetyStop = ultrasonicPumpStop;

        return;
    }

    //------------------------------------------------------
    // Raw Measurement
    //------------------------------------------------------

    float measured = (echoUs * 0.0343f) / 2.0f;

    //------------------------------------------------------
    // Calibration
    //------------------------------------------------------

    float calibrated =
        -0.00379202f * measured * measured
        +1.256231f * measured
        -3.132942f;

    //------------------------------------------------------
    // EMA FILTER: Smooth out noise
    //------------------------------------------------------
    
    // First measurement - initialize filter
    if (isnan(ultrasonicFilteredCm)) {
        ultrasonicFilteredCm = calibrated;
    } else {
        // EMA: Filtered = Alpha * New + (1 - Alpha) * Previous Filtered
        ultrasonicFilteredCm =
            ULTRASONIC_ALPHA * calibrated +
            (1.0f - ULTRASONIC_ALPHA) * ultrasonicFilteredCm;
    }

    // Store filtered value
    ultrasonicDistanceCm = ultrasonicFilteredCm;
    UltrasonicSensor = ultrasonicFilteredCm;

    //------------------------------------------------------
    // THREE-STATE LOGIC: Alarm + Pump Stop with Hysteresis
    //------------------------------------------------------
    
    // Sensor at TOP of tank pointing DOWN:
    // - Tank FULL  = small distance (water close)
    // - Tank EMPTY = large distance (water far)
    
    // ---------- PUMP CONTROL (with hysteresis) ----------
    // Stop pump when tank is EMPTY (distance > 25cm)
    // Only restart when tank is no longer empty (distance < 22cm)
    if (!lowWater && ultrasonicFilteredCm > PUMP_STOP_CM) {
        lowWater = true;   // Tank is empty - STOP PUMP
        Serial.println("*** TANK EMPTY - PUMP STOPPED ***");
    }
    else if (lowWater && ultrasonicFilteredCm < PUMP_RESTART_CM) {
        lowWater = false;  // Tank has water again - RESTART PUMP
        Serial.println("*** TANK HAS WATER - PUMP ENABLED ***");
    }

    // ---------- ALARM (independent of pump state) ----------
    // Alarm only in the 15-25cm range (getting low but not empty yet)
    ultrasonicAlarm = (ultrasonicFilteredCm >= WARNING_START_CM && 
                       ultrasonicFilteredCm <= PUMP_STOP_CM);

    // Pump stop is the lowWater state (above 25cm with hysteresis)
    ultrasonicPumpStop = lowWater;

    // Update Cloud variables
    tankWarning = ultrasonicAlarm;
    pumpSafetyStop = ultrasonicPumpStop;

    //------------------------------------------------------
    // DEBUG: Print all states
    //------------------------------------------------------
    Serial.print("Raw=");
    Serial.print(calibrated, 2);
    Serial.print(" cm  Filtered=");
    Serial.print(ultrasonicFilteredCm, 2);
    Serial.print(" cm");
    Serial.print(" | lowWater=");
    Serial.print(lowWater);
    Serial.print(" | Alarm=");
    Serial.print(tankWarning);
    Serial.print(" | PumpStop=");
    Serial.println(pumpSafetyStop);
}

//======================================================
// VL53L1X ToF
//======================================================

void initTOFSensor() {
    Serial.println("Initializing VL53L1X ToF sensor...");
    
    tofSensor.setBus(&Wire);
    tofSensor.setTimeout(500);

    tofOK = tofSensor.init();
    
    if (!tofOK) {
        Serial.println("VL53L1X not found - ToF sensor disabled");
        Serial.println("Check wiring: VCC, GND, SDA(9), SCL(8)");
        return;
    }

    tofSensor.setDistanceMode(VL53L1X::Long);
    tofSensor.setMeasurementTimingBudget(50000);
    tofSensor.startContinuous(50);

    Serial.println("VL53L1X Ready - ToF sensor working");
}

void readTOFDistance() {
    if (!tofOK) {
        tofDistanceCm = NAN;
        tof_distance_cm = tofDistanceCm;
        return;
    }
    
    uint16_t distance = tofSensor.read();

    if (tofSensor.timeoutOccurred()) {
        tofDistanceCm = NAN;
    } else {
        tofDistanceCm = distance / 10.0f;
    }

    tof_distance_cm = tofDistanceCm;
}

//======================================================
// Ultrasonic Getters
//======================================================

float getUltrasonicDistance() {
    return ultrasonicDistanceCm;
}

bool ultrasonicWarning() {
    return ultrasonicAlarm;
}

bool ultrasonicStopPump() {
    return ultrasonicPumpStop;
}

//======================================================
// Soil Moisture
//======================================================

void initSoilMoistureSensor() {
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_MOISTURE_PIN, ADC_11db);
  Serial.println("Soil moisture sensor initialized");
}

void readSoilMoisture() {
  soilMoistureRaw = analogRead(SOIL_MOISTURE_PIN);
  bool isBelowThreshold = soilMoistureRaw < SOIL_MOISTURE_THRESHOLD;
  bool isWet = SOIL_IS_WET_WHEN_BELOW_THRESHOLD ? isBelowThreshold : !isBelowThreshold;
  soilMoistureState = isWet ? 1 : 0;
  soilMoisture = isWet;
}

//======================================================
// DS18B20 Water Temperature
//======================================================

void initWaterTemperatureSensor() {
  waterSensor.begin();
  Serial.print("DS18B20 devices found: ");
  Serial.println(waterSensor.getDeviceCount());
}

void readWaterTemperature() {
  waterSensor.requestTemperatures();
  float temp = waterSensor.getTempCByIndex(0);
  if (temp != DEVICE_DISCONNECTED_C) {
    waterTemperature = temp;
    water_temperature = temp;
  }
}

//======================================================
// TDS Sensor
//======================================================

void initTDSSensor() {
  pinMode(TDS_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(TDS_PIN, ADC_11db);
  Serial.println("TDS sensor initialized");
}

void readTDSSensor() {
  long adcSum = 0;
  for (int i = 0; i < 30; i++) {
    adcSum += analogRead(TDS_PIN);
    delay(2);
  }
  int adc = adcSum / 30;
  
  float voltage = adc * 3.3 / 4095.0;
  
  float coefficient = 1.0 + 0.02 * (waterTemperature - 25.0);
  float compensatedVoltage = voltage / coefficient;
  
  float tds = (133.42 * compensatedVoltage * compensatedVoltage * compensatedVoltage
               - 255.86 * compensatedVoltage * compensatedVoltage
               + 857.39 * compensatedVoltage) * 0.5;
  
  tdsValue = tds;
  tds_ppm = tds;
}

//======================================================
// Serial Monitor
//======================================================

void printMonitorData() {
  Serial.print("Temp: "); Serial.print(Temp, 1); Serial.print(" C | Humidity: ");
  Serial.print(humidity, 1); Serial.print(" % | Battery: ");
  Serial.print(battery_voltage, 2); Serial.print(" V");
  Serial.print(" | A: "); Serial.print(state_led_A ? "ON" : "OFF");
  Serial.print(" B: "); Serial.print(state_led_B ? "ON" : "OFF");
  Serial.print(" H: "); Serial.print(state_led_H ? "ON" : "OFF");
  
  Serial.print(" | Ultrasonic: ");
  if (isnan(ultrasonicDistanceCm)) {
    Serial.print("No Echo");
  } else {
    Serial.print(ultrasonicDistanceCm, 2);
    Serial.print(" cm");
    
    if (ultrasonicAlarm)
      Serial.print("  LOW WATER");
    
    if (ultrasonicPumpStop)
      Serial.print("  PUMP STOP");
  }
  
  Serial.print(" | ToF: ");
  if (!tofOK) {
    Serial.print("Disabled");
  } else if (isnan(tofDistanceCm)) {
    Serial.print("No Reading");
  } else {
    Serial.print(tofDistanceCm);
    Serial.print(" cm");
  }
  
  Serial.print(" | Soil: "); Serial.print(soilMoistureRaw);
  Serial.print(" ("); Serial.print(soilMoistureState ? "WET" : "DRY");
  Serial.print(") | Water Temp: "); Serial.print(waterTemperature);
  Serial.print(" C | TDS: "); Serial.print(tdsValue);
  Serial.println(" ppm");
}
