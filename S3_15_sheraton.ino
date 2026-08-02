#include "arduino_secrets.h"
// 3-VALVE IRRIGATION CONTROLLER
// ESP32-S3 + DS3231 RTC + 24C32 EEPROM + DHT22 + water-level interlock
// Cloud variables must match the generated thingProperties.h supplied with this sketch.

#include "thingProperties.h"
#include <Wire.h>
#include <RTClib.h>
#include <WiFi.h>
#include "sensors.h"

#define NUM_ZONES 3
#define PUMP_PIN 47
#define BATTERY_PIN 1
#define I2C_SDA 9
#define I2C_SCL 8

const uint8_t SOLENOID_PINS[NUM_ZONES] = {11, 4, 7}; // A, B, H -- change H pin if wired differently

#define MANUAL_TIMEOUT 180000UL
#define MANUAL_RECONNECT_GRACE 5000UL
#define TIME_SYNC_INTERVAL 1000UL
#define BATTERY_READ_INTERVAL 300000UL
#define DHT_READ_INTERVAL 5000UL
#define ULTRASONIC_READ_INTERVAL 1000UL
#define SOIL_READ_INTERVAL 2000UL
#define EEPROM_WRITE_DELAY 5000UL
#define VOLTAGE_DIVIDER_RATIO 4.3f

#define RTC_ADDRESS 0x68
#define EEPROM_ADDRESS 0x57
#define EEPROM_MAGIC 0xA5C4
#define EEPROM_VERSION 1
#define EEPROM_SLOTS 4
#define EEPROM_SLOT_SZ 512

RTC_DS3231 rtc;

struct ScheduleData {
  int32_t startEpochUTC;
  uint32_t durationSec;
  int16_t repeatType;       // 0=once, 1=hourly, 2=daily, 3=weekly
  int16_t repeatInterval;
  bool enabled;
};

struct BackupBlock {
  uint16_t magic;
  uint32_t timestamp;
  uint16_t version;
  ScheduleData schedules[NUM_ZONES];
  uint16_t checksum;
};

struct CloudZone {
  int *hour, *minute, *durationMin, *repeatType, *repeatInterval;
  bool *enabled, *manualCtrl, *led;
};

BackupBlock backup;
CloudZone cz[NUM_ZONES];
bool rtcOK = false, eepromOK = false, scheduleDirty = false;
bool zoneOn[NUM_ZONES] = {}, manualShadow[NUM_ZONES] = {};
uint32_t manualStartMs[NUM_ZONES] = {};
uint32_t lastChangeMs = 0, timeBaseUTC = 0, lastCloudConnectTime = 0;
uint8_t activeSlot = 0;
bool timeBaseValid = false, cloudWasConnected = false;

inline ScheduleData &SD(uint8_t z) { return backup.schedules[z]; }

void initCloudZones() {
  CloudZone zones[NUM_ZONES] = {
    {&start_A_hour_sched, &start_A_minute_sched, &scheduler_A_durationMinutes, &scheduler_A_repeatType, &scheduler_A_repeatInterval, &scheduler_A_enabled, &solenoid_A_manual_control, &state_led_A},
    {&start_B_hour_sched, &start_B_minute_sched, &scheduler_B_durationMinutes, &scheduler_B_repeatType, &scheduler_B_repeatInterval, &scheduler_B_enabled, &solenoid_B_manual_control, &state_led_B},
    {&start_H_hour_sched, &start_H_minute_sched, &scheduler_H_durationMinutes, &scheduler_H_repeatType, &scheduler_H_repeatInterval, &scheduler_H_enabled, &solenoid_H_manual_control, &state_led_H}
  };
  memcpy(cz, zones, sizeof(zones));
}

void initHardware() {
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    pinMode(SOLENOID_PINS[z], OUTPUT);
    digitalWrite(SOLENOID_PINS[z], LOW);
  }
  pinMode(PUMP_PIN, OUTPUT); digitalWrite(PUMP_PIN, LOW);
  pinMode(BATTERY_PIN, INPUT); analogSetAttenuation(ADC_11db);
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(100000);
  rtcOK = (Wire.beginTransmission(RTC_ADDRESS), Wire.endTransmission() == 0 && rtc.begin());
  eepromOK = (Wire.beginTransmission(EEPROM_ADDRESS), Wire.endTransmission() == 0);
  initSensors();
}

void updateBattery() {
  long millivolts = 0;
  for (uint8_t i = 0; i < 10; i++) { millivolts += analogReadMilliVolts(BATTERY_PIN); delay(1); }
  battery_voltage = (millivolts / 10000.0f) * VOLTAGE_DIVIDER_RATIO;
}

uint16_t checksum(const uint8_t *data, size_t size) {
  uint16_t sum = 0; while (size--) sum += *data++; return sum;
}

bool readEEPROM(uint16_t address, uint8_t *data, uint16_t length) {
  if (!eepromOK) return false;
  Wire.beginTransmission(EEPROM_ADDRESS); Wire.write(address >> 8); Wire.write(address & 0xFF);
  if (Wire.endTransmission() != 0) return false;
  uint16_t received = 0;
  while (received < length) {
    uint8_t chunk = min((uint16_t)32, (uint16_t)(length - received));
    uint8_t count = Wire.requestFrom((uint8_t)EEPROM_ADDRESS, chunk);
    if (!count) return false;
    while (Wire.available() && received < length) data[received++] = Wire.read();
  }
  return true;
}

bool writeEEPROM(uint16_t address, const uint8_t *data, uint16_t length) {
  if (!eepromOK) return false;
  for (uint16_t offset = 0; offset < length;) {
    uint8_t chunk = min((uint16_t)16, (uint16_t)(length - offset));
    Wire.beginTransmission(EEPROM_ADDRESS); Wire.write((address + offset) >> 8); Wire.write((address + offset) & 0xFF);
    for (uint8_t i = 0; i < chunk; i++) Wire.write(data[offset + i]);
    if (Wire.endTransmission() != 0) return false;
    offset += chunk; delay(5);
  }
  return true;
}

void pushToCloud();

void initDefaults(bool publish) {
  DateTime now = rtcOK ? rtc.now() : DateTime(2026, 1, 1, 6, 0, 0);
  memset(&backup, 0, sizeof(backup));
  backup.magic = EEPROM_MAGIC; backup.version = EEPROM_VERSION; backup.timestamp = now.unixtime();
  for (uint8_t z = 0; z < NUM_ZONES; z++) SD(z) = {(int32_t)DateTime(now.year(), now.month(), now.day(), 6, 0, 0).unixtime(), 600, 2, 1, false};
  backup.checksum = checksum((uint8_t *)&backup.schedules, sizeof(backup.schedules));
  if (eepromOK) writeEEPROM(0, (uint8_t *)&backup, sizeof(backup));
  if (publish) pushToCloud();
}

uint8_t findBestSlot() {
  uint32_t newest = 0; uint8_t slot = 0; bool found = false;
  for (uint8_t s = 0; s < EEPROM_SLOTS; s++) {
    BackupBlock candidate;
    if (!readEEPROM(s * EEPROM_SLOT_SZ, (uint8_t *)&candidate, sizeof(candidate))) continue;
    if (candidate.magic != EEPROM_MAGIC || candidate.version != EEPROM_VERSION) continue;
    if (checksum((uint8_t *)&candidate.schedules, sizeof(candidate.schedules)) != candidate.checksum) continue;
    if (!found || candidate.timestamp > newest) { newest = candidate.timestamp; slot = s; found = true; }
  }
  if (!found) initDefaults(false);
  return slot;
}

void loadFromEEPROM() {
  BackupBlock candidate;
  if (eepromOK && readEEPROM(activeSlot * EEPROM_SLOT_SZ, (uint8_t *)&candidate, sizeof(candidate)) && candidate.magic == EEPROM_MAGIC && candidate.version == EEPROM_VERSION && checksum((uint8_t *)&candidate.schedules, sizeof(candidate.schedules)) == candidate.checksum) backup = candidate;
  pushToCloud();
}

void saveToEEPROM() {
  if (!scheduleDirty || millis() - lastChangeMs < EEPROM_WRITE_DELAY) return;
  backup.magic = EEPROM_MAGIC; backup.version = EEPROM_VERSION;
  backup.timestamp = rtcOK ? rtc.now().unixtime() : millis() / 1000;
  backup.checksum = checksum((uint8_t *)&backup.schedules, sizeof(backup.schedules));
  uint8_t next = (activeSlot + 1) % EEPROM_SLOTS;
  if (writeEEPROM(next * EEPROM_SLOT_SZ, (uint8_t *)&backup, sizeof(backup))) { activeSlot = next; scheduleDirty = false; }
}

uint8_t daysInMonth(int year, int month) {
  static const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) return 29;
  return days[month - 1];
}

void pushToCloud() {
  DateTime first(SD(0).startEpochUTC);
  scheduler_start_year = first.year(); scheduler_start_month = first.month();
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    DateTime start(SD(z).startEpochUTC);
    *cz[z].hour = start.hour(); *cz[z].minute = start.minute();
    *cz[z].durationMin = SD(z).durationSec / 60; *cz[z].repeatType = SD(z).repeatType;
    *cz[z].repeatInterval = SD(z).repeatInterval; *cz[z].enabled = SD(z).enabled;
  }
}

bool hasChanged(uint8_t z) {
  DateTime oldStart(SD(z).startEpochUTC);
  int day = min(oldStart.day(), daysInMonth(scheduler_start_year, scheduler_start_month));
  DateTime requested(scheduler_start_year, scheduler_start_month, day, *cz[z].hour, *cz[z].minute, 0);
  return SD(z).startEpochUTC != (int32_t)requested.unixtime() || SD(z).durationSec != (uint32_t)(*cz[z].durationMin * 60) || SD(z).repeatType != *cz[z].repeatType || SD(z).repeatInterval != *cz[z].repeatInterval || SD(z).enabled != *cz[z].enabled;
}

void onScheduleChange(uint8_t z) {
  if (*cz[z].durationMin < 0 || *cz[z].repeatInterval <= 0 || scheduler_start_month < 1 || scheduler_start_month > 12) return;
  if (!hasChanged(z)) return;
  DateTime oldStart(SD(z).startEpochUTC);
  int day = min(oldStart.day(), daysInMonth(scheduler_start_year, scheduler_start_month));
  SD(z).startEpochUTC = DateTime(scheduler_start_year, scheduler_start_month, day, *cz[z].hour, *cz[z].minute, 0).unixtime();
  SD(z).durationSec = (uint32_t)*cz[z].durationMin * 60; SD(z).repeatType = *cz[z].repeatType;
  SD(z).repeatInterval = *cz[z].repeatInterval; SD(z).enabled = *cz[z].enabled;
  scheduleDirty = true; lastChangeMs = millis();
}

void onManualChange(uint8_t z, bool value) {
  if (value) { manualShadow[z] = true; manualStartMs[z] = millis(); }
  else if (millis() - lastCloudConnectTime > MANUAL_RECONNECT_GRACE) manualShadow[z] = false;
}

bool isActive(uint8_t z) {
  ScheduleData &s = SD(z); if (!s.enabled) return false;
  uint32_t now = rtcOK ? rtc.now().unixtime() : (timeBaseValid ? timeBaseUTC + millis() / 1000 : millis() / 1000);
  if (now < (uint32_t)s.startEpochUTC) return false;
  if (s.repeatType == 0) return now < (uint32_t)s.startEpochUTC + s.durationSec;
  int64_t interval = s.repeatType == 1 ? 3600LL * s.repeatInterval : s.repeatType == 2 ? 86400LL * s.repeatInterval : s.repeatType == 3 ? 604800LL * s.repeatInterval : 0;
  return interval > 0 && ((int64_t)now - s.startEpochUTC) % interval < s.durationSec;
}

void controlZones() {
  static bool pumpOn = false; bool anyOn = false;
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    if (manualShadow[z] && millis() - manualStartMs[z] >= MANUAL_TIMEOUT) { manualShadow[z] = false; *cz[z].manualCtrl = false; }
    bool shouldOn = (isActive(z) || manualShadow[z]); // Water level sensor removed - always safe
    if (shouldOn != zoneOn[z]) { digitalWrite(SOLENOID_PINS[z], shouldOn); zoneOn[z] = shouldOn; }
    *cz[z].led = zoneOn[z]; anyOn |= zoneOn[z];
  }
  if (anyOn != pumpOn) { digitalWrite(PUMP_PIN, anyOn); pumpOn = anyOn; }
}

void updateTime() {
  uint32_t now = rtcOK ? rtc.now().unixtime() : 0;
  if (WiFi.status() == WL_CONNECTED) {
    uint32_t cloudTime = ArduinoCloud.getLocalTime();
    if (cloudTime > 1600000000UL) { now = cloudTime; timePicker = cloudTime; if (rtcOK && abs((long)(cloudTime - rtc.now().unixtime())) > 5) rtc.adjust(DateTime(cloudTime)); }
  }
  if (now) { timePicker = now; timeBaseUTC = now - millis() / 1000; timeBaseValid = true; }
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("Irrigation controller starting");
  initHardware(); initProperties(); initCloudZones();
  if (eepromOK) { activeSlot = findBestSlot(); loadFromEEPROM(); } else initDefaults(true);
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  updateBattery();
  readDHT22();
  water_level_safe = true; // Water level sensor removed - always safe
}

void loop() {
  ArduinoCloud.update();
  bool connected = ArduinoCloud.connected();
  if (!cloudWasConnected && connected) { lastCloudConnectTime = millis(); loadFromEEPROM(); }
  cloudWasConnected = connected;
  static uint32_t lastTime = 0, lastBatt = 0, lastDHT = 0, lastUltrasonic = 0, lastSoil = 0; 
  uint32_t now = millis();
  if (now - lastTime >= TIME_SYNC_INTERVAL) { updateTime(); lastTime = now; }
  if (now - lastBatt >= BATTERY_READ_INTERVAL) { updateBattery(); lastBatt = now; }
  if (now - lastUltrasonic >= ULTRASONIC_READ_INTERVAL) { readUltrasonicDistanceCm(); lastUltrasonic = now; }
  if (now - lastSoil >= SOIL_READ_INTERVAL) { readSoilMoisture(); lastSoil = now; }
  if (now - lastDHT >= DHT_READ_INTERVAL) {
    readDHT22();
    readWaterTemperature();
    readTDSSensor();
    printMonitorData();
    lastDHT = now;
  }
  controlZones(); 
  saveToEEPROM(); 
  delay(10);
}

void onSchedulerStartYearChange() { for (uint8_t z = 0; z < NUM_ZONES; z++) onScheduleChange(z); }
void onSchedulerStartMonthChange() { for (uint8_t z = 0; z < NUM_ZONES; z++) onScheduleChange(z); }
void onStartAHourSchedChange() { onScheduleChange(0); }
void onStartAMinuteSchedChange() { onScheduleChange(0); }
void onSchedulerADurationMinutesChange() { onScheduleChange(0); }
void onSchedulerARepeatIntervalChange() { onScheduleChange(0); }
void onSchedulerARepeatTypeChange() { onScheduleChange(0); }
void onSchedulerAEnabledChange() { onScheduleChange(0); }

void onStartBHourSchedChange() { onScheduleChange(1); }
void onStartBMinuteSchedChange() { onScheduleChange(1); }
void onSchedulerBDurationMinutesChange() { onScheduleChange(1); }
void onSchedulerBRepeatIntervalChange() { onScheduleChange(1); }
void onSchedulerBRepeatTypeChange() { onScheduleChange(1); }
void onSchedulerBEnabledChange() { onScheduleChange(1); }

void onStartHHourSchedChange() { onScheduleChange(2); }
void onStartHMinuteSchedChange() { onScheduleChange(2); }
void onSchedulerHDurationMinutesChange() { onScheduleChange(2); }
void onSchedulerHRepeatIntervalChange() { onScheduleChange(2); }
void onSchedulerHRepeatTypeChange() { onScheduleChange(2); }
void onSchedulerHEnabledChange() { onScheduleChange(2); }

void onSolenoidAManualControlChange() { onManualChange(0, solenoid_A_manual_control); }
void onSolenoidBManualControlChange() { onManualChange(1, solenoid_B_manual_control); }
void onSolenoidHManualControlChange() { onManualChange(2, solenoid_H_manual_control); }