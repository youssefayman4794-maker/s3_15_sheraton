#pragma once

//======================================================
// Sensor Initialization
//======================================================

void initSensors();

//======================================================
// DHT22
//======================================================

void initDHT22();
void readDHT22();

//======================================================
// RCWL-1670 Ultrasonic (Existing - Keep as is)
//======================================================

void initUltrasonicSensor();
void readUltrasonicDistanceCm();

//======================================================
// VL53L1X ToF (New - Added separately)
//======================================================

void initTOFSensor();
void readTOFDistance();

//======================================================
// Soil Moisture
//======================================================

void initSoilMoistureSensor();
void readSoilMoisture();

//======================================================
// DS18B20 Water Temperature
//======================================================

void initWaterTemperatureSensor();
void readWaterTemperature();

//======================================================
// TDS Sensor
//======================================================

void initTDSSensor();
void readTDSSensor();

//======================================================
// Ultrasonic Distance Getters (NEW)
//======================================================

float getUltrasonicDistance();
bool ultrasonicWarning();
bool ultrasonicStopPump();

//======================================================
// Serial Monitor
//======================================================

void printMonitorData();
