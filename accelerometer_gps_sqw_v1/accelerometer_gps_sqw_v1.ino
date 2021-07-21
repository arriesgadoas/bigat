
// PROPOSED FLOWCHART: https://lucid.app/lucidchart/6b1d42b4-e0a5-4aa0-bbd3-9fb8a59df981/view?page=0_0#

#include "I2Cdev.h"
#include "MPU6050.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"
#include "Wire.h"
#include "LSM6DSLSensor.h"
#include <TinyGPS++.h>

// Pins

// All devices are powered with 3.3V
// If using MPU6050, connect AD0 to 3.3V
// In order to avoid I2C address conflict with DS3231

#define SDA 21    // Connect to accelerometer, DS3231 RTC
#define SCL 22    // Connect to accelerometer, DS3231 RTC
#define SCK 14    // For SD Card, LoRa transceiver
#define MISO 2    // For SD Card, LoRa transceiver
#define MOSI 15   // For SD Card, LoRa transceiver
#define CS 13     // For SD Card
#define RXD1 32   // For NEO-6M GPS
#define TXD1 33   // For NEO-6M GPS
#define PPSPIN 35 // For NEO-6M GPS (PPS = pulse per second)
#define SQWPIN 25 // For DS3231 RTC (SQW = square wave)

// MPU6050 interface
MPU6050 accel(MPU6050_ADDRESS_AD0_HIGH);

// LSM6DSL interface
TwoWire LSM6DSL_i2c(0);
LSM6DSLSensor LSM_accel(&LSM6DSL_i2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW);

// GPS interface
TinyGPSPlus gps;

// Device ID to identify which data came from which device
const int devID = 1;

// DS3231 registers
const int RTC_I2C_ADDRESS = 0x68;
const int RTC_CONTROL = 0x0E;
const int RTC_SQW_1HZ = 0x00;
const int RTC_SQW_1024HZ = 0x08;
const int RTC_SQW_4096HZ = 0x10;

// Structure of accelerometer data to be saved
struct dataStore {
  bool PPS;                   // For syncing. value = 1 is the point of synchronization
  unsigned long tstamp;       // There are tstamp/1024 seconds per tick of PPS
  int16_t ax;                 // x-axis of accelerometer
  int16_t ay;                 // y-axis of accelerometer
  int16_t az;                 // z-axis of accelerometer
};

// Structure of GPS data to be saved
struct gpsVariables {
  double lat;                 // Latitude
  double lng;                 // Longitude
  int32_t alt;                // Altitude in cm
  uint32_t date;              // Date of getting the data
  uint32_t time;              // Time of getting the data
};

// dataFile for temporary storage (binary)
File dataFile;

// txtFile for user readable storage
File txtFile;

// File name. Format = <devID>_DDMMYY_HHMMSSCC
// See preparing_datagathering()
String filename;

// Boolean to check if data is saved to txt
bool saved = true;

// For detection of changes in PPS
// See PPS_tick()
int old_PPS = LOW;
int new_PPS;

// For detection of changes in SQW
// See detect_SQW()
int old_SQW = HIGH;
int new_SQW;

// Initializing data structs needed
struct dataStore myData;
struct gpsVariables gpsData;

// Initialize LSM6DSL accel
void LSM6DSL_init() {
  LSM6DSL_i2c.begin(SDA, SCL, 400000);
  LSM_accel.begin();
  LSM_accel.Enable_X();
  LSM_accel.Set_X_ODR(1660.0f);
}

// Getting raw accelerometer data from LSM6DSL accel
// Divide to 2^14 to get value in g
void LSM6DSL_getacceldata(){
  int16_t rawData[3];
  LSM_accel.Get_X_AxesRaw(rawData);
  myData.ax = rawData[0];
  myData.ay = rawData[1];
  myData.az = rawData[2];
}

// Getting raw accelerometer data from MPU6050 accel
// Divide to 2^14 to get value in g
void MPU6050_getacceldata() {
  accel.getAcceleration(&myData.ax, &myData.ay, &myData.az);
}

// https://github.com/00steve00/Super-Accurate-Arduino-Clock/blob/master/GPS-RTC-Clock.ino
byte dec2bcd(byte n) {
  uint16_t a = n;
  byte b = (a*103) >> 10;
  return  n + b*6;
}

// I2C comms with DS3231
// https://github.com/00steve00/Super-Accurate-Arduino-Clock/blob/master/GPS-RTC-Clock.ino
void sendRTC(byte reg_addr, byte byte_data) {
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(reg_addr);   //set register pointer to address on DS3231
  Wire.write(byte_data);
  Wire.endTransmission();
}

// Activate SQW of DS3231
void SQW_init() {
  sendRTC(RTC_CONTROL,RTC_SQW_1HZ);
}

// Detect if SQW of DS3231 pulses or not
bool detect_SQW() {
  new_SQW = digitalRead(SQWPIN);
  bool tick = (old_SQW == HIGH && new_SQW == LOW);
  old_SQW = new_SQW;
  return tick;
}

// Debugging purposes
// To check if GPS is receiving data from satellites
// Takes around 5 mins
// Time is first synchronized, then date, 
// then # of satellites, then location
void displayInfo()
{
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }
  Serial.print("# of Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.println();
  Serial.println();
}

// Wait for valid location from GPS
void wait_for_GPS_location(){
  while (!gps.location.isValid()) {
    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read())) {
        displayInfo();
      }
    }
  }

  // 15 seconds of delay alloted for location to relax
  delay(15000);
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  gpsData.lat = gps.location.lat();
  gpsData.lng = gps.location.lng();
  gpsData.alt = gps.altitude.value();

  // Debugging purposes
  Serial.println("GPS DONE");
  Serial.println(gpsData.lat, 8);
  Serial.println(gpsData.lng, 8);
  Serial.println(gpsData.alt, 8);
  
}

// GPS PPS. Ticks every 1 second and is synchronized 
// for every GPS with typical difference of < 10ms.
bool PPS_tick() {
  new_PPS = digitalRead(PPSPIN);
  bool tick = (old_PPS == LOW && new_PPS == HIGH);
  old_PPS = new_PPS;
  return tick;
}

// Request to get accelerometer data
// Should be replaced with request via LoRa
bool requestReceived() {
  while (Serial.available() > 0) {
    return ((char)Serial.read() == 's');
  }
  return false;
}

// Preparing data gathering
void preparing_datagathering() {
  // Getting latest GPS data for time and date
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Assembling filename
  filename = "/";
  filename += String(devID);
  filename += "_";
  gpsData.date = gps.date.value();
  filename += (String)gpsData.date;
  filename += "_";
  gpsData.time = gps.time.value();

  // Three seconds of headstart because data gathering
  // starts after this preparation, which takes 3s.
  filename += (String)(gpsData.time + 300);

  // For debugging purposes
  Serial.println(filename);
  
  while (!PPS_tick()) {};
  // Synchronizing GPS PPS with RTC SQW
  sendRTC(0x00,dec2bcd(0));
  // Creating temporary file for data
  dataFile = SD.open(filename+".dat", FILE_WRITE);
  
  while (!PPS_tick()) {};
  // Making DS3231 generate 1024 Hz SQW
  sendRTC(RTC_CONTROL,RTC_SQW_1024HZ);
  myData.tstamp = 0;

  // Last tick is when the data gathering starts
  while (!PPS_tick()) {};
}

void get_data() {
  // Detecting PPS tick
  myData.PPS = PPS_tick();

  // SQW count resets when PPS ticks
  if (myData.PPS) {
    myData.tstamp = 0;
  }

  // Getting accelerometer data
  MPU6050_getacceldata();
  // LSM6DSL_getacceldata()
  dataFile.write((const uint8_t *)&myData, sizeof(myData));

  // For debugging purposes
//  if (myData.tstamp > 1020) {
//    Serial.print(myData.PPS);
//    Serial.print(",");
//    Serial.println(myData.tstamp);
//  }

  // This increases every time SQW pulses
  myData.tstamp++;
  saved = false;
}

// Request to stop gathering accel data
// Should be replaced with request via LoRa
bool stopGathering() {
  while (Serial.available() > 0) {
    return ((char)Serial.read() == 'q');
  }
  return false;
}

////////////////////


void setup() {
  // Fast I2C (400kHz)
  Wire.begin(SDA, SCL, 400000);

  // Printing stuff for debugging
  Serial.begin(230400);

  // GPS
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);

  // Initializing SD
  SPI.begin(SCK, MISO, MOSI);
  if (!SD.begin(CS)) {
    Serial.println("CARD MOUNT FAILED");
    return;
  }

  // initialize accelerometer
  Serial.println("Initializing I2C devices...");
  // LSM6DSL_init();
  accel.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

//   Initializing dataFile in write mode
//  dataFile = SD.open("/datalog.dat", FILE_WRITE);
//  if (!dataFile) {
//    Serial.println("FAILED TO OPEN FILE");
//  }

  // Preparing pins for SQW and PPS
  pinMode(SQWPIN, INPUT_PULLUP);
  pinMode(PPSPIN, INPUT);

  // Initialize DS3231 SQW
  SQW_init();

  // Wait for valid GPS location
  wait_for_GPS_location();
}

void loop() {
  // This will break when a start request is received
  while (!requestReceived()) {};

  // For debugging purposes
  Serial.println("START!!");

  // Preparation for data gathering
  preparing_datagathering();

  // This will break when a stop request is received
  while (!stopGathering()) {

    // Will only get data when SQW pulses
    // This will make data gathering restricted to 1024 sps
    if (detect_SQW()) {
      get_data();
    }
  }

  // Flushing data to temporary file
  dataFile.close();

  // For debugging purposes
  Serial.println("DONE!!");

  // If data from dataFile is not saved to txtFile
  if (!saved) {

    // Opening binary file, now in read mode
    dataFile = SD.open(filename+".dat", FILE_READ);

    // Opening txtFile in write mode
    txtFile = SD.open(filename+".txt", FILE_WRITE);

    // Debug to make sure that there are available bytes to 
    // be read and written
    Serial.println(dataFile.available());

    // Writing GPS data to txtfile
    txtFile.print(gpsData.lat, 10);
    txtFile.print(",");
    txtFile.print(gpsData.lng, 10);
    txtFile.print(",");
    txtFile.print(gpsData.alt);
    txtFile.print(",");
    txtFile.print(gpsData.date);
    txtFile.print(",");
    txtFile.print(gpsData.time);
    txtFile.print("\n");

    // To make sure that all bytes will be processed
    while (dataFile.available() > 0) {

      // Converting binary data to respective data type and variable names
      dataFile.read((uint8_t *)&myData, sizeof(myData));

      // Saving accel data as characters in txtFile
      txtFile.print(myData.PPS);
      txtFile.print(",");
      txtFile.print(myData.tstamp);
      txtFile.print(",");
      txtFile.print(myData.ax);
      txtFile.print(",");
      txtFile.print(myData.ay);
      txtFile.print(",");
      txtFile.print(myData.az);
      txtFile.print("\n");

      // Data now saved in txtfile
      saved = true;

      // Debugging to know how many bytes were left to be converted
      Serial.println(dataFile.available());
    }
  }

  // Properly closing the temporary file
  dataFile.close();
  
  // Flushing data to txtfile
  txtFile.close();
}
