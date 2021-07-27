//libraries
#include <LoRa.h>
#include <SPI.h>
#include "SD.h"
#include "FS.h"
#include "Wire.h"
#include "LSM6DSL.h"
#include <TinyGPS++.h>  //https://github.com/mikalhart/TinyGPSPlus


//constants
#define LoRa_SCK 5
#define LoRa_MISO 19
#define LoRa_MOSI 27
#define LoRa_CS 18
#define LoRa_RST 23                // changed from 14
#define LoRa_IRQ 26
// SPIClasMs spiLORA(VSPI);

#define SD_SCK 14
#define SD_MISO 2
#define SD_MOSI 15
#define SD_CS 13
#define SD_speed  27000000
SPIClass spiSD(HSPI);

// For I2C devices (accelerometer, RTC)
#define SDA 21
#define SCL 22

#define RXD1 36   // For NEO-6M GPS
#define TXD1 39   // For NEO-6M GPS

#define PPSPIN 35 // For NEO-6M GPS (PPS = pulse per second)
#define SQWPIN 25 // For DS3231 RTC (SQW = square wave)

// DS3231 registers
const int RTC_I2C_ADDRESS = 0x68;
const int RTC_CONTROL = 0x0E;
const int RTC_SQW_1HZ = 0x00;
const int RTC_SQW_1024HZ = 0x08;
const int RTC_SQW_4096HZ = 0x10;

// LSM6DSL interface
LSM6DSL imu(LSM6DSL_MODE_I2C, 0x6B);

// GPS interface
TinyGPSPlus gps;

//functions
void rTask(void *param);          //recieve packet parallel task in relay mode
void sTask(void *param);          //send packet parallel task in relay mode
void setupLoRa();                 //setup Lora module's freq, Tx power, SF, etc.
void receivePacket();             //check if valid packet is received
void sendPacket();                //send latest packet_t
void printPacket();               //for debugging purposes
void myPacket();                  //updates packet_t to contain id and level of this node


//packet structure definition
typedef struct packet {
  byte key;                       //passkey
  byte command;                   //0,1,2, or 3
  byte level;                     //level assigend to a node
  byte id;                        //node id must be unique
  byte path[10];                  //track packet path[sorce_node id, 2nd_hop_node id, 3rd_hop_node id,...]
};

// Structure of GPS data to be saved
struct gpsVariables {
  double lat;                 // Latitude
  double lng;                 // Longitude
  int32_t alt;                // Altitude in cm
  uint32_t date;              // Date of getting the data
  uint32_t time;              // Time of getting the data
};

// Structure of accelerometer data to be saved
struct dataStore {
  bool PPS;                   // For syncing. value = 1 is the point of synchronization
  unsigned long tstamp;       // There are tstamp/1024 seconds per tick of PPS
  int16_t ax;                 // x-axis of accelerometer
  int16_t ay;                 // y-axis of accelerometer
  int16_t az;                 // z-axis of accelerometer
};

struct background {
  int32_t count;
  int32_t ax;
  int32_t ay;
  int32_t az;
};

struct peakAccel {
  double lat;                 // Latitude
  double lng;                 // Longitude
  int32_t alt;                // Altitude in cm
  uint32_t date;              // Date of getting the data
  uint32_t time;              // Time of getting the data
  int16_t ax;
  int16_t ay;
  int16_t az;
  int32_t mag;
};

int32_t current_mag;

//variable declarations
static TaskHandle_t rT = NULL;
static TaskHandle_t sT = NULL;
static const BaseType_t core = 1;
boolean standby = true;
byte level = 100;
unsigned long relayModeTime = 180000;
const byte id = 2;
struct packet packet_t;

unsigned long tsave;
File dataFile;        // dataFile for temporary storage (binary)
File txtFile;         // txtFile for user readable storage
bool saved = true;    // boolean to check if data is saved to text

// File name. Format = <devID>_DDMMYY_HHMMSSCC
// See preparing_datagathering()
String filename;

// For detection of changes in PPS
// See PPS_tick()
int old_PPS = LOW;
int new_PPS;

// For detection of changes in SQW
// See detect_SQW()
int old_SQW = HIGH;
int new_SQW;

// Initializing data structs needed
struct dataStore accelData;
struct gpsVariables gpsData;
struct background backgroundData;
struct peakAccel pgaData;



void rTask(void *param) {
  unsigned long start = millis();
  do{
    //initialize standby to TRUE to enter relay mode
    standby = true;
    if(standby == true){
      Serial.println("waiting for packets...");
      while(standby == true && millis() - start < relayModeTime){
          receivePacket();
        }
      }

     if (packet_t.level > level) {
        for (int i = 0; i < 10; i++) {
          if (packet_t.path[i] == 0) {
            packet_t.path[i] = id;
            break;
          }
          if (i == 9) {
            break;
          }
        }
        printPacket();
        sendPacket();
      }

      else {
        Serial.println("ignore low level packet");
      }
    }while(millis() - start < relayModeTime);

    vTaskDelete(NULL);
}


void sTask(void *param) {
  int rD;
  rD = random(relayModeTime);
  vTaskDelay(rD / portTICK_PERIOD_MS);
  myPacket();
  sendPacket();
  vTaskDelete(NULL);
}


void setupLoRa() {
  Serial.println("setting up Lora...");
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_CS);
//  spiLORA.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_CS);

  LoRa.setPins(LoRa_CS, LoRa_RST, LoRa_IRQ);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(12);
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  Serial.println("Lora setup done.");
}


void receivePacket() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    LoRa.readBytes((uint8_t*)&packet_t, packetSize);
    if (packet_t.key == 83) {
      Serial.println("received command packet");
      standby = false;
    }

    else {
      Serial.println("invalid command packet");
      standby = true;
    }
  }
}


void sendPacket() {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&packet_t, sizeof(packet_t));
  LoRa.endPacket();
  Serial.println("packet forwarded");
}


void printPacket() {
  Serial.print("id: ");
  Serial.print(packet_t.id);
  Serial.print("\t");
  Serial.print("level: ");
  Serial.print(packet_t.level);
  Serial.print("\t");
  Serial.print("path: ");
  for (int i = 0; i < 10; i++ ) {
    Serial.print(packet_t.path[i]);
    Serial.print(" ");
    if (i == 9) {
      Serial.print("\n");
      break;
    }
  }
}


void myPacket(){
  packet_t.key = 83;
  packet_t.command = 1;
  packet_t.level = level;
  packet_t.id = id;
  packet_t.path[0] = id;
  }

// Initialize LSM6DSL accel
void LSM6DSL_init() {
  imu.begin();
}

// Getting raw accelerometer data from LSM6DSL accel
// Divide to 2^14 to get value in g
void LSM6DSL_getacceldata(){
  accelData.ax = imu.readRawAccelX();
  accelData.ay = imu.readRawAccelY();
  accelData.az = imu.readRawAccelZ();
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

  // 10 seconds of delay alloted for location to relax
  delay(10000);
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

int32_t calculate_mag(int32_t ax, int32_t ay, int32_t az) {
  return sqrt(sq(ax) + sq(ay) + sq(az));
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
void preparing_datagathering(int filetype) {
  // Getting latest GPS data for time and date
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Assembling filename
  filename = "/";
  filename += String(packet_t.id);
  filename += "_";
  gpsData.date = gps.date.value();
  filename += (String)gpsData.date;
  filename += "_";
  gpsData.time = gps.time.value();

  // Three seconds of headstart because data gathering
  // starts after this preparation, which takes 3s.
  filename += (String)(gpsData.time + 300);

  if (filetype == 0) {
    filename += "_background";
    backgroundData.count = 0;
    backgroundData.ax = 0;
    backgroundData.ay = 0;
    backgroundData.az = 0;
  }

  else if (filetype == 1) {
    filename += "_data";
    pgaData.ax = 0;
    pgaData.ay = 0;
    pgaData.az = 0;
    pgaData.mag = 0;
  }

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
  accelData.tstamp = 0;

  // Last tick is when the data gathering starts
  while (!PPS_tick()) {};
}

int get_data() {
  // Detecting PPS tick
  accelData.PPS = PPS_tick();

  // SQW count resets when PPS ticks
  if (accelData.PPS) { accelData.tstamp = 0; }

  // Getting accelerometer data
  // MPU6050_getacceldata();
  LSM6DSL_getacceldata();
  dataFile.write((const uint8_t *)&accelData, sizeof(accelData));

  // For debugging purposes
//  if (accelData.tstamp > 1020) {
//    Serial.print(accelData.PPS);
//    Serial.print(",");
//    Serial.println(accelData.tstamp);
//  }

  // This increases every time SQW pulses
  accelData.tstamp++;
  saved = false;

  return accelData.PPS;
}

// Request to stop gathering accel data
// Should be replaced with request via LoRa
bool stopGathering() {
  while (Serial.available() > 0) {
    return ((char)Serial.read() == 'q');
  }
  return false;
}

bool saving_data(int filetype) {
  if (!saved) {

    // Opening binary file, now in read mode
    dataFile = SD.open(filename+".dat", FILE_READ);

    // Opening txtFile in write mode
    txtFile = SD.open(filename+".txt", FILE_WRITE);

    // Debug to make sure that there are available bytes to 
    // be read and written
    Serial.print("Bytes to be written: ");
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

    if (filetype == 1) {
      pgaData.lat = gpsData.lat;
      pgaData.lng = pgaData.lng;
      pgaData.alt = pgaData.alt;
      pgaData.date = pgaData.date;
      pgaData.time = pgaData.time;
    }

    // To make sure that all bytes will be processed
    while (dataFile.available() > 0) {

      // Converting binary data to respective data type and variable names
      dataFile.read((uint8_t *)&accelData, sizeof(accelData));

      // Saving accel data as characters in txtFile
      txtFile.print(accelData.PPS);
      txtFile.print(",");
      txtFile.print(accelData.tstamp);
      txtFile.print(",");
      txtFile.print(accelData.ax);
      txtFile.print(",");
      txtFile.print(accelData.ay);
      txtFile.print(",");
      txtFile.print(accelData.az);
      txtFile.print("\n");

      if (filetype == 0) {
        backgroundData.count += 1;
        backgroundData.ax += accelData.ax;
        backgroundData.ay += accelData.ay;
        backgroundData.az += accelData.az;
      }

      if (filetype == 1) {
        current_mag = calculate_mag(accelData.ax, accelData.ay, accelData.az);
        if (current_mag > pgaData.mag) {
          pgaData.ax = accelData.ax;
          pgaData.ay = accelData.ay;
          pgaData.az = accelData.az;
          pgaData.mag = current_mag;
        }
      }

      saved = true;   // Data now saved in txtfile

      // Debugging to know how many bytes were left to be converted
      Serial.print("Bytes to be written: ");
      Serial.println(dataFile.available());
    }
  }

  dataFile.close();   // Properly closing the temporary file
  
  txtFile.close();    // Flushing data to txtfile

  Serial.println("Saving done!");   // For debugging purposes
}

void process_background() {
  backgroundData.ax /= backgroundData.count;
  backgroundData.ay /= backgroundData.count;
  backgroundData.az /= backgroundData.count;
}

void get_background_data() {
  preparing_datagathering(0);
  
  int num_ticks = 0;
  while (num_ticks < 5) {
    if (detect_SQW()) {
      num_ticks += get_data();
    }
  }
  
  dataFile.close();
  saving_data(0);
  process_background();
}

void setup() {
 
  Serial.begin(115200);

  // GPS
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  
 
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  setupLoRa();
  packet_t.path[0] = id;

  myPacket();
  sendPacket();

  // initialize SD Card
//  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if(!SD.begin(SD_CS, spiSD, SD_speed)) {
    Serial.println("card mount failed");
    return;
  }

  else {
    Serial.print("card mounted");
  }

   // fast i2c (400kHz)
  Wire.begin(SDA, SCL, 400000);

  // initialize accelerometer
  Serial.println("Initializing I2C devices...");
  imu.begin();

  // Preparing pins for SQW and PPS
  pinMode(SQWPIN, INPUT_PULLUP);
  pinMode(PPSPIN, INPUT);

  // Initialize DS3231 SQW
  SQW_init();

  // initialize dataFile in write mode
  dataFile = SD.open("/datalog.dat", FILE_WRITE);
  if (!dataFile) {
    Serial.print("Failed to open file");
  }

  reset:
    if (standby == true) {
      Serial.println("waiting for command packets...");
      while (standby == true) {
        receivePacket();
      }
    }

  //case 0: 
  if (packet_t.command == 0) {
    for (int h = 0; h < 10; h++) {
      if (packet_t.path[h] == id) {
        Serial.println("ignore redundant packet");
        break;
      }

      if (packet_t.path[h] == 0) {
        packet_t.path[h] = id;
        sendPacket();
        break;
      }
    }
    standby = true;
    Serial.println("exiting... reset");
    goto reset;
  }

  //case 1: 
  if (packet_t.command == 1) {
    if (packet_t.level < level) {
      level = packet_t.level + 1;
      packet_t.level = level;
      packet_t.id = id;
      sendPacket();
    }
    xTaskCreatePinnedToCore(rTask, "receive packet", 1024, NULL, 1, &rT, core);
    xTaskCreatePinnedToCore(sTask, "send packet", 1024, NULL, 1, &sT, core);

    unsigned long s = millis();
    do{}while(millis() - s < relayModeTime);
    
    standby = true;

    // Wait for valid GPS location
    wait_for_GPS_location();

    // Insert here getting background data of accelerometer
    // and saving it to SD card while getting average
    
    Serial.println("exiting... reset");
    goto reset;
  }

  //case 2:
  if (packet_t.command == 2) {

    if (packet_t.level < level) {
      packet_t.level = level;
      sendPacket();
      Serial.println("start data logger");
      tsave = millis(); 

      // Preparation for data gathering
      preparing_datagathering(1);

      // This will break when a stop request is received
      // Replace this with command that will stop data gathering via LoRa
      while (!stopGathering()) {
    
        // Will only get data when SQW pulses
        // This will make data gathering restricted to 1024 sps
        if (detect_SQW()) {
          get_data();
        }
      }
      
      // Flushing data to temporary file
      dataFile.close();
    
      // Saving gathered data into TXT file
      saving_data(1);
    }
    standby = true;
    goto reset;
  }
}

void loop() {
  Serial.println("LOOP");
  delay(500);
}
