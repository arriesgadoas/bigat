//libraries
#include <LoRa.h>
#include <SPI.h>
#include "SD_MMC.h"
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

// For I2C devices (accelerometer, RTC)
#define SDA 21
#define SCL 22

#define RXD1 36   // For NEO-6M GPS
#define TXD1 39   // For NEO-6M GPS

#define PPSPIN 35 // For NEO-6M GPS (PPS = pulse per second)
#define SQWPIN 25 // For DS3231 RTC (SQW = square wave)

// DS3231 registers
#define RTC_I2C_ADDRESS 0x68
#define RTC_CONTROL 0x0E
#define RTC_SQW_1HZ 0x00
#define RTC_SQW_1024HZ 0x08
#define RTC_SQW_4096HZ 0x10

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
//void printPacket();               //for debugging purposes
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
  int16_t tstamp;             // There are tstamp/256 seconds per tick of PPS
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
  byte key;
  byte id;
  byte level;
  byte path[10];
  double lat;                 // Latitude
  double lng;                 // Longitude
  int32_t alt;                // Altitude in cm
  uint32_t date;              // Date of getting the data
  uint32_t time;              // Time of getting the data
  int16_t b_ax;
  int16_t b_ay;
  int16_t b_az;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int32_t mag;
};

int32_t current_mag;

//variable declarations
//static TaskHandle_t rT = NULL;
//static TaskHandle_t sT = NULL;
//static const BaseType_t core = 1;
boolean standby = true;
boolean logger = true;
byte level = 100;
unsigned long relayModeTime = 10000;
const byte id = 3;
struct packet packet_t;
byte key = 83;
byte key2 = 84;

unsigned long tsave;
File dataFile;        // dataFile for temporary storage (binary)
File txtFile;         // txtFile for user readable storage
bool saved = true;    // boolean to check if data is saved to text

String filename;      // File name. Format = <devID>_YYYYMMDD_HHMMSSCC. See preparing_datagathering()

// For detection of changes in PPS. See PPS_tick()
int old_PPS = LOW;
int new_PPS;

// For detection of changes in SQW
int sqw_count = 0;
int count = 0;

// Initializing data structs needed
struct dataStore accelData;
struct gpsVariables gpsData;
struct background backgroundData;
struct peakAccel pgaData;
struct peakAccel pgaDataOtherNodes;

// Variables for SD buffer
const int sdBufSize = 16;
struct dataStore toSD[2][sdBufSize];
uint8_t toSD_arrayCount = 0;
bool toSD_arrayFull = false;
bool toSD_arrayChange = false;
bool toSD_arrayFull_copy;

bool isBackground;


void rTask(void *param) {
  unsigned long start = millis();
  do {
    //initialize standby to TRUE to enter relay mode
    standby = true;
    if (standby == true) {
      Serial.println("waiting for packets...");
      while (standby == true && millis() - start < relayModeTime) {
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
      //printPacket();
      sendPacket();
    }

    else {
      Serial.println("ignore low level packet");
    }
  } while (millis() - start < relayModeTime);
  Serial.println("relay time finished");
  vTaskDelete(NULL);
}


void sTask(void *param) {
  int rD;
  rD = random(relayModeTime);
  vTaskDelay((rD + 10000) / portTICK_PERIOD_MS);
  myPacket(1);
  sendPacket();
  vTaskDelete(NULL);
}


void rPeakTask(void *param) {
  unsigned long start = millis();
  do {
    //initialize standby to TRUE to enter relay mode
    standby = true;
    if (standby == true) {
      Serial.println("waiting for packets...");
      while (standby == true && millis() - start < relayModeTime) {
        receivePeakPacket();
      }
    }

    if (pgaDataOtherNodes.level > level) {
      for (int i = 0; i < 10; i++) {
        if (pgaDataOtherNodes.path[i] == 0) {
          pgaDataOtherNodes.path[i] = id;
          break;
        }
        if (i == 9) {
          break;
        }
      }
      //printPacket();
      sendPeakPacket(pgaDataOtherNodes);
    }

    else {
      Serial.println("ignore low level packet");
    }
  } while (millis() - start < relayModeTime);

  vTaskDelete(NULL);
}


void sPeakTask(void *param) {
  int rD;
  rD = random(relayModeTime);
  vTaskDelay((rD + 10000) / portTICK_PERIOD_MS);
  //Assign key, id, and level for LoRa transmission of peak g
  pgaData.id = id;
  pgaData.key = key2;
  pgaData.level = level;
  sendPeakPacket(pgaData);
  vTaskDelete(NULL);
}

void stopTask(void *param) {
  while (1) {
    receivePacket();
    if (packet_t.level < level) {
      if (packet_t.command == 3) {
        packet_t.level = level;
        sendPacket();
        logger = false;
        break;
      }
    }
    if (toSD_arrayChange != toSD_arrayFull) {
      Serial.println();
      Serial.println("SD");
      for (int i=0; i<sdBufSize; i++) {
        dataFile.write((const uint8_t *)&toSD[toSD_arrayChange][i], sizeof(toSD[toSD_arrayChange][i]));
      }
      dataFile.flush();
      toSD_arrayChange = !toSD_arrayChange;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
  
  vTaskDelete(NULL);
}

void dTask(void *param) {
  Serial.print("dtask: ");
  Serial.println(logger);
  while (logger == true) {
    if (sqw_count <= 0) {
      sqw_count = 4;
      get_data();
    }
  }
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
    if (packet_t.key == key) {
      Serial.println("received command packet");
      standby = false;
    }

    else {
      Serial.println("invalid command packet");
      standby = true;
    }
  }
}


void receivePeakPacket() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    LoRa.readBytes((uint8_t*)&pgaDataOtherNodes, packetSize);
    if (pgaDataOtherNodes.key == key2) {
      Serial.println("received peak g packet");
      standby = false;
    }

    else {
      Serial.println("invalid packet");
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

void sendPeakPacket(peakAccel s) {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&s, sizeof(s));
  LoRa.endPacket();
  Serial.println("packet forwarded");
}


//void printPacket() {
//  Serial.print("id: ");
//  Serial.print(packet_t.id);
//  Serial.print("\t");
//  Serial.print("level: ");
//  Serial.print(packet_t.level);
//  Serial.print("\t");
//  Serial.print("path: ");
//  for (int i = 0; i < 10; i++ ) {
//    Serial.print(packet_t.path[i]);
//    Serial.print(" ");
//    if (i == 9) {
//      Serial.print("\n");
//      break;
//    }
//  }
//}


void myPacket(byte c) {
  packet_t.key = key;
  packet_t.command = c;
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
void LSM6DSL_getacceldata() {
  imu.readRawAccel3D(&accelData.ax, &accelData.ay, &accelData.az);
}

// https://github.com/00steve00/Super-Accurate-Arduino-Clock/blob/master/GPS-RTC-Clock.ino
byte dec2bcd(byte n) {
  uint16_t a = n;
  byte b = (a * 103) >> 10;
  return  n + b * 6;
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
  sendRTC(RTC_CONTROL, RTC_SQW_1HZ);
}

// Detect if SQW of DS3231 pulses or not
void detect_SQW() {
  sqw_count--;
}

// Debugging purposes To check if GPS is receiving data from satellites. Takes around 5 mins
// Time is first synchronized, then date, then # of satellites, then location
void displayInfo()
{
  if (gps.location.isValid()) {
    Serial.print("Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else {
    Serial.println("Location: Not Available");
  }

  Serial.print("Date: ");
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid()) {
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
  else {
    Serial.println("Not Available");
  }
  Serial.print("# of Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.println();
  Serial.println();
}

// Wait for valid location from GPS
void wait_for_GPS_location() {
  while (!gps.location.isValid()) {
    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read())) {
        displayInfo();
      }
    }
  }

  // 5 seconds of delay alloted for location to relax
  delay(5000);
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  gpsData.lat = gps.location.lat();
  gpsData.lng = gps.location.lng();
  gpsData.alt = gps.altitude.value();

  // Debugging purposes
  Serial.print("GPS DONE: ");
  Serial.print(gpsData.lat, 8);
  Serial.print(",");
  Serial.print(gpsData.lng, 8);
  Serial.print(" | Alt (cm): ");
  Serial.println(gpsData.alt, 8);

}

// GPS PPS. Ticks every 1 second and is synchronized for every GPS with typical difference of < 10ms.
bool PPS_tick() {
  new_PPS = digitalRead(PPSPIN);
  bool tick = (old_PPS == LOW && new_PPS == HIGH);
  old_PPS = new_PPS;
  return tick;
}

// Calculate max change of acceleration between background and actual data
int32_t calculate_mag(int32_t ax, int32_t ay, int32_t az) {
  int32_t dx = ax - backgroundData.ax;
  int32_t dy = ay - backgroundData.ay;
  int32_t dz = az - backgroundData.az;
  return sqrt(sq(dx) + sq(dy) + sq(dz));
}

// Preparing data gathering
void preparing_datagathering(int filetype) {

  detachInterrupt(digitalPinToInterrupt(SQWPIN));
  sqw_count = 0;
  
  // Getting latest GPS data for time and date
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Assembling filename
  filename = "/";
  filename += String(id);
  filename += "_";
  gpsData.date = (gps.date.year()*10000) + (gps.date.month()*100) + gps.date.day();
  filename += (String)gpsData.date;
  filename += "_";
  gpsData.time = gps.time.value() / 100;

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
  while (!PPS_tick()) {};         // Waiting for next tick
  
  sendRTC(0x00,dec2bcd(0));                                    // Synchronizing GPS PPS with RTC SQW
  dataFile = SD_MMC.open(filename+".dat", FILE_WRITE);         // Creating temporary file for data
  Serial.println("RTC START");
  while (!PPS_tick()) {};         // Waiting for next tick
  
  sendRTC(RTC_CONTROL,RTC_SQW_1024HZ);          // Making DS3231 generate 1024 Hz SQW
  accelData.tstamp = 0;                         // Resetting timestamp
  Serial.println("RTC CONFIG");
  attachInterrupt(digitalPinToInterrupt(SQWPIN), detect_SQW, FALLING);
  
  while (!PPS_tick()) {};         // After this, data gathering starts
}

void savetoSD( void * parameter ) {
  for (int i=0; i<sdBufSize; i++) {
    dataFile.write((const uint8_t *)&toSD[*((bool*)parameter)][i], sizeof(toSD[*((bool*)parameter)][i]));
  }
  dataFile.flush();
  vTaskDelete(NULL);
}

bool get_data() {
  
  // Detecting PPS tick
  bool buf_tick = PPS_tick();

  // SQW count resets when PPS ticks
  if (buf_tick) { accelData.tstamp = 0;}
  
  // Getting accelerometer data
  LSM6DSL_getacceldata();

  toSD[toSD_arrayFull][toSD_arrayCount] = accelData;
  if (accelData.tstamp >= 254) {
    Serial.println(accelData.tstamp);
  }

  toSD_arrayCount++;
  
  if (toSD_arrayCount >= sdBufSize) {
    if (isBackground) {
      toSD_arrayFull_copy = toSD_arrayFull;
      xTaskCreatePinnedToCore(savetoSD, "save to SD card", 8096, (void*)&toSD_arrayFull_copy, 1, NULL, 0);
    }
    toSD_arrayCount = 0;
    toSD_arrayFull = !toSD_arrayFull;
  }
//
//  // This increases every time SQW pulses
  accelData.tstamp++;
  
  saved = false;
  return buf_tick;
}

bool saving_data(int filetype) {
  detachInterrupt(digitalPinToInterrupt(SQWPIN));
  if (!saved) {

    // Opening binary file, now in read mode
    dataFile = SD_MMC.open(filename+".dat", FILE_READ);

    // Opening txtFile in write mode
    txtFile = SD_MMC.open(filename+".txt", FILE_WRITE);

    // Debug to make sure that there are available bytes to 
    // be read and written
    Serial.print("Bytes to be written: ");
    Serial.println(dataFile.available());

    // Writing GPS data to txtfile
    txtFile.print(gpsData.lat, 10); txtFile.print(",");
    txtFile.print(gpsData.lng, 10); txtFile.print(",");
    txtFile.print(gpsData.alt); txtFile.print(",");
    txtFile.print(gpsData.date); txtFile.print(",");
    txtFile.print(gpsData.time); txtFile.print("\n");

    if (filetype == 1) {
      pgaData.lat = gpsData.lat;
      pgaData.lng = gpsData.lng;
      pgaData.alt = gpsData.alt;
      pgaData.date = gpsData.date;
      pgaData.time = gpsData.time;
    }

    
    Serial.print("Bytes to be written: ");
    uint32_t file_size = dataFile.available();
    unsigned long time_saving = millis();
    // To make sure that all bytes will be processed
    while (dataFile.available() > 0) {

      // Converting binary data to respective data type and variable names
      dataFile.read((uint8_t *)&accelData, sizeof(accelData));

      // Saving accel data as characters in txtFile
      txtFile.print(accelData.tstamp); txtFile.print(",");
      txtFile.print(accelData.ax); txtFile.print(",");
      txtFile.print(accelData.ay); txtFile.print(",");
      txtFile.print(accelData.az); txtFile.print("\n");

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

      if ((unsigned long)(millis() - time_saving) > 1000) {
        time_saving = millis();
        Serial.print("Saving data: "); 
        Serial.print(100 - (int)((dataFile.available() * 100.0) / (file_size * 1.0))); Serial.print("% \t\t");
        Serial.print(file_size - dataFile.available()); Serial.print(" out of "); Serial.println(file_size);
      }
    }
    Serial.println("Saving data: 100%");
    if (filetype == 1) {
      pgaData.b_ax = backgroundData.ax;
      pgaData.b_ay = backgroundData.ay;
      pgaData.b_az = backgroundData.az;
      
      Serial.print("Background: ");
      Serial.print(backgroundData.ax); Serial.print(", ");
      Serial.print(backgroundData.ay); Serial.print(", ");
      Serial.println(backgroundData.az);
      
      Serial.print("PGA:");
      Serial.print(pgaData.ax); Serial.print(", ");
      Serial.print(pgaData.ay); Serial.print(", ");
      Serial.println(pgaData.az);

      Serial.print("DISP: ");
      Serial.println(pgaData.mag);
    }
  }

  dataFile.close();   // Properly closing the temporary file
  txtFile.close();    // Flushing data to txtfile

  Serial.print("Saving done! Filename: ");   // For debugging purposes
  Serial.print(filename); Serial.println(".txt");
}

void process_background() {
  backgroundData.ax /= backgroundData.count;
  backgroundData.ay /= backgroundData.count;
  backgroundData.az /= backgroundData.count;
}

void get_background_data() {
  Serial.println("Getting background data...");
  int num_ticks = 0;
  isBackground = true;
  
  preparing_datagathering(0);
  while (num_ticks < 5) {
    Serial.print(".");
    if (sqw_count <= 0) {
      sqw_count = 4;
      num_ticks += get_data();
    }
  }
  isBackground = false;
  
  dataFile.close();
  Serial.println("Saving background data...");
  saving_data(0);
  process_background();
  Serial.println("Done: background data");
}

void setup() {

  Serial.begin(115200);

  // GPS
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);


  vTaskDelay(1000 / portTICK_PERIOD_MS);
  setupLoRa();
  packet_t.id = id;
  packet_t.path[0] = id;

  myPacket(0);
  sendPacket();

  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);

  if (!SD_MMC.begin("/sdcard", false)) {
    Serial.println("CARD MOUNT FAILED");
    return;
  }
  else {
    Serial.println("card mounted");
  }

  // fast i2c (400kHz)
  Wire.begin(SDA, SCL, 400000);

  // initialize accelerometer
  Serial.println("Initializing I2C devices...");
  LSM6DSL_init();

  // Preparing pins for SQW and PPS
  pinMode(SQWPIN, INPUT_PULLUP);
  pinMode(PPSPIN, INPUT);

  // Initialize DS3231 SQW
  SQW_init();

  // initialize dataFile in write mode
  dataFile = SD_MMC.open("/datalog.dat", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Failed to open file");
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

    // Wait for valid GPS location
    wait_for_GPS_location();
    
    if (packet_t.level < level) {
      level = packet_t.level + 1;
      packet_t.level = level;
      packet_t.id = id;
      sendPacket();
    }

    xTaskCreatePinnedToCore(rTask, "receive packet", 1024, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(sTask, "send packet", 1024, NULL, 2, NULL, 1);

    unsigned long s = millis();
    do {} while (millis() - s < relayModeTime);

    standby = true;

    Serial.println("exiting... reset");
    goto reset;
  }

  //case 2:
  if (packet_t.command == 2) {

    if (packet_t.level < level) {
      packet_t.level = level;
      sendPacket();
  
      // Insert here getting background data of accelerometer
      // and saving it to SD card while getting average
      get_background_data();
      
      Serial.println("start data logger");
      tsave = millis();

      Serial.println("1");

      logger = true;

      xTaskCreatePinnedToCore(stopTask, "stop data logging", 8096, NULL, 1, NULL, 0);
//      xTaskCreatePinnedToCore(dTask, "data log", 8096, NULL, 1, NULL, 1);

      Serial.println("2");

      // Preparation for data gathering
      preparing_datagathering(1);
      
      while (logger == true) {
        Serial.print(":");
        if (sqw_count <= 0) {
          sqw_count = 4;
          get_data();
        }
      }
      
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // Flushing data to temporary file
      dataFile.close();

      // Relay time problem. Please solve this
      xTaskCreatePinnedToCore(rPeakTask, "receive peak g packet", 1024, NULL, 1, NULL, 0);

      // Saving gathered data into TXT file
      saving_data(1);

      vTaskDelay(1000 / portTICK_PERIOD_MS);
      xTaskCreatePinnedToCore(sPeakTask, "send peak g packet", 1024, NULL, 2, NULL, 1);

      unsigned long s = millis();
      do {} while (millis() - s < relayModeTime);
    }
    standby = true;
    goto reset;
  }

  standby = true;
  goto reset;
}

void loop() {
  Serial.println("LOOP");
  delay(500);
}
