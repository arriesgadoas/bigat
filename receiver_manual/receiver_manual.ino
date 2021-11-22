#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa


// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LORA_CS 18     // GPIO18 -   SX1276 CS
#define LORA_RST 14   // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)U

typedef struct packet {
  byte key;                       //passkey
  byte command;                   //0,1,2, or 3
  byte level;                     //level assigend to a node
  byte id;                        //node id must be unique
  int path[10];                  //track packet path[sorce_node id, 2nd_hop_node id, 3rd_hop_node id,...]
};


struct packet packet_t;
int packet_t_size = sizeof(packet_t);

struct peakAccel {
  byte key;
  byte id;
  byte level;
  int path[10];
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
  double mag_calc;
};

struct peakAccel pgaData;
int pgaData_size = sizeof(pgaData);

byte id;
byte level;
String path;
String toSerial;
double lat_;                 // Latitude
double lng_;                 // Longitude
int32_t alt;                // Altitude in cm
uint32_t date_;              // Date of getting the data
uint32_t time_;              // Time of getting the data
int16_t b_ax;
int16_t b_ay;
int16_t b_az;
int16_t ax;
int16_t ay;
int16_t az;
int32_t mag;
double mag_calc;

int serial_buf;


void setupLora() {
  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Very important for LoRa Radio pin configuration!
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);

  }

  LoRa.setSpreadingFactor(7);
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
}


void sendPacket() {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&packet_t, sizeof(packet_t));
  LoRa.endPacket();
}

void createCommandPacket(int c, int l) {
  packet_t.command = c;
  packet_t.level = 0;
  packet_t.id = 0;
  packet_t.key = 83;
  packet_t.path[0] = 0;
}


/**********************************************************
   packet_t.command values
   0 ---> test connection
   1 ---> setup level
   2 ---> start data logging
   3 ---> stop data logging

 **********************************************************/

void getNodes() {
  int packetSize = LoRa.parsePacket ();
  if (packetSize) // Only read if there is some data to read..
  {
    if (packetSize == packet_t_size) {
      LoRa.readBytes((uint8_t *)&packet_t, packetSize);
      if (packet_t.key = 83) {
        id = packet_t.id;
        toSerial = "Node connected: " + String(packet_t.id);
        Serial.println(toSerial);
      }
    }
    else {
      LoRa.flush();
    }
  }
}

void setupNetwork() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) // Only read if there is some data to read..
  {
    Serial.print("RECEIVED ");
    // Crashing here
    if (packetSize == packet_t_size) {
      LoRa.readBytes((uint8_t *)&packet_t, packetSize);
    
      if (packet_t.key == 83) {
        path = "";
        Serial.print("VALID :");
        if (packet_t.path[0] != 0) {
          id = packet_t.id;
          level = packet_t.level;
          for (int i = 0; i < 10; i++) {
            path += String(packet_t.path[i]);
            if (i != 9) {
              path += "-";
            }
          }
          toSerial = "{id:" + String(id) + "," + "level:" + String(level) + "," + "path:" + path + "}";
          Serial.println(toSerial);
        }
      }
    }
    else {
      LoRa.flush();
    }
  }
}

void stopLogging() {
  // put your main code here, to run repeatedly:
  int packetSize = LoRa.parsePacket ();
  if (packetSize) // Only read if there is some data to read..
  {
    if (packetSize == pgaData_size) {
      Serial.print("RECEIVED:");
      LoRa.readBytes((uint8_t *)&pgaData, packetSize);
      Serial.println(pgaData.key);
      if (pgaData.key == 84) {
        path = "";
        if (pgaData.path[0] != 0) {
          id = pgaData.id;
          level = pgaData.level;
          for (int i = 0; i < 10; i++) {
            path += String(pgaData.path[i]);
            if (i != 9) {
              path += "-";
            }
          }
        }
        lat_ = pgaData.lat;
        lng_ = pgaData.lng;
        alt = pgaData.alt;
        date_ = pgaData.date;
        time_ = pgaData.time;
        ax = pgaData.ax;
        ay = pgaData.ay;
        az = pgaData.az;
        mag_calc = pgaData.mag_calc;
        toSerial = "{id:" + String(id) + "," + "level:" + String(level) + "," + "path:" + path + "," + "lat:" + String(lat_) + "," + "lng:" + String(lng_) + "," + "alt:" + String(alt) + "\n" + "date:" + String(date_) + "," + "time:" + String(time_) + "," + "mag:" + String(mag_calc,10) + "}";
        Serial.println(toSerial);
      }
    }
    else {
      LoRa.flush();
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  setupLora();
  Serial.println();
  Serial.println("****************************************");
  Serial.println("COMMANDS: ");
  Serial.println("1: \t Test Node Connection");
  Serial.println("2: \t Setup Network");
  Serial.println("3: \t Start Data Gathering");
  Serial.println("4: \t Stop Data Gathering");
  Serial.println("****************************************");
  Serial.println();
  while (!Serial.available()) {};
  serial_buf = Serial.parseInt();
}

void loop() {
  if (serial_buf == 1) {
    Serial.println("Testing node connection...");
    while (Serial.available() > 0) { Serial.read(); }
    do {
      getNodes(); } while (!Serial.available());
  }
  else if (serial_buf == 2) {
    Serial.println("Setting up network...");
    while (Serial.available() > 0) { Serial.read(); }
    createCommandPacket(1, 0);
    sendPacket();
    do {
      setupNetwork();
    } while (!Serial.available());
  }
  else if (serial_buf == 3) {
    Serial.println("Starting data logging...");
    while (Serial.available() > 0) { Serial.read(); }
    createCommandPacket(2, 0);
    sendPacket();
  }
  else if (serial_buf == 4) {
    Serial.println("Stopping data logging...");
    while (Serial.available() > 0) { Serial.read(); }
    createCommandPacket(3, 0);
    sendPacket();
    do {
      stopLogging();
    } while (!Serial.available());
  }
  else {
    Serial.println();
    Serial.println("****************************************");
    Serial.println("COMMANDS: ");
    Serial.println("1: \t Test Node Connection");
    Serial.println("2: \t Setup Network");
    Serial.println("3: \t Start Data Gathering");
    Serial.println("4: \t Stop Data Gathering");
    Serial.println("****************************************");
    Serial.println();
  }
  serial_buf = 10;
  while (!Serial.available()) {};
  serial_buf = Serial.parseInt();
//  if (Serial.available()) { serial_buf = Serial.parseInt(); }
}
