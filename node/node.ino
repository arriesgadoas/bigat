/**********************************************************
 * BIGAT node sketch                                      
 * author: Ali                                           
 *                                                       
 * Sketch for BIGAT nodes to establish a mesh network     
 * by creating node levels (supernodes).                 
 *                                                       
 * Utilizes the parallel task capability of ESP32 to     
 * allow "simultaneous" receiver and transmitter         
 * mode of the Lora module.                              
 *                                                       
 * packet_t.command values
 * 0 ---> test connection
 * 1 ---> setup level
 * 2 ---> start data logging
 * 3 ---> stop data logging  
 *                                                     
 **********************************************************/

//libraries
#include <LoRa.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>

//constants
#define LoRa_SCK 5
#define LoRa_MISO 19
#define LoRa_MOSI 27
#define LoRa_CS 18
#define LoRa_RST 14
#define LoRa_IRQ 26

#define SD_SCK 14
#define SD_MISO 2
#define SD_MOSI 15
#define SD_CS 13


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

struct dataStrore {
  unsigned long tstamp;
  int16_t ax;
  int16_t ay;
  int16_t az;  
};

//variable declarations
static TaskHandle_t rT = NULL;
static TaskHandle_t sT = NULL;
static const BaseType_t core = 1;
boolean standby = true;
byte level = 100;
unsigned long relayModeTime = 180000;
const byte id = 1;
struct packet packet_t;

unsigned long tsave;
File dataFile;        // dataFile for temporary storage (binary)
File txtFile;         // txtFile for user readable storage
bool saved = true;    // boolean to check if data is saved to text
struct dataStore myData;


//variables used for data log testing; to be deleted later
long counter = 0;


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


void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  setupLoRa();
  packet_t.path[0] = id;

  myPacket();
  sendPacket();

  // initialize SD Card
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if(!SD.begin(SD_CS)) {
    Serial.println("card mount failed");
    return;
  }

  // initialize dataFile in write mode
  dataFile = SD.open("/datalog.dat", FILE_WRITE);
  if (!dataFile) {
    Serial.print("Failed to open file");
  }

  // initial data values
  tsave = millis();
  myData.tstamp = micros();

  // variables for data logging test; to be deleted later
  myData.ax = random(32767);
  myData.ay = random(32767);
  myData.az = random(32767);

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
      sendPacket();
    }
    xTaskCreatePinnedToCore(rTask, "receive packet", 1024, NULL, 1, &rT, core);
    xTaskCreatePinnedToCore(sTask, "send packet", 1024, NULL, 1, &sT, core);

    unsigned long s = millis();
    do{}while(millis() - s < relayModeTime);
    
    standby = true;
    Serial.println("exiting... reset");
    goto reset;
  }

  //case 2:
  if (packet_t.command == 2) {

    if (packet_t.level < level) {
      packet_t.level = level;
      sendPacket();
      Serial.println("start data logger");
      loop();
    }

  }
}


void loop() {

  //put data logging code here
  Serial.println("logging data");

  while (counter < 20000) {
    myData.tstamp = millis();
    counter ++;
    dataFile.write((const uint8_t *)&myData, sizeof(myData));
    saved = false;
  }

  if (!saved) {
    dataFile = SD.open("/datalog.dat", FILE_READ);
    txtFile = SD.open("/txtlog.dat", FILE_WRITE);

    Serial.println(dataFile.available());

    while (dataFile.available() > 0) {
      dataFile.read((uint8_t *)&myData, sizeof(myData));

      // Saving data as characters in txtFile
      txtFile.print(myData.tstamp);
      txtFile.print(",");
      txtFile.print(myData.ax);
      txtFile.print(",");
      txtFile.print(myData.ay);
      txtFile.print(",");
      txtFile.print(myData.az);
      txtFile.print("\n");

      // Debugging to know how many bytes were left to be converted
      Serial.println(dataFile.available());
    }
  }

//  delay(1000);
}
