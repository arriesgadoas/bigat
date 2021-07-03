#include <LoRa.h>
#include <SPI.h>

#define LoRa_SCK 5
#define LoRa_MISO 19
#define LoRa_MOSI 27
#define LoRa_CS 18
#define LoRa_RST 14
#define LoRa_IRQ 26

//define task handlers for receive and send packet tasks
static TaskHandle_t rT = NULL;
static TaskHandle_t sT = NULL;

//specify cpu core where receive packet task and send packet task will run
static const BaseType_t core = 1;

//node state variables
boolean standby = true;
byte level = 100;

//relay mode time
unsigned long relayModeTime = 180000;

//node id --> set unique ID for each bigat node; can be saved to EEPROM
const byte id = 1;

//packet structure definition
typedef struct packet {
  byte key;
  byte command;
  byte level;
  byte id;
  byte path[10];
};

//create a packet instance
struct packet packet_t;

//RTOS receive task
void rTask(void *param) {
  //start timer for relay mode; length of relay mode is deifned by relay mode time
  unsigned long start = millis();
  do{
    //initialize standby to TRUE to enter relay mode
    standby = true;
    if(standby == true){
      Serial.println("waiting for packets...");
      //when packet is received, standby = FALSE; see receivePacket() function for reference
      while(standby == true){
          receivePacket();
        }
      }

     //this part only executes when a valid packet is received (standby = false)
     //check if packet's level is higher since at this point all packet's directions must be downward
     //(i.e. towards the base station)
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

    Serial.println("exiting... reset");
    standby = true;
    esp_restart();
}

//RTOS send task
void sTask(void *param) {
  int rD;
  //wait for random length of time between zero and the declared relay mode time to send node information (id and level)
  rD = random(relayModeTime);
  vTaskDelay(rD / portTICK_PERIOD_MS);
  //create an id packet that contains the node information (id and level)
  myPacket(packet_t);
  sendPacket();
}

//required LoRa config
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


//receives packet and checks packet validity by checking network's passkey;
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

//send current packet_t
void sendPacket() {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&packet_t, sizeof(packet_t));
  LoRa.endPacket();
  Serial.println("packet forwarded");
}

//print contents of packet_t; only used for debugging
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

//packet containg node information
void myPacket(struct packet p){
  p.key = 83;
  p.command = 1;
  p.level = level;
  p.id = id;
  //set first point of path as own id
  p.path[0] = id;
  }

void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  setupLoRa();
  packet_t.path[0] = id;

  if (standby == true) {
    //wait for packet until a valid packet is received
    Serial.println("waiting for command packets...");
    while (standby == true) {
      receivePacket();
    }
  }

  /**************************************/
  // packet_t.command values
  // 0 ---> test connection
  // 1 ---> setup level
  // 2 ---> start data logging
  // 3 ---> stop data logging
  /**************************************/

  //case 0: node receives test connection packet
  if (packet_t.command == 0) {
    for (int h = 0; h < 10; h++) {
      if (packet_t.path[h] == id) {
        Serial.println("ignore redundant packet");
        break;
      }

      if (packet_t.path[h] == 0) {
        packet_t.path[h] = id;
        printPacket();
        sendPacket();
        break;
      }
    }
    standby = true;
    Serial.println("exiting... reset");
    esp_restart();
  }

  //case 1: setup command is received
  if (packet_t.command == 1) {
    //check if sender level is lower than receiver node's level
    if (packet_t.level < level) {
      //get level of sender then add 1
      level = packet_t.level + 1;
      //update packet_t's level
      packet_t.level = level;
      //send packet for other nodes
      printPacket();
      sendPacket();
    }
    //run parallel tasks to forward packets from other nodes and send own information packet
    xTaskCreatePinnedToCore(rTask, "receive packet", 1024, NULL, 1, &rT, core);
    xTaskCreatePinnedToCore(sTask, "send packet", 1024, NULL, 1, &sT, core);

    //delete rTask and sTask
    if(rT != NULL){
      vTaskDelete(rT);
      rT == NULL;
      }

    if(sT != NULL){
      vTaskDelete(sT);
      sT == NULL;
      }
  }

  //case 2: start data logging command is received
  if (packet_t.command == 2) {

    //check if sender level is lower than receiver node's level
    if (packet_t.level < level) {
      //update packet_t's level
      packet_t.level = level;
      //send start command packet for other nodes
      sendPacket();
      //start data logging by going to loop function
      Serial.println("start data logger");
      loop();
    }

  }
}

void loop() {

  //put data logging code here
  Serial.println("logging data");

  delay(1000);
}
