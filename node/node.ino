#include <LoRa.h>
#include <SPI.h>

#define LoRa_SCK 5 
#define LoRa_MISO 19
#define LoRa_MOSI 27 
#define LoRa_CS 18    
#define LoRa_RST 14
#define LoRa_IRQ 26

//node state variables
boolean standby = true;
byte level = 100;

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

void setup() {

  Serial.begin(115200);
  delay(1000);
  setupLoRa();
  packet_t.path[0] = id;

//reset
start:
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
    for(int h = 0; h < 10; h++){
      if(packet_t.path[h] == id){
          Serial.println("ignore redundant packet");
          break;
        }
        
      if(packet_t.path[h] == 0){
          packet_t.path[h] = id;
          printPacket();
          sendPacket();
          break;
        }
      }
    standby = true;
    goto start;
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

    //delay relative to level for downward packet success
    delay((1000 * level)+5000);
    int randDelay = random(5000);
    delay(randDelay);
    //set proper level for reporting to base satation
    packet_t.level = level;
    packet_t.id = id;
    printPacket();
    sendPacket();

    //3 mins. relay mode for reporting to base station
    unsigned long start = millis();
    do {
      standby = true;
      if (standby == true) {
        Serial.println("waiting for node id and level...");
        while (standby == true && millis() - start < 180000) {
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

      else{
        Serial.println("ignore low level packet");
        }
    }
    while (millis() - start < 180000);
    Serial.println("exiting... reset");
    standby = true;
    goto start;
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
