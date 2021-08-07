#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa


// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LORA_CS 18     // GPIO18 -   SX1276 CS
#define LORA_RST 14   // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)

typedef struct packet {
  byte key;                       //passkey
  byte command;                   //0,1,2, or 3
  byte level;                     //level assigend to a node
  byte id;                        //node id must be unique
  byte path[10];                  //track packet path[sorce_node id, 2nd_hop_node id, 3rd_hop_node id,...]
};

struct packet packet_t;

int id;
int level;
int command;
int path[10];
String commandS;

void setupLora() {
  // Very important for SPI pin configuration!
  Serial.println("Setting up LoRa");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Very important for LoRa Radio pin configuration!
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);

  }

  LoRa.setSpreadingFactor(12);
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);

  Serial.println("Setup LoRa done.");
}

void sendPacket() {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&packet_t, sizeof(packet_t));
  LoRa.endPacket();
}

void createCommandPacket(int c, int l) {
  Serial.print("Sending command: ");
  Serial.println(c);
  packet_t.command = c;
  packet_t.level = l;
  packet_t.id = 0;
  packet_t.key = 83;
  packet_t.path[0] = 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  setupLora();

  //xTaskCreatePinnedToCore(readSerAndSend, "serial input command", 1024, NULL, 1, &serInput, 1);
  createCommandPacket(1,0);
  sendPacket();
}


void loop() {
  // put your main code here, to run repeatedly:
  
  int packetSize = LoRa.parsePacket ();
  if (packetSize) // Only read if there is some data to read..
  {
    LoRa.readBytes((uint8_t *)&packet_t, packetSize);
    if (packet_t.key = 83) {
      id = packet_t.id;
      level = packet_t.level;
      command = packet_t.command;
      if(command == 0){
          Serial.print("Node ");Serial.print(id);Serial.println(" now connected");
        }
      else if (command == 1){
          Serial.print("id: \t");
          Serial.println(id);
          Serial.print("level: \t");
          Serial.println(level);
          Serial.print("path: \t");
          for(int i=0; i<10; i++){
              path[i] = packet_t.path[i];
              Serial.print(path[i]);Serial.print(":");
            }
          Serial.print("\n");
        }
      
    }
  }
}
