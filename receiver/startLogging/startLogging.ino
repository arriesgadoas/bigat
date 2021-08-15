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


void setupLora() {
  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Very important for LoRa Radio pin configuration!
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);

  }

  LoRa.setSpreadingFactor(12);
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
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  setupLora();

  //change first parameter for packet_t.command
  createCommandPacket(2, 0);
  sendPacket();
}
byte id;
byte level;
String path;
String toSerial;
void loop() {
 
}
