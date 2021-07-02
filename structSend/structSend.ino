#include <LoRa.h>

#define LoRa_SCK 5        // GPIO5 - SX1276 SCK
#define LoRa_MISO 19     // GPIO19 - SX1276 MISO
#define LoRa_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LoRa_CS 18     // GPIO18 -   SX1276 CS
#define LoRa_RST 14   // GPIO14 -    SX1276 RST
#define LoRa_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)

typedef struct r {
  byte key;
  byte command;
  byte level;
  byte id;
  byte path[10];
};

struct r p;

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

void populateStruct(byte c, byte l, byte i) {
  p.key = 83;
  p.command = c;
  p.level = l;
  p.id = i;
  p.path[0] = i;
}

void sendPacket() {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&p, sizeof(p));
  LoRa.endPacket();
  Serial.println("packet forwarded");
}

void printPacket() {
  String path = (char*)p.path;
  Serial.print("key: "); Serial.println(p.key);
  Serial.print("command: "); Serial.println(p.command);
  Serial.print("level: "); Serial.println(p.level);
  Serial.print("id: "); Serial.println(p.id);
  Serial.print("path: "); Serial.println(path);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupLoRa();
  populateStruct(0, 2, 5);
  printPacket();
  sendPacket();
}

void loop() {}
