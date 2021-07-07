#include <LoRa.h>
#include <SPI.h>

#define LoRa_SCK 5
#define LoRa_MISO 19
#define LoRa_MOSI 27
#define LoRa_CS 18
#define LoRa_RST 14
#define LoRa_IRQ 26

//message string
String rMessage  = "";
char sMessage[64];

//define task handlers for receive and send packet tasks
static TaskHandle_t rT = NULL;
static TaskHandle_t sT = NULL;

//specify cpu core where receive packet task and send packet task will run
static const BaseType_t core = 1;

boolean standby = true;

//relay mode time
unsigned long relayModeTime = 60000;

void rTask(void *param) {
  unsigned long start = millis();
  standby = true;
  do {
    if (standby == true) {
      while (standby == true && millis() - start < relayModeTime) {
        receivePacket();
      }
    }
    standby = true;
    Serial.println(rMessage);
    rMessage = "";
  } while (millis() - start < relayModeTime);
  vTaskDelete(NULL);
}

void sTask(void *param) {
    int rD;
    rD = random(relayModeTime);
    Serial.println(rD);
    vTaskDelay(rD / portTICK_PERIOD_MS);
    sendPacket("message from node");
    vTaskDelete(NULL);
}


void sendPacket(String p) {
  LoRa.beginPacket();
  LoRa.print(p);
  LoRa.endPacket();
  Serial.println("packet sent");
}

void receivePacket() {
  if (LoRa.parsePacket()) {
    while (LoRa.available()) {
      rMessage = LoRa.readString();
    }
    standby = false;
  }

  else {
    standby = true;
  }
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

void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  setupLoRa();

  xTaskCreatePinnedToCore(sTask, "send packet", 1024, NULL, 1, &sT, core);
  xTaskCreatePinnedToCore(rTask, "receive packet", 1024, NULL, 1, &rT, core);
  
  unsigned long s =  millis();
  do{}while(millis() - s < relayModeTime);
  
}

void loop() {
  Serial.println("test done...");
  delay(1000);
}
