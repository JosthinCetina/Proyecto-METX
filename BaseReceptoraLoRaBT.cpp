#include <SPI.h>
#include <LoRa.h>
#include "BluetoothSerial.h"

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Iniciando Receptor LoRa...");
  
  // Nombre para identificar fácilmente en PC
  SerialBT.begin("ESP32_LoRa_PC");
  
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433.5E6)) {
    Serial.println("Error LoRa!");
    while (true);
  }

  Serial.println("Listo - Busca: ESP32_LoRa_PC en tu PC");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String mensaje = "";
    while (LoRa.available()) {
      mensaje += (char)LoRa.read();
    }
    Serial.println(mensaje);
    SerialBT.println(mensaje);  // Envía a Bluetooth (PC)
  }
  delay(50);
}
