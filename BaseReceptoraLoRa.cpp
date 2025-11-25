#include <SPI.h>
#include <LoRa.h>

// Definición de pines para ESP32
#define SCK   5       
#define MISO  19      
#define MOSI  27      
#define SS    18      
#define RST   14      
#define DIO0  26      

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Inicializando Receptor LoRa (ESP32)...");

  // Iniciar SPI con los pines definidos
  SPI.begin(SCK, MISO, MOSI, SS);

  // Configuración de pines LoRa
  LoRa.setPins(SS, RST, DIO0);

  // Inicia LoRa en 433.5 MHz (igual al emisor)
  if (!LoRa.begin(433.5E6)) {
    Serial.println("Error: no se pudo inicializar LoRa.");
    while (true);
  }

  // ====== AJUSTES PARA RECIBIR TU TRANSMISIÓN ======

  // Spreading Factor = 9
  LoRa.setSpreadingFactor(9);

  // Bandwidth = 125 kHz
  LoRa.setSignalBandwidth(125E3);

  // Coding Rate = 4/6
  LoRa.setCodingRate4(6);

  // Modo de ganancia automática (recomendado)
  LoRa.enableCrc();   // CRC debe coincidir con el emisor
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    Serial.println("-----------------------------");
    Serial.print("Paquete recibido: ");

    String mensaje = "";
    while (LoRa.available()) {
      mensaje += (char)LoRa.read();
    }
    Serial.println(mensaje);
  }
}
