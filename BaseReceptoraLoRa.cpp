#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

// Definición de pines para ESP32
#define SCK   5       
#define MISO  19      
#define MOSI  27      
#define SS    18      
#define RST   14      
#define DIO0  26      

struct SensorData {
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
};

SensorData receivedData;

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

  // Spreading Factor = 7
  LoRa.setSpreadingFactor(7);

  // Bandwidth = 125 kHz
  LoRa.setSignalBandwidth(125E3);

  // Coding Rate = 4/6
  LoRa.setCodingRate4(6);

  // Modo de ganancia automática (recomendado)
  LoRa.enableCrc();   // CRC debe coincidir con el emisor
  LoRa.receive();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    if (packetSize == sizeof(SensorData)) {
      LoRa.readBytes((uint8_t*)&receivedData, sizeof(receivedData));
      Serial.print(receivedData.accX, 2); 
      Serial.print(";");
      Serial.print(receivedData.accY, 2); 
      Serial.print(";");
      Serial.print(receivedData.accZ, 2); 
      Serial.print(";");
      Serial.print(receivedData.gyrX, 2); 
      Serial.print(";");
      Serial.print(receivedData.gyrY, 2); 
      Serial.print(";");
      Serial.println(receivedData.gyrZ, 2);
      
    } else {
      Serial.print("Error: Paquete de tamaño inesperado (");
      Serial.print(packetSize);
      Serial.println(" bytes recibidos).");
      while (LoRa.available()) {
        LoRa.read(); // Leer y descartar byte por byte
      }
    }
    LoRa.receive(); 
  }
}
