#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ==== Pines LoRa ====
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_CS     5
#define LORA_RST   14
#define LORA_DIO0   2
#define LORA_BAND  433.5E6  // Frecuencia: 433.5 MHz

Adafruit_MPU6050 mpu;
bool lora_ok = false;

// ==== Calibración ====
float accX_offset = 0, accY_offset = 0, accZ_offset = 0;

// --------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n========== INICIANDO EMISOR LoRa ==========\n");

  // ==== Inicializar MPU6050 ====
  Serial.println("Iniciando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("No se detectó el MPU6050.");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("\nCalibrando MPU6050... No muevas la placa!");
  delay(100);
  calibrarMPU();
  Serial.println("Calibración completada!\n");

  // ==== Inicializar LoRa ====
  Serial.println("Iniciando LoRa...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("ERROR: No se detectó LoRa.");
    while (true) {
      delay(2000);
      Serial.println("Esperando LoRa...");
    }
  }

  lora_ok = true;
  Serial.println("LoRa inicializado correctamente!");

  // ==== APLICAR CONFIGURACIÓN SOLICITADA ====
  LoRa.setSpreadingFactor(9);         // SF9
  LoRa.setSignalBandwidth(125E3);     // BW = 125 kHz
  LoRa.setCodingRate4(6);             // CR = 4/6
  LoRa.setTxPower(20);                // Potencia = 20 dBm

  // LoRa.enableCrc();                // CRC opcional

  Serial.println("\n--- CONFIGURACIÓN LoRa ---");
  Serial.println("Frecuencia: 433.5 MHz");
  Serial.println("Potencia: 20 dBm");
  Serial.println("SF: 9");
  Serial.println("BW: 125 kHz");
  Serial.println("CR: 4/6");
  Serial.println("----------------------------------\n");
}

// --------------------------------------------------

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Aplicar correcciones
  float accX = a.acceleration.x - accX_offset;
  float accY = a.acceleration.y - accY_offset;
  float accZ = a.acceleration.z - accZ_offset;

  float gyroX = g.gyro.x;
  float gyroY = g.gyro.y;
  float gyroZ = g.gyro.z;

  // Crear paquete
  String paquete = "";
  paquete += String(accX, 2) + ";";
  paquete += String(accY, 2) + ";";
  paquete += String(accZ, 2) + ";";
  paquete += String(gyroX, 2) + ";";
  paquete += String(gyroY, 2) + ";";
  paquete += String(gyroZ, 2);

  Serial.println(paquete);

  // Enviar LoRa
  if (lora_ok) {
    LoRa.beginPacket();
    LoRa.print(paquete);
    LoRa.endPacket();
  }

  delay(2);  // ~500 Hz → cada 2 ms
}

// --------------------------------------------------

void calibrarMPU() {
  const int muestras = 200;
  float accX_sum = 0, accY_sum = 0, accZ_sum = 0;

  for (int i = 0; i < muestras; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accX_sum += a.acceleration.x;
    accY_sum += a.acceleration.y;
    accZ_sum += a.acceleration.z;
    delay(10);
  }

  accX_offset = accX_sum / muestras;
  accY_offset = accY_sum / muestras;
  accZ_offset = accZ_sum / muestras;
}
