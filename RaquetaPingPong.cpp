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
unsigned long previousMicros = 0;
const long interval = 20000; // 20ms para 50Hz (1/50 = 0.02s)

// ==== Calibración ====
float accX_offset = 0, accY_offset = 0, accZ_offset = 0;
//float gyrX_offset = 0, gyrY_offset = 0, gyrZ_offset = 0;

// --------------------------------------------------

struct SensorData {
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n========== INICIANDO EMISOR LoRa ==========\n");
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
  LoRa.setSpreadingFactor(7);         // SF7
  LoRa.setSignalBandwidth(125E3);     // BW = 125 kHz
  LoRa.setCodingRate4(6);             // CR = 4/6
  LoRa.setTxPower(20);                // Potencia = 20 dBm
}

void loop() {
  unsigned long currentMicros = micros();

  // ********** Bucle de Temporización de 50Hz **********
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros; // Guarda el último tiempo de muestreo/transmisión

    // ** LECTURA DEL SENSOR **
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
    SensorData data;
    data.accX = a.acceleration.x - accX_offset;
    data.accY = a.acceleration.y - accY_offset;
    data.accZ = a.acceleration.z - accZ_offset;
    data.gyrX = g.gyro.x;
    data.gyrY = g.gyro.y;
    data.gyrZ = g.gyro.z;
    // ** ENVIAR LoRa **
    if (lora_ok) {
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&data, sizeof(data));
      LoRa.endPacket();
    }
  }
}

// --------------------------------------------------

void calibrarMPU() {
  const int muestras = 200;
  float accX_sum = 0, accY_sum = 0, accZ_sum = 0;
  //float gyrX_sum = 0, gyrY_sum = 0, gyrZ_sum = 0;

  for (int i = 0; i < muestras; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accX_sum += a.acceleration.x;
    accY_sum += a.acceleration.y;
    accZ_sum += a.acceleration.z;
    //gyrX_sum += a.gyro.x; 
    //gyrY_sum += a.gyro.y;
    //gyrZ_sum += a.gyro.z;
    delay(10);
  }

  accX_offset = accX_sum / muestras;
  accY_offset = accY_sum / muestras;
  accZ_offset = accZ_sum / muestras;
  //gyrX_offset = gyrX_sum / muestras;
  //gyrY_offset = gyrY_sum / muestras;
  //gyrZ_offset = gyrZ_sum / muestras;
}
