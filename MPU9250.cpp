// Código en C para MPU9250
// Envio de datos por Bluetooth
// Fs = 3 kHz - Ts = 333us
// Paquetes de datos de 24 bytes (16 bits), 2 bytes (16 bits) por eje, para 74 kB/s (592 kb/s).

#include <Wire.h>
#include <MPU9250.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
MPU9250 mpu;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_MPU9250_BIN");
  Wire.begin(21, 22, 1000000); // I2C a 1 MHz

  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 no detectado");
    while (1);
  }

  mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_3600HZ);
  mpu.setSrd(0); // Sin división → 8kHz (luego limitamos a 3kHz)
  Serial.println("Listo a 3kHz binario");
}

void loop() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  if (now - lastMicros >= 333) { // cada 333µs ≈ 3kHz
    lastMicros = now;
    mpu.readSensor();

    int16_t ax = mpu.getAccRawX();
    int16_t ay = mpu.getAccRawY();
    int16_t az = mpu.getAccRawZ();
    int16_t gx = mpu.getGyroRawX();
    int16_t gy = mpu.getGyroRawY();
    int16_t gz = mpu.getGyroRawZ();

    uint8_t packet[12];
    memcpy(packet, &ax, 2);
    memcpy(packet + 2, &ay, 2);
    memcpy(packet + 4, &az, 2);
    memcpy(packet + 6, &gx, 2);
    memcpy(packet + 8, &gy, 2);
    memcpy(packet + 10, &gz, 2);

    SerialBT.write(packet, 12); // Enviar binario crudo (12 bytes/lectura)
  }
}
