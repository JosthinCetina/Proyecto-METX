// Código para una BNO055 con Fs = 200 Hz, más lento!!

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BNO055_BIN");
  Wire.begin(21, 22, 400000); // 400 kHz (límite estable del BNO055)
  
  if (!bno.begin()) {
    Serial.println("No se detecta BNO055");
    while (1);
  }

  bno.setMode(OPERATION_MODE_ACCGYRO);

  delay(100);
  Serial.println("BNO055 listo - modo ACC+GYRO");
  SerialBT.println("BNO055 listo - modo ACC+GYRO");
}

void loop() {
  sensors_event_t accelEvent, gyroEvent;
  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  if (now - lastMicros >= 5000) {
    lastMicros = now;

    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

    int16_t ax = (int16_t)(accelEvent.acceleration.x * 100); // m/s² *100
    int16_t ay = (int16_t)(accelEvent.acceleration.y * 100);
    int16_t az = (int16_t)(accelEvent.acceleration.z * 100);
    int16_t gx = (int16_t)(gyroEvent.gyro.x * 100); // rad/s *100
    int16_t gy = (int16_t)(gyroEvent.gyro.y * 100);
    int16_t gz = (int16_t)(gyroEvent.gyro.z * 100);

    uint8_t packet[12];
    memcpy(packet, &ax, 2);
    memcpy(packet + 2, &ay, 2);
    memcpy(packet + 4, &az, 2);
    memcpy(packet + 6, &gx, 2);
    memcpy(packet + 8, &gy, 2);
    memcpy(packet + 10, &gz, 2);

    // Enviar por Bluetooth
    SerialBT.write(packet, 12);
  }
}
