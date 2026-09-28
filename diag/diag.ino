#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  Wire.begin(21, 22);
  Wire.setClock(100000);
  mpu.initialize();

  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission(false);
  int got = Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
  Serial.print("raw Wire got=");
  Serial.print(got);
  if (got) {
    Serial.print(" WHO_AM_I=0x");
    Serial.println(Wire.read(), HEX);
  } else {
    Serial.println();
  }

  uint8_t id = mpu.getDeviceID();
  Serial.print("lib getDeviceID=0x");
  Serial.println(id, HEX);
  Serial.print("lib testConnection=");
  Serial.println(mpu.testConnection() ? "OK" : "FAIL");
}

void loop() {}