#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  Wire.begin(21, 22);
  Wire.setClock(100000);

  mpu.initialize();
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  Wire.begin(21, 22);
  Wire.setClock(100000);

  Serial.println(F("WHO_AM_I mismatch ignored (clone module) - calibrating..."));
  Serial.println(F("Place the sensor (or robot) FLAT and COMPLETELY STILL."));
  Serial.println(F("Calibration takes ~15-20s."));

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  Serial.println();
  Serial.println(F("--- COPY THESE INTO esp32c6_self_balancing_robot.ino (offset lines) ---"));
  Serial.print(F("mpu.setXAccelOffset(")); Serial.print(mpu.getXAccelOffset()); Serial.println(F(");"));
  Serial.print(F("mpu.setYAccelOffset(")); Serial.print(mpu.getYAccelOffset()); Serial.println(F(");"));
  Serial.print(F("mpu.setZAccelOffset(")); Serial.print(mpu.getZAccelOffset()); Serial.println(F(");"));
  Serial.print(F("mpu.setXGyroOffset (")); Serial.print(mpu.getXGyroOffset()); Serial.println(F(");"));
  Serial.print(F("mpu.setYGyroOffset (")); Serial.print(mpu.getYGyroOffset()); Serial.println(F(");"));
  Serial.print(F("mpu.setZGyroOffset (")); Serial.print(mpu.getZGyroOffset()); Serial.println(F(");"));
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(F("DONE"));
}

void loop() {}