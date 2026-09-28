#include <Wire.h>
#include <math.h>

// -------------------- I2C Pins --------------------
#define SDA_PIN 21      // Change if required
#define SCL_PIN 22      // Change if required

#define MPU_ADDR 0x68

// -------------------- Sensor Variables --------------------
int16_t ax, ay, az;
int16_t gx, gy, gz;

float gyroXoffset = 0;
float gyroYoffset = 0;
float gyroZoffset = 0;

float pitch = 0;
float roll = 0;

unsigned long previousTime = 0;

// -----------------------------------------------------------
void setup() {

  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  delay(1000);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("==================================");
  Serial.println("MPU6050 Calibration");
  Serial.println("Keep sensor PERFECTLY STILL...");
  Serial.println("==================================");

  delay(2000);

  calibrateGyro();

  previousTime = micros();
}

// -----------------------------------------------------------
void loop() {

  readMPU();

  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  // Remove gyro offsets
  float gx_dps = (gx - gyroXoffset) / 131.0;
  float gy_dps = (gy - gyroYoffset) / 131.0;
  float gz_dps = (gz - gyroZoffset) / 131.0;

  // Accelerometer angle
  float accPitch = atan2(ay, sqrt((ax * ax) + (az * az))) * 180.0 / PI;
  float accRoll  = atan2(-ax, az) * 180.0 / PI;

  // Complementary Filter
  pitch = 0.98 * (pitch + gx_dps * dt) + 0.02 * accPitch;
  roll  = 0.98 * (roll  + gy_dps * dt) + 0.02 * accRoll;

  Serial.print("Pitch: ");
  Serial.print(pitch);

  Serial.print("   Roll: ");
  Serial.print(roll);

  Serial.print("   GX:");
  Serial.print(gx_dps);

  Serial.print("   GY:");
  Serial.print(gy_dps);

  Serial.print("   GZ:");
  Serial.println(gz_dps);

  delay(10);
}

// -----------------------------------------------------------
void calibrateGyro() {

  long gxSum = 0;
  long gySum = 0;
  long gzSum = 0;

  const int samples = 1000;

  for (int i = 0; i < samples; i++) {

    readMPU();

    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(3);
  }

  gyroXoffset = gxSum / (float)samples;
  gyroYoffset = gySum / (float)samples;
  gyroZoffset = gzSum / (float)samples;

  Serial.println();
  Serial.println("Calibration Finished");

  Serial.print("Gyro X Offset = ");
  Serial.println(gyroXoffset);

  Serial.print("Gyro Y Offset = ");
  Serial.println(gyroYoffset);

  Serial.print("Gyro Z Offset = ");
  Serial.println(gyroZoffset);

  Serial.println();
}

// -----------------------------------------------------------
void readMPU() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();      // Skip temperature

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}