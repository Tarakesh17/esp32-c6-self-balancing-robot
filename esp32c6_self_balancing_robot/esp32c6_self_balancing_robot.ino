/*
 * =====================================================================================
 *  ESP32-C6 DevKitC-1  |  Self-Balancing Robot  (DMP approach)
 * =====================================================================================
 *
 *  BOARD      : "ESP32C6 Dev Module"  (Arduino-ESP32 core 3.x)
 *
 *  WIRING
 *  ------
 *    MPU6050            ESP32-C6
 *      VCC      ->      3V3          (NOT 5V - the module's LDO is unreliable on clones)
 *      GND      ->      GND
 *      SDA      ->      GPIO21
 *      SCL      ->      GPIO22
 *      AD0      ->      GND          (I2C address 0x68)
 *      INT      ->      (unused, we poll the FIFO)
 *
 *    L298N              ESP32-C6
 *      IN1      ->      GPIO6
 *      IN2      ->      GPIO7
 *      ENA      ->      GPIO8
 *      IN3      ->      GPIO4
 *      IN4      ->      GPIO5
 *      ENB      ->      GPIO10
 *      +12V     ->      2x18650 in series (~7.4V)
 *      GND      ->      battery GND  *AND*  ESP32 GND  (common ground is mandatory)
 *      +5V      ->      ESP32 5V/VIN pin (on-board 78M05 regulator, keep 5V jumper ON)
 * =====================================================================================
 */

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

// =====================================================================================
//  PIN MAP
// =====================================================================================
#define I2C_SDA        21
#define I2C_SCL        22

#define MOT_A_IN1       6
#define MOT_A_IN2       7
#define MOT_A_EN        8      // PWM - ENA
#define MOT_B_IN3       4
#define MOT_B_IN4       5
#define MOT_B_EN       10      // PWM - ENB

// =====================================================================================
//  TUNABLES
// =====================================================================================

// ---- PID gains (tune in order: Kp, then Kd, then Ki) ----
double Kp = 10.0;
double Ki = 0;
double Kd = 0.5;

// ---- PID output clamp. Also the max PWM duty we ever send (0-255 scale) ----
#define PID_OUTPUT_LIMIT   255.0

// ---- Yaw anti-spin PID (keeps robot pointing straight) ----
double YawKp = 2.5;
double YawKi = 0.0;
double YawKd = 0.6;

// ---- PID sample time. MUST match the DMP rate (default DMP output = 100 Hz = 10 ms).
//      PID_v1 defaults to 100 ms which is far too slow to balance anything. ----
#define PID_SAMPLE_TIME_MS   10

// ---- Which DMP axis is "pitch" for your mounting orientation ----
//      0 = yaw, 1 = pitch, 2 = roll.  If the robot leans forward and the printed
//      angle barely moves, you picked the wrong axis - try 2.
#define TILT_AXIS             1

// ---- Mechanical zero. If the robot creeps forward at rest, nudge this by
//      +/- 0.5 deg until it sits still. ----
#define ANGLE_TRIM           0.5

// ---- Motor deadband. BO motors typically do nothing below ~40/255 duty.
//      Too high causes buzzing / limit-cycle oscillation. ----
#define MOTOR_MIN_PWM         20

// ---- PWM config. L298N is slow; 1 kHz is a safe, quiet-enough compromise. ----
#define PWM_FREQ_HZ        1000
#define PWM_RESOLUTION        8      // 8-bit -> analogWrite range 0..255

// ---- Fall detection ----
#define FALL_OUTPUT_THRESHOLD   (PID_OUTPUT_LIMIT - 5.0)
#define FALL_HOLD_MS          2000
#define FALL_ANGLE_LIMIT        45.0
#define RECOVER_ANGLE            3.0

// ---- Serial telemetry interval (printing every loop would throttle the loop) ----
#define PRINT_INTERVAL_MS      50

// =====================================================================================
//  GLOBALS
// =====================================================================================
MPU6050 mpu;                        // AD0 -> GND, so default address 0x68

bool     dmpReady    = false;
uint8_t  devStatus   = 0;
uint16_t packetSize  = 0;
uint16_t fifoCount   = 0;
uint8_t  fifoBuffer[64];

Quaternion  q;                      // [w, x, y, z]
VectorFloat gravity;                // [x, y, z]
float       ypr[3];                 // [yaw, pitch, roll] in radians

double pidInput    = 0.0;
double pidOutput   = 0.0;
double pidSetpoint = 0.0;

double yawInput    = 0.0;
double yawOutput   = 0.0;
double yawSetpoint = 0.0;

PID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);
PID yawPid(&yawInput, &yawOutput, &yawSetpoint, YawKp, YawKi, YawKd, DIRECT);
#define YAW_OUTPUT_LIMIT   80.0

bool     fallen        = false;
uint32_t saturatedSince = 0;
uint32_t lastPrint      = 0;

// =====================================================================================
//  MOTOR CONTROL
// =====================================================================================
void setupMotors() {
  pinMode(MOT_A_IN1, OUTPUT);
  pinMode(MOT_A_IN2, OUTPUT);
  pinMode(MOT_B_IN3, OUTPUT);
  pinMode(MOT_B_IN4, OUTPUT);
  pinMode(MOT_A_EN,  OUTPUT);
  pinMode(MOT_B_EN,  OUTPUT);

  analogWriteResolution(MOT_A_EN, PWM_RESOLUTION);
  analogWriteResolution(MOT_B_EN, PWM_RESOLUTION);
  analogWriteFrequency(MOT_A_EN, PWM_FREQ_HZ);
  analogWriteFrequency(MOT_B_EN, PWM_FREQ_HZ);

  stopMotors();
}

void stopMotors() {
  analogWrite(MOT_A_EN, 0);
  analogWrite(MOT_B_EN, 0);
  digitalWrite(MOT_A_IN1, LOW);
  digitalWrite(MOT_A_IN2, LOW);
  digitalWrite(MOT_B_IN3, LOW);
  digitalWrite(MOT_B_IN4, LOW);
}

/*
 * driveMotorPair(cmdA, cmdB)
 * --------------------------
 * The two motors run INDEPENDENTLY: cmdA = balance + yaw, cmdB = balance - yaw.
 * Equal parts balance the robot, the differential steers it back to straight.
 *
 * Sign convention:
 *    pidOutput > 0  ->  tilting FORWARD  ->  drive FORWARD  to catch it
 *    pidOutput < 0  ->  tilting BACKWARD ->  drive BACKWARD to catch it
 *
 * If the robot accelerates INTO the fall, swap one motor's wires (OUT3/OUT4),
 * not this code.
 */
void driveMotorPair(double cmdA, double cmdB) {
  int pwmA = (int)fabs(cmdA);
  int pwmB = (int)fabs(cmdB);

  if (pwmA > (int)PID_OUTPUT_LIMIT) pwmA = (int)PID_OUTPUT_LIMIT;
  if (pwmB > (int)PID_OUTPUT_LIMIT) pwmB = (int)PID_OUTPUT_LIMIT;
  if (pwmA > 0 && pwmA < MOTOR_MIN_PWM) pwmA = MOTOR_MIN_PWM;
  if (pwmB > 0 && pwmB < MOTOR_MIN_PWM) pwmB = MOTOR_MIN_PWM;

  if (cmdA > 0)      { digitalWrite(MOT_A_IN1, HIGH); digitalWrite(MOT_A_IN2, LOW); }
  else if (cmdA < 0) { digitalWrite(MOT_A_IN1, LOW);  digitalWrite(MOT_A_IN2, HIGH); }
  else               { digitalWrite(MOT_A_IN1, LOW);  digitalWrite(MOT_A_IN2, LOW); }

  if (cmdB > 0)      { digitalWrite(MOT_B_IN3, HIGH); digitalWrite(MOT_B_IN4, LOW); }
  else if (cmdB < 0) { digitalWrite(MOT_B_IN3, LOW);  digitalWrite(MOT_B_IN4, HIGH); }
  else               { digitalWrite(MOT_B_IN3, LOW);  digitalWrite(MOT_B_IN4, LOW); }

  analogWrite(MOT_A_EN, pwmA);
  analogWrite(MOT_B_EN, pwmB);
}

// =====================================================================================
//  PID RESET (used after a fall, to dump the accumulated integral term)
// =====================================================================================
void resetPid() {
  pid.SetMode(MANUAL);
  pidOutput = 0.0;
  pid.SetMode(AUTOMATIC);
  yawPid.SetMode(MANUAL);
  yawOutput = 0.0;
  yawPid.SetMode(AUTOMATIC);
}

// =====================================================================================
//  SETUP
// =====================================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* wait briefly for USB CDC */ }

  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F(" ESP32-C6 Self-Balancing Robot (DMP)"));
  Serial.println(F("================================================"));

  setupMotors();

  // ---------------- I2C ----------------
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);                 // 100 kHz - reliable on clones + long wires

  // ---------------- MPU6050 ----------------
  Serial.println(F("[MPU] Initializing device..."));
  mpu.initialize();

  Serial.print(F("[MPU] Connection test: "));
  Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 FAILED"));

  // ---------------- DMP (with retry) ----------------
  Serial.println(F("[DMP] Initializing DMP..."));
  for (int attempt = 1; attempt <= 3; attempt++) {
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) break;
    Serial.print(F("[DMP] attempt "));
    Serial.print(attempt);
    Serial.print(F(" failed (code "));
    Serial.print(devStatus);
    Serial.println(F(") - retrying"));
    delay(300);
  }

  // These six numbers come from the calibration sketch (imu_cal.ino).
  // Re-run calibration if you re-mount the sensor or swap the module.
  mpu.setXAccelOffset(-5758);
  mpu.setYAccelOffset( 5042);
  mpu.setZAccelOffset(10170);
  mpu.setXGyroOffset (   11);
  mpu.setYGyroOffset (   55);
  mpu.setZGyroOffset (   -4);

  if (devStatus == 0) {
    Serial.println(F("[DMP] Enabling DMP..."));
    mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("[DMP] FIFO packet size: "));
    Serial.println(packetSize);

    mpu.resetFIFO();
    dmpReady = true;
    Serial.println(F("[DMP] Ready."));
  } else {
    Serial.print(F("[DMP] INIT FAILED (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    Serial.println(F("      1 = memory load failed"));
    Serial.println(F("      2 = DMP config updates failed"));
    Serial.println(F("      Motors will stay disabled."));
    stopMotors();
  }

  // ---------------- PID ----------------
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(PID_SAMPLE_TIME_MS);
  pid.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);

  yawPid.SetMode(AUTOMATIC);
  yawPid.SetSampleTime(PID_SAMPLE_TIME_MS);
  yawPid.SetOutputLimits(-YAW_OUTPUT_LIMIT, YAW_OUTPUT_LIMIT);

  Serial.println(F("[PID] Kp=8.0  Ki=10.0  Kd=1.8  limits=+/-200  DIRECT"));
  Serial.println(F("Format:  angle,<deg>  out,<pwm>  yaw,<deg>  state,<BAL|FALL>"));
  Serial.println(F("------------------------------------------------"));
}

// =====================================================================================
//  MAIN LOOP  -  free-running, no blocking delays
// =====================================================================================
void loop() {
  if (!dmpReady) {
    stopMotors();
    return;
  }

  fifoCount = mpu.getFIFOCount();

  // ---- FIFO overflow: 1024-byte buffer is full, data is garbage. Flush it. ----
  if (fifoCount >= 1024) {
    mpu.resetFIFO();
    Serial.println(F("[FIFO] overflow - reset"));
    return;
  }

  // ---- Misaligned FIFO (partial packet from a glitched read). Flush it. ----
  if (packetSize > 0 && (fifoCount % packetSize) != 0) {
    mpu.resetFIFO();
    return;
  }

  // ---- Nothing new yet: bail out immediately and come straight back. ----
  if (fifoCount < packetSize) {
    return;
  }

  // ---- Drain to the NEWEST packet so we always act on the freshest angle ----
  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  // ---------------- Compute tilt angle ----------------
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  double angle = (ypr[TILT_AXIS] * 180.0 / M_PI) + ANGLE_TRIM;
  pidInput = angle;

  // ---------------- PID ----------------
  pid.Compute();

  // Yaw PID: ypr[0] = DMP yaw in radians -> degrees
  yawInput = ypr[0] * 180.0 / M_PI;
  yawPid.Compute();

  // ---------------- Fall detection ----------------
  uint32_t now = millis();

  if (fabs(angle) > FALL_ANGLE_LIMIT) {
    fallen = true;
  }

  if (fabs(pidOutput) >= FALL_OUTPUT_THRESHOLD) {
    if (saturatedSince == 0) {
      saturatedSince = now;
    } else if ((now - saturatedSince) > FALL_HOLD_MS) {
      fallen = true;
    }
  } else {
    saturatedSince = 0;
  }

  // ---------------- Drive / protect ----------------
  if (fallen) {
    stopMotors();

    if (fabs(angle) < RECOVER_ANGLE) {
      fallen         = false;
      saturatedSince = 0;
      resetPid();
      mpu.resetFIFO();
      Serial.println(F("[FALL] recovered - PID reset, re-armed"));
    }
  } else {
    // balance + yaw differential on the two motors
    driveMotorPair(pidOutput + yawOutput, pidOutput - yawOutput);
  }

  // ---------------- Telemetry (throttled, non-blocking) ----------------
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    Serial.print(F("angle,"));
    Serial.print(angle, 2);
    Serial.print(F("  out,"));
    Serial.print(pidOutput, 1);
    Serial.print(F("  yaw,"));
    Serial.print(yawOutput, 1);
    Serial.print(F("  state,"));
    Serial.println(fallen ? F("FALL") : F("BAL"));
  }
}

/*
 * =====================================================================================
 *  TUNING PROCEDURE
 * =====================================================================================
 *  0. Hold the robot upright. Confirm angle ~0 when vertical and POSITIVE when
 *     tilted forward. If negative, swap one motor's wires on OUT3/OUT4.
 *
 *  1. Set Ki = 0, Kd = 0. Raise Kp until the robot fights back hard and
 *     oscillates around vertical with roughly equal-sized swings.
 *
 *  2. Add Kd (start 0.5, step 0.2) until oscillation damps out. Too much Kd =
 *     high-frequency buzzing/jitter.
 *
 *  3. Add Ki last (start 5, step 2). Ki removes slow drift/creep. Too much Ki =
 *     slow lumbering oscillation that grows over several seconds.
 *
 *  4. Adjust ANGLE_TRIM in 0.2 deg steps until the robot stops drifting.
 *
 *  Change ONE gain at a time. Re-flash between changes and give it 20 seconds.
 * =====================================================================================
 */