/*
 * =====================================================================================
 *  ESP32-C6 DevKitC-1  |  Self-Balancing Robot (DMP + Yaw hold)  |  WiFi AP Tuning
 * =====================================================================================
 *
 *  ACCESS POINT
 *      SSID : BalanceBot
 *      PASS : balance123
 *      URL  : http://192.168.4.1
 *
 *  Connect from a phone, drag the sliders, watch it settle. Gains apply instantly
 *  and persist to NVS when you hit Save, so a tuned robot survives a power cycle.
 *  Your phone will warn the network has no internet — that is expected, and is why
 *  the page has zero external assets.
 *
 *  ------------------------------- SINGLE CORE WARNING -----------------------------
 *  The ESP32-C6 has ONE usable application core. The web server shares it with the
 *  balance loop. Network is only serviced in the idle gap between DMP packets, and
 *  loopHz is reported so you can see the cost:
 *      ~100  healthy      <70  page is stealing time      <50  stop tuning over WiFi
 *  Tune over WiFi, then set ENABLE_WIFI 0 and reflash for real runs.
 *  ---------------------------------------------------------------------------------
 *
 *  WIRING (unchanged)
 *    MPU6050  VCC->3V3  GND->GND  SDA->GPIO21  SCL->GPIO22  AD0->GND (0x68)
 *    L298N    IN1->6  IN2->7  ENA->8   IN3->4  IN4->5  ENB->10
 *             +12V-> 2x18650 (~7.4V)   +5V-> ESP32 5V/VIN (keep 5V jumper ON)
 *             GND-> battery GND *AND* ESP32 GND (common ground is mandatory)
 * =====================================================================================
 */

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <Preferences.h>

#define ENABLE_WIFI 1          // 0 = radio-silent build for real runs

#if ENABLE_WIFI
  #include <WiFi.h>
  #include <WebServer.h>
#endif

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
//  NETWORK
// =====================================================================================
#define AP_SSID   "BalanceBot"
#define AP_PASS   "balance123"       // min 8 chars or the AP quietly comes up open

// =====================================================================================
//  DEFAULTS  (NVS values loaded at boot override these)
// =====================================================================================
#define DEF_KP      105.0
#define DEF_KI        0.0
#define DEF_KD        0.5

#define DEF_YKP       2.5
#define DEF_YKI       0.0
#define DEF_YKD       0.6

#define DEF_TRIM      0.5
#define DEF_LIMIT   255.0
#define DEF_YLIMIT   80.0
#define DEF_MINPWM     20
#define DEF_AXIS        1       // 0=yaw 1=pitch 2=roll
#define DEF_YAWON       1       // yaw hold enabled

#define PID_SAMPLE_TIME_MS    10
#define PWM_FREQ_HZ         1000
#define PWM_RESOLUTION         8

#define FALL_HOLD_MS        2000
#define FALL_ANGLE_LIMIT      45.0
#define RECOVER_ANGLE          3.0
#define FALL_MARGIN            5.0    // fall threshold = limit - margin

#define NET_SERVICE_MIN_US   800      // idle time needed before entering the server

// =====================================================================================
//  LIVE TUNABLES  (all writable over the API)
// =====================================================================================
double cfgKp   = DEF_KP,   cfgKi   = DEF_KI,   cfgKd   = DEF_KD;
double cfgYKp  = DEF_YKP,  cfgYKi  = DEF_YKI,  cfgYKd  = DEF_YKD;
double cfgTrim = DEF_TRIM, cfgLimit = DEF_LIMIT, cfgYLimit = DEF_YLIMIT;
int    cfgMinPwm = DEF_MINPWM;
int    cfgAxis   = DEF_AXIS;
bool   cfgYawOn  = DEF_YAWON;

// Offsets from your imu_cal.ino run. Re-run calibration if you re-mount the sensor.
int16_t offAX = -5758, offAY =  5042, offAZ = 10170;
int16_t offGX =    11, offGY =    55, offGZ =    -4;

bool armed  = true;      // master motor enable
bool fallen = true;      // start latched: arms itself once you hold it upright

// =====================================================================================
//  RUNTIME
// =====================================================================================
MPU6050 mpu;

bool     dmpReady   = false;
uint8_t  devStatus  = 0;
uint16_t packetSize = 0;
uint16_t fifoCount  = 0;
uint8_t  fifoBuffer[64];

Quaternion  q;
VectorFloat gravity;
float       ypr[3];

double pidInput = 0.0, pidOutput = 0.0, pidSetpoint = 0.0;
double yawInput = 0.0, yawOutput = 0.0, yawSetpoint = 0.0;

double rawYaw    = 0.0;   // absolute DMP yaw, degrees (drifts, no magnetometer)
double yawRef    = 0.0;   // heading captured at arm time
double yawErr    = 0.0;   // wrapped deviation from yawRef

PID pid(&pidInput, &pidOutput, &pidSetpoint, DEF_KP, DEF_KI, DEF_KD, DIRECT);
PID yawPid(&yawInput, &yawOutput, &yawSetpoint, DEF_YKP, DEF_YKI, DEF_YKD, DIRECT);

uint32_t saturatedSince = 0;
uint32_t lastPacketUs   = 0;
uint32_t loopCounter    = 0;
uint32_t loopWindowMs   = 0;
uint32_t fallCount      = 0;
float    loopHz         = 0.0;

Preferences prefs;

#if ENABLE_WIFI
WebServer server(80);
bool netUp = false;
#endif

// =====================================================================================
//  MOTOR CONTROL
// =====================================================================================
void stopMotors() {
  analogWrite(MOT_A_EN, 0);
  analogWrite(MOT_B_EN, 0);
  digitalWrite(MOT_A_IN1, LOW); digitalWrite(MOT_A_IN2, LOW);
  digitalWrite(MOT_B_IN3, LOW); digitalWrite(MOT_B_IN4, LOW);
}

void setupMotors() {
  pinMode(MOT_A_IN1, OUTPUT); pinMode(MOT_A_IN2, OUTPUT);
  pinMode(MOT_B_IN3, OUTPUT); pinMode(MOT_B_IN4, OUTPUT);
  pinMode(MOT_A_EN,  OUTPUT); pinMode(MOT_B_EN,  OUTPUT);

  analogWriteResolution(MOT_A_EN, PWM_RESOLUTION);
  analogWriteResolution(MOT_B_EN, PWM_RESOLUTION);
  analogWriteFrequency(MOT_A_EN, PWM_FREQ_HZ);
  analogWriteFrequency(MOT_B_EN, PWM_FREQ_HZ);

  stopMotors();
}

/*
 * driveMotorPair(cmdA, cmdB)
 * --------------------------
 * cmdA = balance + yaw, cmdB = balance - yaw. The equal part balances, the
 * differential steers back to the held heading.
 *
 *    pidOutput > 0  ->  tilting FORWARD  ->  drive FORWARD  to catch it
 *    pidOutput < 0  ->  tilting BACKWARD ->  drive BACKWARD to catch it
 *
 * If the robot accelerates INTO the fall, swap one motor's wires on OUT3/OUT4.
 * If yaw correction pushes it further off heading, negate yawOutput at the call.
 */
void driveMotorPair(double cmdA, double cmdB) {
  int pwmA = (int)fabs(cmdA);
  int pwmB = (int)fabs(cmdB);

  if (pwmA > (int)cfgLimit) pwmA = (int)cfgLimit;
  if (pwmB > (int)cfgLimit) pwmB = (int)cfgLimit;
  if (pwmA > 0 && pwmA < cfgMinPwm) pwmA = cfgMinPwm;
  if (pwmB > 0 && pwmB < cfgMinPwm) pwmB = cfgMinPwm;

  if (cmdA > 0)      { digitalWrite(MOT_A_IN1, HIGH); digitalWrite(MOT_A_IN2, LOW);  }
  else if (cmdA < 0) { digitalWrite(MOT_A_IN1, LOW);  digitalWrite(MOT_A_IN2, HIGH); }
  else               { digitalWrite(MOT_A_IN1, LOW);  digitalWrite(MOT_A_IN2, LOW);  }

  if (cmdB > 0)      { digitalWrite(MOT_B_IN3, HIGH); digitalWrite(MOT_B_IN4, LOW);  }
  else if (cmdB < 0) { digitalWrite(MOT_B_IN3, LOW);  digitalWrite(MOT_B_IN4, HIGH); }
  else               { digitalWrite(MOT_B_IN3, LOW);  digitalWrite(MOT_B_IN4, LOW);  }

  analogWrite(MOT_A_EN, pwmA);
  analogWrite(MOT_B_EN, pwmB);
}

// =====================================================================================
//  PID HELPERS
// =====================================================================================
void applyTunings() {
  pid.SetTunings(cfgKp, cfgKi, cfgKd);
  pid.SetOutputLimits(-cfgLimit, cfgLimit);
  pid.SetSampleTime(PID_SAMPLE_TIME_MS);

  yawPid.SetTunings(cfgYKp, cfgYKi, cfgYKd);
  yawPid.SetOutputLimits(-cfgYLimit, cfgYLimit);
  yawPid.SetSampleTime(PID_SAMPLE_TIME_MS);
}

// Wrap to (-180, 180] so crossing the yaw discontinuity doesn't command a full-scale
// spin. Without this the robot slams the differential hard every time it passes 180.
double wrap180(double a) {
  while (a >  180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// PID_v1 re-seeds its integral from *myOutput on the return to AUTOMATIC, so zeroing
// the outputs here is what actually clears the accumulated integral terms.
void resetPid() {
  pid.SetMode(MANUAL);    pidOutput = 0.0; pid.SetMode(AUTOMATIC);
  yawPid.SetMode(MANUAL); yawOutput = 0.0; yawPid.SetMode(AUTOMATIC);
  yawRef = rawYaw;        // re-capture heading; the old one has drifted
}

void applyOffsets() {
  mpu.setXAccelOffset(offAX); mpu.setYAccelOffset(offAY); mpu.setZAccelOffset(offAZ);
  mpu.setXGyroOffset(offGX);  mpu.setYGyroOffset(offGY);  mpu.setZGyroOffset(offGZ);
}

// =====================================================================================
//  PERSISTENCE (NVS)
// =====================================================================================
void saveConfig() {
  prefs.begin("bot", false);
  prefs.putDouble("kp", cfgKp);   prefs.putDouble("ki", cfgKi);   prefs.putDouble("kd", cfgKd);
  prefs.putDouble("ykp", cfgYKp); prefs.putDouble("yki", cfgYKi); prefs.putDouble("ykd", cfgYKd);
  prefs.putDouble("trim", cfgTrim);
  prefs.putDouble("limit", cfgLimit);
  prefs.putDouble("ylimit", cfgYLimit);
  prefs.putInt("minpwm", cfgMinPwm);
  prefs.putInt("axis", cfgAxis);
  prefs.putBool("yawon", cfgYawOn);
  prefs.putShort("oax", offAX); prefs.putShort("oay", offAY); prefs.putShort("oaz", offAZ);
  prefs.putShort("ogx", offGX); prefs.putShort("ogy", offGY); prefs.putShort("ogz", offGZ);
  prefs.end();
}

void loadConfig() {
  prefs.begin("bot", true);
  cfgKp    = prefs.getDouble("kp",   DEF_KP);
  cfgKi    = prefs.getDouble("ki",   DEF_KI);
  cfgKd    = prefs.getDouble("kd",   DEF_KD);
  cfgYKp   = prefs.getDouble("ykp",  DEF_YKP);
  cfgYKi   = prefs.getDouble("yki",  DEF_YKI);
  cfgYKd   = prefs.getDouble("ykd",  DEF_YKD);
  cfgTrim  = prefs.getDouble("trim", DEF_TRIM);
  cfgLimit = prefs.getDouble("limit",  DEF_LIMIT);
  cfgYLimit= prefs.getDouble("ylimit", DEF_YLIMIT);
  cfgMinPwm= prefs.getInt("minpwm", DEF_MINPWM);
  cfgAxis  = prefs.getInt("axis",   DEF_AXIS);
  cfgYawOn = prefs.getBool("yawon", DEF_YAWON);
  offAX = prefs.getShort("oax", offAX); offAY = prefs.getShort("oay", offAY);
  offAZ = prefs.getShort("oaz", offAZ); offGX = prefs.getShort("ogx", offGX);
  offGY = prefs.getShort("ogy", offGY); offGZ = prefs.getShort("ogz", offGZ);
  prefs.end();
}

void clearConfig() {
  prefs.begin("bot", false); prefs.clear(); prefs.end();
  cfgKp = DEF_KP; cfgKi = DEF_KI; cfgKd = DEF_KD;
  cfgYKp = DEF_YKP; cfgYKi = DEF_YKI; cfgYKd = DEF_YKD;
  cfgTrim = DEF_TRIM; cfgLimit = DEF_LIMIT; cfgYLimit = DEF_YLIMIT;
  cfgMinPwm = DEF_MINPWM; cfgAxis = DEF_AXIS; cfgYawOn = DEF_YAWON;
  applyTunings();
}

// =====================================================================================
//  HTTP API
// =====================================================================================
#if ENABLE_WIFI

String stateJson() {
  char buf[520];
  snprintf(buf, sizeof(buf),
    "{\"angle\":%.2f,\"output\":%.1f,\"yaw\":%.1f,\"yawOut\":%.1f,"
    "\"armed\":%s,\"fallen\":%s,\"dmp\":%s,"
    "\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.3f,"
    "\"ykp\":%.2f,\"yki\":%.2f,\"ykd\":%.3f,\"yawon\":%s,"
    "\"trim\":%.2f,\"limit\":%.0f,\"ylimit\":%.0f,\"minpwm\":%d,\"axis\":%d,"
    "\"loopHz\":%.0f,\"falls\":%lu,\"uptime\":%lu,"
    "\"offsets\":{\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d}}",
    pidInput, pidOutput, yawErr, yawOutput,
    armed ? "true" : "false", fallen ? "true" : "false", dmpReady ? "true" : "false",
    cfgKp, cfgKi, cfgKd, cfgYKp, cfgYKi, cfgYKd, cfgYawOn ? "true" : "false",
    cfgTrim, cfgLimit, cfgYLimit, cfgMinPwm, cfgAxis,
    loopHz, (unsigned long)fallCount, (unsigned long)(millis() / 1000),
    offAX, offAY, offAZ, offGX, offGY, offGZ);
  return String(buf);
}

void sendState() { server.send(200, "application/json", stateJson()); }

double argD(const char *k, double cur) { return server.hasArg(k) ? server.arg(k).toDouble() : cur; }
int    argI(const char *k, int cur)    { return server.hasArg(k) ? server.arg(k).toInt()    : cur; }

void handlePid() {
  cfgKp = argD("kp", cfgKp);   cfgKi = argD("ki", cfgKi);   cfgKd = argD("kd", cfgKd);
  cfgYKp = argD("ykp", cfgYKp); cfgYKi = argD("yki", cfgYKi); cfgYKd = argD("ykd", cfgYKd);
  if (cfgKp < 0) cfgKp = 0;   if (cfgKi < 0) cfgKi = 0;   if (cfgKd < 0) cfgKd = 0;
  if (cfgYKp < 0) cfgYKp = 0; if (cfgYKi < 0) cfgYKi = 0; if (cfgYKd < 0) cfgYKd = 0;
  applyTunings();
  sendState();
}

void handleConfig() {
  cfgTrim   = argD("trim",   cfgTrim);
  cfgLimit  = argD("limit",  cfgLimit);
  cfgYLimit = argD("ylimit", cfgYLimit);
  cfgMinPwm = argI("minpwm", cfgMinPwm);
  cfgAxis   = argI("axis",   cfgAxis);
  if (server.hasArg("yawon")) {
    cfgYawOn = (server.arg("yawon") == "1" || server.arg("yawon") == "true");
    if (cfgYawOn) { yawRef = rawYaw; yawPid.SetMode(MANUAL); yawOutput = 0; yawPid.SetMode(AUTOMATIC); }
    else yawOutput = 0;
  }
  if (cfgLimit  < 10)  cfgLimit  = 10;    if (cfgLimit  > 255) cfgLimit  = 255;
  if (cfgYLimit < 0)   cfgYLimit = 0;     if (cfgYLimit > 200) cfgYLimit = 200;
  if (cfgMinPwm < 0)   cfgMinPwm = 0;     if (cfgMinPwm > 120) cfgMinPwm = 120;
  if (cfgAxis < 0 || cfgAxis > 2) cfgAxis = DEF_AXIS;
  applyTunings();
  sendState();
}

void handleOffsets() {
  offAX = argI("ax", offAX); offAY = argI("ay", offAY); offAZ = argI("az", offAZ);
  offGX = argI("gx", offGX); offGY = argI("gy", offGY); offGZ = argI("gz", offGZ);
  applyOffsets();
  mpu.resetFIFO();
  sendState();
}

void handleArm()      { armed = true;  resetPid(); sendState(); }
void handleDisarm()   { armed = false; stopMotors(); sendState(); }
void handleReset()    { resetPid(); mpu.resetFIFO(); saturatedSince = 0; sendState(); }
void handleZeroYaw()  { yawRef = rawYaw; sendState(); }
void handleSave()     { saveConfig(); server.send(200, "application/json", "{\"ok\":true}"); }
void handleLoad()     { loadConfig(); applyTunings(); applyOffsets(); sendState(); }
void handleDefaults() { clearConfig(); sendState(); }

void handleReboot() {
  stopMotors();
  server.send(200, "application/json", "{\"ok\":true,\"action\":\"rebooting\"}");
  delay(120);                 // only blocking delay in the firmware, and we are leaving
  ESP.restart();
}

void handleNotFound() {
  server.send(404, "application/json", "{\"error\":\"no such endpoint\",\"try\":\"/api/state\"}");
}

// ---------------------------------------------------------------------------------
//  Served page — attitude horizon over a rolling angle/output trace. No external
//  assets: the AP has no internet, so nothing can be fetched from a CDN.
// ---------------------------------------------------------------------------------
const char PAGE_INDEX[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>BalanceBot</title>
<style>
:root{
  --ground:#0E1621;--panel:#16212E;--rule:#26374A;--ink:#C8D6E5;--dim:#6B8299;
  --angle:#35D0D8;--output:#F2A73B;--yaw:#9B8CFF;--alert:#FF5C5C;--ok:#5BD98A;
  --mono:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,monospace;
  --sans:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
}
*{box-sizing:border-box}
body{margin:0;background:var(--ground);color:var(--ink);font-family:var(--sans);
  -webkit-text-size-adjust:100%}
.wrap{max-width:560px;margin:0 auto;padding:16px 14px 40px}
h1{font-size:13px;font-weight:600;letter-spacing:.18em;text-transform:uppercase;margin:0 0 2px}
.sub{font-family:var(--mono);font-size:11px;color:var(--dim);margin-bottom:16px}
.panel{background:var(--panel);border:1px solid var(--rule);border-radius:3px;
  padding:14px;margin-bottom:12px}
.lbl{font-family:var(--mono);font-size:10px;letter-spacing:.14em;text-transform:uppercase;
  color:var(--dim);display:flex;justify-content:space-between;margin-bottom:8px}
canvas{display:block;width:100%;background:var(--ground);border:1px solid var(--rule);
  border-radius:2px}
#horizon{height:132px;margin-bottom:10px}
#trace{height:120px}
.readout{display:flex;gap:12px;font-family:var(--mono);margin-top:10px}
.readout div{flex:1}
.readout b{display:block;font-size:21px;font-weight:500;letter-spacing:-.02em}
.readout span{font-size:10px;letter-spacing:.12em;text-transform:uppercase;color:var(--dim)}
.a{color:var(--angle)}.o{color:var(--output)}.y{color:var(--yaw)}
.row{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.row label{font-family:var(--mono);font-size:11px;width:60px;color:var(--dim);letter-spacing:.06em}
.row input[type=range]{flex:1;accent-color:var(--angle);min-width:0}
.row.yw input[type=range]{accent-color:var(--yaw)}
.row input[type=number]{width:78px;background:var(--ground);color:var(--ink);
  border:1px solid var(--rule);border-radius:2px;padding:6px 7px;font-family:var(--mono);font-size:13px}
.btns{display:flex;gap:8px;flex-wrap:wrap}
button{flex:1;min-width:84px;background:transparent;color:var(--ink);border:1px solid var(--rule);
  border-radius:2px;padding:11px 8px;font-family:var(--mono);font-size:11px;
  letter-spacing:.1em;text-transform:uppercase;cursor:pointer}
button:hover{border-color:var(--dim)}
button:focus-visible{outline:2px solid var(--angle);outline-offset:2px}
button.danger{color:var(--alert);border-color:#4A2630}
button.go{color:var(--ok);border-color:#26493A}
.tog{font-family:var(--mono);font-size:10px;letter-spacing:.1em;text-transform:uppercase;
  color:var(--dim);cursor:pointer;user-select:none}
.tog b{color:var(--yaw)}
.status{display:flex;justify-content:space-between;font-family:var(--mono);font-size:11px;
  color:var(--dim);border-top:1px solid var(--rule);padding-top:10px;margin-top:12px}
.pill{color:var(--ok)}.pill.bad{color:var(--alert)}.pill.warn{color:var(--output)}
@media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body><div class="wrap">

<h1>BalanceBot</h1>
<div class="sub" id="head">connecting&hellip;</div>

<div class="panel">
  <canvas id="horizon"></canvas>
  <canvas id="trace"></canvas>
  <div class="readout">
    <div><b class="a" id="vAng">--</b><span>tilt deg</span></div>
    <div><b class="o" id="vOut">--</b><span>output</span></div>
    <div><b class="y" id="vYaw">--</b><span>yaw err</span></div>
    <div><b id="vHz">--</b><span>loop hz</span></div>
  </div>
</div>

<div class="panel">
  <span class="lbl"><span>Balance gains</span></span>
  <div class="row"><label>Kp</label><input type=range id=rKp min=0 max=300 step=1>
    <input type=number id=nKp step=1></div>
  <div class="row"><label>Ki</label><input type=range id=rKi min=0 max=400 step=1>
    <input type=number id=nKi step=1></div>
  <div class="row"><label>Kd</label><input type=range id=rKd min=0 max=20 step=.05>
    <input type=number id=nKd step=.05></div>
</div>

<div class="panel">
  <span class="lbl"><span>Yaw hold</span><span class="tog" id=tYaw>yaw <b>--</b></span></span>
  <div class="row yw"><label>Kp</label><input type=range id=rYKp min=0 max=30 step=.1>
    <input type=number id=nYKp step=.1></div>
  <div class="row yw"><label>Ki</label><input type=range id=rYKi min=0 max=30 step=.1>
    <input type=number id=nYKi step=.1></div>
  <div class="row yw"><label>Kd</label><input type=range id=rYKd min=0 max=10 step=.05>
    <input type=number id=nYKd step=.05></div>
  <div class="row yw"><label>Limit</label><input type=range id=rYL min=0 max=200 step=5>
    <input type=number id=nYL step=5></div>
</div>

<div class="panel">
  <span class="lbl"><span>Mechanical</span></span>
  <div class="row"><label>Trim</label><input type=range id=rTr min=-10 max=10 step=.1>
    <input type=number id=nTr step=.1></div>
  <div class="row"><label>Limit</label><input type=range id=rLm min=40 max=255 step=5>
    <input type=number id=nLm step=5></div>
  <div class="row"><label>MinPWM</label><input type=range id=rMp min=0 max=120 step=1>
    <input type=number id=nMp step=1></div>
</div>

<div class="panel">
  <div class="btns">
    <button class="go" id=bArm>Arm</button>
    <button class="danger" id=bDis>Disarm</button>
    <button id=bRst>Reset PID</button>
    <button id=bYz>Zero yaw</button>
    <button id=bSav>Save</button>
    <button id=bDef>Defaults</button>
  </div>
  <div class="status">
    <span>state <b class="pill" id=sState>--</b></span>
    <span>falls <b id=sFalls>--</b></span>
    <span>up <b id=sUp>--</b>s</span>
  </div>
</div>

<script>
const $=i=>document.getElementById(i);
const hz=$('horizon'),tr=$('trace');
const hc=hz.getContext('2d'),tc=tr.getContext('2d');
const N=220,aH=new Array(N).fill(0),oH=new Array(N).fill(0),yH=new Array(N).fill(0);
let cfg={limit:255},busy=false;
const cs=()=>getComputedStyle(document.body);

function fit(c){const d=devicePixelRatio||1,r=c.getBoundingClientRect();
  c.width=r.width*d;c.height=r.height*d;c.getContext('2d').setTransform(d,0,0,d,0,0);return r;}

/* Attitude indicator: the ground rotates, the aircraft mark holds still. */
function drawHorizon(ang){
  const r=fit(hz),w=r.width,h=r.height,cx=w/2,cy=h/2,R=Math.min(w,h)/2-6;
  hc.clearRect(0,0,w,h);
  hc.save();hc.beginPath();hc.arc(cx,cy,R,0,7);hc.clip();
  hc.translate(cx,cy);hc.rotate(-ang*Math.PI/180);
  hc.fillStyle='#0A1119';hc.fillRect(-R*2,-R*2,R*4,R*2);
  hc.fillStyle='#132436';hc.fillRect(-R*2,0,R*4,R*2);
  hc.strokeStyle=cs().getPropertyValue('--angle');hc.lineWidth=1.5;
  hc.beginPath();hc.moveTo(-R*2,0);hc.lineTo(R*2,0);hc.stroke();
  hc.strokeStyle='#26374A';hc.lineWidth=1;
  for(let d=-40;d<=40;d+=10){if(!d)continue;const y=d*(R/50),len=d%20?7:14;
    hc.beginPath();hc.moveTo(-len,y);hc.lineTo(len,y);hc.stroke();}
  hc.restore();
  hc.strokeStyle='#26374A';hc.beginPath();hc.arc(cx,cy,R,0,7);hc.stroke();
  hc.strokeStyle=cs().getPropertyValue('--ink');hc.lineWidth=2;hc.beginPath();
  hc.moveTo(cx-26,cy);hc.lineTo(cx-8,cy);hc.moveTo(cx+8,cy);hc.lineTo(cx+26,cy);hc.stroke();
}

function drawTrace(){
  const r=fit(tr),w=r.width,h=r.height,mid=h/2;
  tc.clearRect(0,0,w,h);
  tc.strokeStyle='#1E2C3C';tc.lineWidth=1;
  tc.beginPath();tc.moveTo(0,mid);tc.lineTo(w,mid);tc.stroke();
  const line=(arr,sc,col)=>{tc.strokeStyle=col;tc.lineWidth=1.5;tc.beginPath();
    for(let i=0;i<N;i++){const x=i*w/(N-1);
      const y=mid-Math.max(-1,Math.min(1,arr[i]/sc))*(mid-4);
      i?tc.lineTo(x,y):tc.moveTo(x,y);}tc.stroke();};
  line(yH,45,cs().getPropertyValue('--yaw'));
  line(oH,cfg.limit||255,cs().getPropertyValue('--output'));
  line(aH,20,cs().getPropertyValue('--angle'));
}

function bind(rid,nid,key,ep){
  const R=$(rid),Nn=$(nid);
  const push=v=>{Nn.value=v;R.value=v;fetch(`/api/${ep}?${key}=${v}`).catch(()=>{});};
  R.oninput=()=>{Nn.value=R.value;};
  R.onchange=()=>push(R.value);
  Nn.onchange=()=>push(Nn.value);
  return v=>{if(document.activeElement!==R&&document.activeElement!==Nn){R.value=v;Nn.value=v;}};
}
const setKp=bind('rKp','nKp','kp','pid'),  setKi=bind('rKi','nKi','ki','pid'),
      setKd=bind('rKd','nKd','kd','pid'),
      setYKp=bind('rYKp','nYKp','ykp','pid'), setYKi=bind('rYKi','nYKi','yki','pid'),
      setYKd=bind('rYKd','nYKd','ykd','pid'), setYL=bind('rYL','nYL','ylimit','config'),
      setTr=bind('rTr','nTr','trim','config'), setLm=bind('rLm','nLm','limit','config'),
      setMp=bind('rMp','nMp','minpwm','config');

const hit=p=>fetch('/api/'+p).then(r=>r.json()).then(apply).catch(()=>{});
$('bArm').onclick=()=>hit('arm');   $('bDis').onclick=()=>hit('disarm');
$('bRst').onclick=()=>hit('reset'); $('bYz').onclick=()=>hit('zeroyaw');
$('bSav').onclick=()=>hit('save').then(()=>hit('state'));
$('bDef').onclick=()=>{if(confirm('Reset all gains to firmware defaults?'))hit('defaults');};
$('tYaw').onclick=()=>hit('config?yawon='+(cfg.yawon?0:1));

function apply(s){
  if(!s||s.angle===undefined)return;
  cfg=s;
  aH.push(s.angle);aH.shift(); oH.push(s.output);oH.shift(); yH.push(s.yaw);yH.shift();
  $('vAng').textContent=s.angle.toFixed(1);
  $('vOut').textContent=s.output.toFixed(0);
  $('vYaw').textContent=s.yaw.toFixed(0);
  $('vHz').textContent=s.loopHz.toFixed(0);
  $('vHz').style.color=s.loopHz<50?'var(--alert)':s.loopHz<70?'var(--output)':'var(--dim)';
  setKp(s.kp);setKi(s.ki);setKd(s.kd);
  setYKp(s.ykp);setYKi(s.yki);setYKd(s.ykd);setYL(s.ylimit);
  setTr(s.trim);setLm(s.limit);setMp(s.minpwm);
  $('tYaw').innerHTML='yaw <b>'+(s.yawon?'on':'off')+'</b>';
  const st=$('sState');
  st.textContent=!s.dmp?'no dmp':!s.armed?'disarmed':s.fallen?'down':'balancing';
  st.className='pill'+(!s.dmp||s.fallen?' bad':(!s.armed?' warn':''));
  $('sFalls').textContent=s.falls;$('sUp').textContent=s.uptime;
  $('head').textContent='192.168.4.1  ·  '+(s.dmp?'dmp ok':'dmp fault');
  drawHorizon(s.angle);drawTrace();
}

async function poll(){
  if(busy)return; busy=true;
  try{apply(await(await fetch('/api/state')).json());}
  catch(e){$('head').textContent='link lost — reconnect to BalanceBot';}
  busy=false;
}
setInterval(poll,100);
addEventListener('resize',()=>{drawHorizon(cfg.angle||0);drawTrace();});
poll();
</script></div></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", PAGE_INDEX); }

void setupNetwork() {
  WiFi.mode(WIFI_AP);
  netUp = WiFi.softAP(AP_SSID, AP_PASS);

  // Radio power-save adds tens of ms of latency the balance loop cannot afford.
  WiFi.setSleep(false);

  Serial.print(F("[NET] AP \"")); Serial.print(AP_SSID);
  Serial.print(F("\"  pass ")); Serial.println(AP_PASS);
  Serial.print(F("[NET] http://")); Serial.println(WiFi.softAPIP());

  if (!netUp) { Serial.println(F("[NET] softAP failed")); return; }

  server.on("/",             handleRoot);
  server.on("/api/state",    sendState);
  server.on("/api/pid",      handlePid);
  server.on("/api/config",   handleConfig);
  server.on("/api/offsets",  handleOffsets);
  server.on("/api/arm",      handleArm);
  server.on("/api/disarm",   handleDisarm);
  server.on("/api/reset",    handleReset);
  server.on("/api/zeroyaw",  handleZeroYaw);
  server.on("/api/save",     handleSave);
  server.on("/api/load",     handleLoad);
  server.on("/api/defaults", handleDefaults);
  server.on("/api/reboot",   handleReboot);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("[NET] HTTP server up"));
}
#endif  // ENABLE_WIFI

// =====================================================================================
//  SETUP
// =====================================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { }

  Serial.println(F("\n================================================"));
  Serial.println(F(" ESP32-C6 Self-Balancing Robot (DMP + Yaw + AP)"));
  Serial.println(F("================================================"));

  setupMotors();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);                 // 100 kHz - reliable on clones + long wires

  Serial.println(F("[MPU] Initializing device..."));
  mpu.initialize();
  Serial.print(F("[MPU] Connection test: "));
  Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 FAILED"));

  Serial.println(F("[DMP] Initializing DMP..."));
  for (int attempt = 1; attempt <= 3; attempt++) {
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) break;
    Serial.print(F("[DMP] attempt ")); Serial.print(attempt);
    Serial.print(F(" failed (code ")); Serial.print(devStatus);
    Serial.println(F(") - retrying"));
    delay(300);
  }

  loadConfig();      // NVS overrides the DEF_* values, including offsets

  // Offsets from imu_cal.ino. Push new ones live without reflashing:
  //   curl "http://192.168.4.1/api/offsets?ax=..&ay=..&az=..&gx=..&gy=..&gz=.."
  //   curl "http://192.168.4.1/api/save"
  applyOffsets();

  if (devStatus == 0) {
    Serial.println(F("[DMP] Enabling DMP..."));
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("[DMP] FIFO packet size: ")); Serial.println(packetSize);
    mpu.resetFIFO();
    dmpReady = true;
    Serial.println(F("[DMP] Ready."));
  } else {
    Serial.print(F("[DMP] INIT FAILED (code ")); Serial.print(devStatus); Serial.println(F(")"));
    Serial.println(F("      1 = memory load failed"));
    Serial.println(F("      2 = DMP config updates failed"));
    Serial.println(F("      Motors will stay disabled."));
    stopMotors();
  }

  pid.SetMode(AUTOMATIC);
  yawPid.SetMode(AUTOMATIC);
  applyTunings();

  // Print the values actually in use, not a hardcoded string that drifts out of date.
  Serial.print(F("[PID] Kp=")); Serial.print(cfgKp);
  Serial.print(F(" Ki="));      Serial.print(cfgKi);
  Serial.print(F(" Kd="));      Serial.print(cfgKd);
  Serial.print(F(" limit=+/-")); Serial.println(cfgLimit);
  Serial.print(F("[YAW] Kp=")); Serial.print(cfgYKp);
  Serial.print(F(" Ki="));      Serial.print(cfgYKi);
  Serial.print(F(" Kd="));      Serial.print(cfgYKd);
  Serial.print(F(" limit=+/-")); Serial.print(cfgYLimit);
  Serial.println(cfgYawOn ? F("  [ON]") : F("  [OFF]"));

#if ENABLE_WIFI
  setupNetwork();
#else
  Serial.println(F("[NET] WiFi disabled at compile time"));
#endif

  Serial.println(F("Stand the robot upright to arm."));
  loopWindowMs = millis();
  lastPacketUs = micros();
}

// =====================================================================================
//  NETWORK SERVICE - only ever called from the idle gap between DMP packets
// =====================================================================================
inline void serviceNetwork() {
#if ENABLE_WIFI
  if (!netUp) return;
  // A DMP packet lands every ~10 ms. Only step into the web server if enough of that
  // window remains that a short request cannot push us past the next packet.
  if ((micros() - lastPacketUs) < (10000 - NET_SERVICE_MIN_US)) {
    server.handleClient();
  }
#endif
}

// =====================================================================================
//  MAIN LOOP  -  free-running, no blocking delays
// =====================================================================================
void loop() {
  if (!dmpReady) { stopMotors(); serviceNetwork(); return; }

  fifoCount = mpu.getFIFOCount();

  if (fifoCount >= 1024) {                                   // overflow: data is junk
    mpu.resetFIFO();
    Serial.println(F("[FIFO] overflow - reset"));
    return;
  }
  if (packetSize > 0 && (fifoCount % packetSize) != 0) {     // partial packet
    mpu.resetFIFO();
    return;
  }
  if (fifoCount < packetSize) {                              // idle window
    serviceNetwork();
    return;
  }

  while (fifoCount >= packetSize) {                          // drain to newest packet
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }
  lastPacketUs = micros();

  // ---------------- Attitude ----------------
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  double angle = (ypr[cfgAxis] * 180.0 / M_PI) + cfgTrim;
  pidInput = angle;
  pid.Compute();

  // ---------------- Yaw hold ----------------
  // DMP yaw has no magnetometer reference, so it drifts and wraps at +/-180.
  // Hold RELATIVE to a heading captured at arm time, and wrap the deviation.
  rawYaw = ypr[0] * 180.0 / M_PI;
  yawErr = wrap180(rawYaw - yawRef);
  yawInput = yawErr;          // setpoint stays 0; output opposes the deviation
  if (cfgYawOn) yawPid.Compute();
  else          yawOutput = 0.0;

  // ---------------- Fall detection ----------------
  uint32_t now = millis();
  double fallThreshold = cfgLimit - FALL_MARGIN;

  if (fabs(angle) > FALL_ANGLE_LIMIT) {
    if (!fallen) fallCount++;
    fallen = true;
  }
  if (fabs(pidOutput) >= fallThreshold) {
    if (saturatedSince == 0) {
      saturatedSince = now;
    } else if ((now - saturatedSince) > FALL_HOLD_MS) {
      if (!fallen) fallCount++;
      fallen = true;
    }
  } else {
    saturatedSince = 0;
  }

  // ---------------- Drive / protect ----------------
  if (!armed || fallen) {
    stopMotors();
    if (armed && fallen && fabs(angle) < RECOVER_ANGLE) {
      fallen = false;
      saturatedSince = 0;
      resetPid();                       // also re-captures yawRef
      mpu.resetFIFO();
      Serial.println(F("[FALL] recovered - PID reset, re-armed"));
    }
  } else {
    driveMotorPair(pidOutput + yawOutput, pidOutput - yawOutput);
  }

  // ---------------- Telemetry, once per second ----------------
  loopCounter++;
  if (now - loopWindowMs >= 1000) {
    loopHz = loopCounter * 1000.0 / (now - loopWindowMs);
    loopCounter = 0;
    loopWindowMs = now;
    Serial.print(F("angle,"));  Serial.print(angle, 2);
    Serial.print(F("  out,"));  Serial.print(pidOutput, 1);
    Serial.print(F("  yawErr,")); Serial.print(yawErr, 1);
    Serial.print(F("  yawOut,")); Serial.print(yawOutput, 1);
    Serial.print(F("  hz,"));   Serial.print(loopHz, 0);
    Serial.print(F("  "));
    Serial.println(!armed ? "DISARMED" : (fallen ? "DOWN" : "BAL"));
  }
}

/*
 * =====================================================================================
 *  API QUICK REFERENCE   (base: http://192.168.4.1)
 * =====================================================================================
 *   /                       tuning page
 *   /api/state              full telemetry + config as JSON
 *   /api/pid?kp=&ki=&kd=            balance gains
 *   /api/pid?ykp=&yki=&ykd=         yaw gains
 *   /api/config?trim=&limit=&ylimit=&minpwm=&axis=&yawon=
 *   /api/offsets?ax=&ay=&az=&gx=&gy=&gz=
 *   /api/arm  /api/disarm  /api/reset  /api/zeroyaw
 *   /api/save  /api/load  /api/defaults  /api/reboot
 *
 *   All params optional — omit one and its current value is kept. Every config
 *   endpoint returns the full state so you can confirm what landed.
 *
 *   curl "http://192.168.4.1/api/pid?kp=80&kd=0.8"
 *   curl "http://192.168.4.1/api/disarm"          # kill switch
 *
 * =====================================================================================
 *  TUNING
 * =====================================================================================
 *   0. Hold upright. Confirm angle ~0 vertical, POSITIVE tilted forward.
 *      If negative, swap one motor's wires on OUT3/OUT4.
 *   1. Yaw off. Ki=0, Kd=0. Raise Kp until it fights back with even oscillations.
 *   2. Add Kd until the oscillation damps. Too much = high-frequency buzzing.
 *   3. Add Ki last. Too much = slow lumbering oscillation that grows.
 *   4. Trim in 0.2 deg steps until it stops drifting.
 *   5. Yaw on last, once balance is solid.
 *
 *   Sliders apply instantly — no reflash. Save writes to NVS.
 * =====================================================================================
 */
