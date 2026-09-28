# BalanceBot — ESP32-C6 Self-Balancing Robot

Two-wheel self-balancing robot on ESP32-C6 + MPU6050 DMP with balance PID and yaw anti-spin PID driving independent L298N channels.

## Demo

> Placeholder — add photo / video link here.
>
> - Video: `<link to demo video>`
> - Photo: `docs/demo.jpg`

## Firmware Variants

| Sketch | Purpose |
|---|---|
| `esp32c6_self_balancing_robot/esp32c6_self_balancing_robot.ino` | Main balance loop (DMP + balance PID + yaw PID, fall detect + auto-recovery) |
| `esp32c6_balancebot_ap/esp32c6_balancebot_ap.ino` | Same balancer + WiFi AP tuner (`BalanceBot` / `http://192.168.4.1`) with NVS-persisted gains |
| `imu_cal/imu_cal.ino` | Auto-calibration — hold upright and still, prints 6 offsets |
| `motor_test/motor_test.ino` | Basic motor spin test |
| `motor_test2/motor_test2.ino` | Speed-adjustable motor direction test |
| `mpu_test/mpu_test.ino` | Raw MPU6050 read test |
| `i2c_scan/i2c_scan.ino` | I2C bus scanner (expects 0x68) |
| `diag/diag.ino` | MPU WHO_AM_I diagnostic |

Libraries required: `I2Cdev`, `MPU6050` (jrowberg), `PID_v1`.

## BOM

| Qty | Part | Notes |
|---|---|---|
| 1 | ESP32-C6 DevKitC-1 | Arduino-ESP32 core 3.x |
| 1 | MPU6050 module (clone OK) | Must be powered from 3V3, not 5V |
| 1 | L298N motor driver | Keep 5V jumper ON to power ESP32 |
| 2 | 300 RPM BO motors + wheels | Mirrored mount on acrylic chassis |
| 2 | 18650 cells in series (~7.4V) | To L298N 12V + GND |
| 1 | Acrylic chassis + mounts | — |

## Wiring

| From | To | Notes |
|---|---|---|
| MPU6050 VCC | ESP32 3V3 | NOT 5V — clones fail on 5V |
| MPU6050 GND | ESP32 GND | — |
| MPU6050 SDA | ESP32 GPIO21 | `I2C_SDA=21` |
| MPU6050 SCL | ESP32 GPIO22 | `I2C_SCL=22` |
| MPU6050 AD0 | GND | I2C address 0x68 |
| L298N IN1 | ESP32 GPIO6 | Motor A direction |
| L298N IN2 | ESP32 GPIO7 | Motor A direction |
| L298N ENA | ESP32 GPIO8 | Motor A PWM |
| L298N IN3 | ESP32 GPIO4 | Motor B direction |
| L298N IN4 | ESP32 GPIO5 | Motor B direction |
| L298N ENB | ESP32 GPIO10 | Motor B PWM |
| 2x18650 + | L298N 12V | ~7.4V in |
| 2x18650 − | L298N GND + ESP32 GND | Common GND mandatory |
| L298N 5V | ESP32 5V/VIN | Via on-board regulator, jumper ON |

Motor outputs: Motor A → OUT1/OUT2, Motor B → OUT3/OUT4.
`driveMotorPair(balance+yaw, balance-yaw)` — same direction = forward/back, differential = yaw correction. If the robot accelerates INTO a fall, swap Motor B wires on OUT3/OUT4.

Reference photos: `esp32-c6 pinout.png`, `Build_ Manual_Self-Balancing Robot.pdf` (see Large Files note below).

## PID Configuration

### Balance PID

| Param | Value |
|---|---|
| Kp | 8.0 |
| Ki | 10.0 |
| Kd | 1.8 |
| Output limit | 200 (0–255 PWM scale) |
| MOTOR_MIN_PWM | 45 |
| Sample time | 10 ms (matches DMP 100 Hz) |

> Tuning note from project: `Ki=10` is high for this hardware (typical 0–2). If slow oscillation occurs, lower `Ki` to 0–2 and raise `Kp` to 12–18 first.

### Yaw PID (anti-spin)

| Param | Value |
|---|---|
| Kp | 2.5 |
| Ki | 0 |
| Kd | 0.6 |
| Output limit | 80 |

## Calibration Offsets

From `imu_cal.ino`, robot held upright and still. Re-run after any mechanical remount. Values preserved exactly:

| Register | Value |
|---|---|
| XAccel | -5758 |
| YAccel | 5042 |
| ZAccel | 10170 |
| XGyro | 11 |
| YGyro | 55 |
| ZGyro | -4 |

```cpp
mpu.setXAccelOffset(-5758);
mpu.setYAccelOffset( 5042);
mpu.setZAccelOffset(10170);
mpu.setXGyroOffset (   11);
mpu.setYGyroOffset (   55);
mpu.setZGyroOffset (   -4);
```

## CLI Commands

Replace `<PORT>` with your serial port (find via `arduino-cli board list`). Close Serial Monitor before uploading.

```bash
# Compile balancing robot
arduino-cli compile --fqbn esp32:esp32:esp32c6 esp32c6_self_balancing_robot

# Upload balancing robot
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32c6 esp32c6_self_balancing_robot

# Compile calibration sketch
arduino-cli compile --fqbn esp32:esp32:esp32c6 imu_cal

# Upload calibration sketch (hold robot upright and still)
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32c6 imu_cal

# I2C scanner
arduino-cli compile --fqbn esp32:esp32:esp32c6 i2c_scan
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32c6 i2c_scan

# List boards/ports
arduino-cli board list

# Serial monitor (or use Arduino IDE at 115200 baud)
arduino-cli monitor -p <PORT> --config 115200
```

Resume procedure: hold robot upright → press RESET → wait for `READY` / `angle,~0` in serial → release. Serial telemetry: `angle,<deg>  out,<pwm>  yaw,<deg>  state,<BAL|FALL>`.

## Known Issues + Fixes

| Problem | Fix |
|---|---|
| DMP init code 1 (memory load failed) | Lower I2C to 100 kHz + 3× retry loop |
| MPU6050 clone WHO_AM_I=0x70, `testConnection()` prints FAILED | Expected on clones; DMP still works, offsets apply regardless |
| MPU6050 disappears from bus | VCC was on 5V → move to 3V3; reseat SDA/SCL |
| One motor not spinning (B channel) | Check ENB→GPIO10, OUT3/OUT4, jumpers; swap Motor B to OUT1/OUT2 to isolate driver vs wiring |
| Slow motor response / deadband | `MOTOR_MIN_PWM=45`, raise `Kp`, `PID_OUTPUT_LIMIT=200` |
| DMP firmware load unreliable | 100 kHz I2C + retry + solid 3V3 power |
| Fall / saturation shutdown | Angle >45° or output saturated ~2 s → FALL; re-arms when \|angle\| < 3° |

## Next Steps

1. Test balance: hold upright → RESET → Serial Monitor → `angle≈0` vertical, forward tilt = positive.
2. Tune PID: lower `Ki` to 0–2, raise `Kp` to 12–18, adjust `Kd` for damping; add small `Ki` last.
3. Verify yaw: rotate by hand → wheels counter-rotate per `yaw` telemetry.
4. Optional remote: BLE phone joystick for forward/back/steer (ESP32-C6 native BLE).
5. Optional battery monitor: voltage divider to ADC pin.

## Project Structure

```text
balancebot/
├── README.md
├── .gitignore
├── esp32c6_self_balancing_robot/
│   └── esp32c6_self_balancing_robot.ino
├── esp32c6_balancebot_ap/
│   └── esp32c6_balancebot_ap.ino
├── imu_cal/
│   └── imu_cal.ino
├── motor_test/
│   └── motor_test.ino
├── motor_test2/
│   └── motor_test2.ino
├── mpu_test/
│   └── mpu_test.ino
├── i2c_scan/
│   └── i2c_scan.ino
├── diag/
│   └── diag.ino
├── Build_ Manual_Self-Balancing Robot.pdf
└── esp32-c6 pinout.png
```

## License

MIT — see LICENSE. No tuning results are claimed beyond the values tabled above.
