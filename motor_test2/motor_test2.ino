#define IN1 6
#define IN2 7
#define ENA 8
#define IN3 4
#define IN4 5
#define ENB 10

int speed = 200;

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  delay(1000);
  Serial.println("=== DIRECTION TEST ===");
  Serial.println("Mark the side the robot moves on the FORWARD phase = your FRONT");
  Serial.println("Type a number 0-255 in the monitor to change speed live (Enter).");
}

void loop() {
  readSpeed();

  Serial.println("FORWARD phase (5s) -> side moving = FRONT");
  drive(1, speed);
  delay(5000);
  stop();
  delay(3000);

  readSpeed();

  Serial.println("BACKWARD phase (5s) -> opposite side = REAR");
  drive(-1, speed);
  delay(5000);
  stop();
  delay(3000);
}

void drive(int dir, int spd) {
  if (dir > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void readSpeed() {
  if (Serial.available()) {
    int v = Serial.parseInt();
    if (v >= 0 && v <= 255) {
      speed = v;
      Serial.print("Speed set to ");
      Serial.println(speed);
    }
    while (Serial.available()) Serial.read();
  }
}