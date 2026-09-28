#define IN1 6
#define IN2 7
#define ENA 8
#define IN3 4
#define IN4 5
#define ENB 10

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcAttach(ENA, 5000, 8);
  ledcAttach(ENB, 5000, 8);
  Serial.println("Motor test starting in 2s...");
  delay(2000);
}

void loop() {
  Serial.println("FORWARD 2s");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 200); ledcWrite(ENB, 200);
  delay(2000);

  Serial.println("STOP");
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
  delay(1000);

  Serial.println("BACKWARD 2s");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 200); ledcWrite(ENB, 200);
  delay(2000);

  Serial.println("STOP");
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
  delay(1000);

  Serial.println("SPEED SWEEP 0->255");
  for (int s = 0; s <= 255; s += 25) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    ledcWrite(ENA, s); ledcWrite(ENB, s);
    delay(300);
  }
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
  delay(1000);
}
