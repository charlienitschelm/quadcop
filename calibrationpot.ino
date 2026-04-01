const int DIR_1 = 2,  PWM_1 = 3,  POT_1 = A3;
const int DIR_2 = 4,  PWM_2 = 5,  POT_2 = A4;
const int DIR_3 = 6,  PWM_3 = 7,  POT_3 = A5;

const int SPEED = 100;

void setAll(bool extend) {
  // LOW = extend, HIGH = retract
  digitalWrite(DIR_1, extend ? LOW : HIGH);
  digitalWrite(DIR_2, extend ? LOW : HIGH);
  digitalWrite(DIR_3, extend ? LOW : HIGH);
  analogWrite(PWM_1, SPEED);
  analogWrite(PWM_2, SPEED);
  analogWrite(PWM_3, SPEED);
}

void stopAll() {
  analogWrite(PWM_1, 0);
  analogWrite(PWM_2, 0);
  analogWrite(PWM_3, 0);
}

void setup() {
  pinMode(DIR_1, OUTPUT); pinMode(PWM_1, OUTPUT);
  pinMode(DIR_2, OUTPUT); pinMode(PWM_2, OUTPUT);
  pinMode(DIR_3, OUTPUT); pinMode(PWM_3, OUTPUT);

  Serial.begin(9600);
  Serial.println("=== CALIBRATION START ===");
  Serial.println("Retracting all fully...");

  setAll(false); // retract
  delay(10000);
  stopAll();
  delay(1000);

  int min1 = analogRead(POT_1);
  int min2 = analogRead(POT_2);
  int min3 = analogRead(POT_3);

  Serial.println("--- RETRACTED (MIN) ---");
  Serial.print("Act 1: "); Serial.println(min1);
  Serial.print("Act 2: "); Serial.println(min2);
  Serial.print("Act 3: "); Serial.println(min3);

  delay(2000);
  Serial.println("Extending all fully...");

  setAll(true); // extend
  delay(10000);
  stopAll();
  delay(1000);

  int max1 = analogRead(POT_1);
  int max2 = analogRead(POT_2);
  int max3 = analogRead(POT_3);

  Serial.println("--- EXTENDED (MAX) ---");
  Serial.print("Act 1: "); Serial.println(max1);
  Serial.print("Act 2: "); Serial.println(max2);
  Serial.print("Act 3: "); Serial.println(max3);

  Serial.println("=== CALIBRATION DONE ===");
  Serial.println("Copy these 6 numbers and send them to Claude!");

  stopAll();
}

void loop() {}