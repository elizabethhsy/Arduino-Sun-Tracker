#define STEP_PIN 10
#define DIR_PIN 11

bool dir = true;

void setup() {
  // put your setup code here, to run once:
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(DIR_PIN, dir);

  for (int i = 0; i < 200; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delay(2);
      digitalWrite(STEP_PIN, LOW);  
      delay(2);
  }

  dir = !dir;
  delay(1000);
}
