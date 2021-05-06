#include <Servo.h>

// max SERVO pos 140 deg!!

#define STEP_PIN 11
#define DIR_PIN 12
#define SERVO_PIN 10

Servo myservo;  // create servo object to control a servo
int pos = 0;
float angle = 150;

void setup() {
  // put your setup code here, to run once:
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 12 to the servo object

  delay(3000);

  move_stepper_to_angle(270);

  delay(5000);

  move_stepper_to_angle(180);
}

// set servo angle
void setServoAngle(float angle) {
  float safe_angle = angle;
  if(angle < 0) {
    safe_angle = 0;
  }
  
  if(angle > 140) {
    safe_angle = 140;
  }

  myservo.write(safe_angle);
}

float stepper_angle = 0;

void move_stepper_to_angle(float angle) {
  
  float angle_diff = angle - stepper_angle;

  if(angle_diff > 0) {
    digitalWrite(DIR_PIN, LOW);
  } else {
    digitalWrite(DIR_PIN, HIGH);
  }
  
  stepper_angle = angle;

  int num_steps = (int)(abs(angle_diff / 1.8));

  for (int i = 0; i < num_steps; i ++) {
    digitalWrite(STEP_PIN, HIGH);
    delay(2);
    digitalWrite(STEP_PIN, LOW);
    delay(2);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //setServoAngle(angle);
  //delay(1000);
}
