#include "math.h"
#include <TimeLib.h>
#include <Servo.h>

// max SERVO pos 140 deg!!

#define STEP_PIN 11
#define DIR_PIN 12
#define SERVO_PIN 10

Servo myservo;  // create servo object to control a servo
float stepper_angle = 0;
float LT = 0; // local time (hr)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setTime(1620300099);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 12 to the servo object
}

// set servo angle
void setServoAngle(float angle) {
  float safe_angle = 90.0 - angle;
  if(safe_angle < 0) {
    safe_angle = 0;
  }
  
  if(safe_angle > 140) {
    safe_angle = 140;
  }

  //Serial.println(safe_angle);
  myservo.write(safe_angle);
}

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

float radToDeg(float angle) {
  // CONST
  float pi = 3.14159265359;
  
  angle = angle/pi*180;
  return angle;
}

float degToRad(float angle) {
  // CONST
  float pi = 3.14159265359;
  
  angle = angle/180*pi;
  return angle;
}

bool isLeapYear(int year) {
  if(year%4 == 0) {
    if(year%100!=0 or year%400==0) {
      return true;
    }
  }
  return false;
}

int daysSinceStartOfYear(time_t t) {
  int febDays = 28;
  if(isLeapYear(year(t))) {
    febDays = 29;
  }
  int months[12] = {31, febDays, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // number of days in each month
  int days;
  for(int i = 1; i < month(t); i++) {
    days += months[i];
  }
  days += day(t);
  return days;
}

void loop() {
  // put your main code here, to run repeatedly:
  // CONST
  float pi = 3.14159265359;

  time_t t = now(); // Store the current time in time
  
  // values to input
  float latitude = 51.500150;
  float longitude = 1;
  int time_difference = 1;
  int d = 126; // days since the start of the year
 
  // calculating the angles using https://www.pveducation.org/pvcdrom/properties-of-sunlight/the-suns-position
  float latitude_rad = degToRad(latitude);
  int lstm = 15*time_difference;
  float B = (2*pi)/365*(d - 81); // angle in radians
  float EoT = 9.87*sin(2*B)-7.53*cos(B)-1.5*sin(B); // equation of time
  float TC = 4*(longitude - lstm) + EoT; // time correction
  float LST = LT + TC/60; // local solar time
  float HRA = degToRad(15*(LST-12)); // hour angle (rad)
  float declination = 23.45*sin(B); // declination angle
  float declination_rad = degToRad(declination); // declination angle (rad)
  float elevation_rad = asin(sin(declination_rad)*sin(latitude_rad) + cos(declination_rad)*cos(latitude_rad)*cos(HRA)); // elevation (rad)
  float elevation_deg = radToDeg(elevation_rad);
  float azimuth = acos((sin(declination_rad)*cos(latitude_rad) - cos(declination_rad)*sin(latitude_rad)*cos(HRA))/cos(elevation_rad));
  float azimuth_deg = radToDeg(azimuth);

  //Serial.println("LT " + (String) LT);
  //Serial.println("TC " + (String) TC);
  //Serial.println("Days " + (String) d);
  //Serial.println("declination " + (String) declination);
  Serial.println("hour " + (String) LT);
  Serial.println("elevation " + (String) elevation_deg); //54
  //Serial.println("EoT " + (String) EoT);
  //Serial.println("LST " + (String) LST);
  //Serial.println("HRA " + (String) radToDeg(HRA/pi));
  Serial.println("azimuth " + (String) azimuth_deg); //162

  Serial.println("");
  setServoAngle(elevation_deg);
  delay(2000);
  
  move_stepper_to_angle(azimuth_deg);

  delay(3000);

  LT += 1;
}
