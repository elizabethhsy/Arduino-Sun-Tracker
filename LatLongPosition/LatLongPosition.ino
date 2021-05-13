#include <PWMServo.h>
#include "math.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// max SERVO pos 140 deg!!

#define STEP_PIN 11
#define DIR_PIN 12
#define SERVO_PIN 10

SoftwareSerial ss(A3, A2);
TinyGPSPlus gps;
PWMServo myservo;  // create servo object to control a servo
float stepper_angle = 0;
float LT = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //ss.begin(9600);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  myservo.attach(SERVO_PIN_B);  // attaches the servo on pin 12 to the servo object
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

  Serial.println(safe_angle);
  myservo.write((int) safe_angle);
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

bool isLeapYear() {
  int yr = gps.date.year();
  if(yr%4 == 0) {
    if(yr%100!=0 or yr%400==0) {
      return true;
    }
  }
  return false;
}

int daysSinceStartOfYear() {
  int febDays = 28;
  if(isLeapYear()) {
    febDays = 29;
  }
  int months[12] = {31, febDays, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // number of days in each month
  int days = 0;
  for(int i = 1; i < gps.date.month(); i++) {
    days += months[i];
  }
  days += gps.date.day();
  return days;
}

void loop() {
  //while (ss.available() > 0)
    //gps.encode(ss.read());
  
  // put your main code here, to run repeatedly:
  // CONST
  float pi = 3.14159265359;

  //float LT = gps.time.hour() + (float)gps.time.minute()/60.0;
  
  // values to input
  float latitude = 51.500150;
  float longitude = 1;
  //int time_difference = 1;
  int d = daysSinceStartOfYear(); // days since the start of the year
 
  // calculating the angles using https://www.pveducation.org/pvcdrom/properties-of-sunlight/the-suns-position
  float latitude_rad = degToRad(latitude);
  //int lstm = 15*time_difference;
  float B = (2*pi)/365*(d - 81); // angle in radians
  float EoT = 9.87*sin(2*B)-7.53*cos(B)-1.5*sin(B); // equation of time
  float TC = 4*(longitude) + EoT; // time correction
  float LST = LT + TC/60; // local solar time
  float HRA = degToRad(15*(LST-12)); // hour angle (rad)
  float declination = 23.45*sin(B); // declination angle
  float declination_rad = degToRad(declination); // declination angle (rad)
  float elevation_rad = asin(sin(declination_rad)*sin(latitude_rad) + cos(declination_rad)*cos(latitude_rad)*cos(HRA)); // elevation (rad)
  float elevation_deg = radToDeg(elevation_rad);
  float azimuth = acos((sin(declination_rad)*cos(latitude_rad) - cos(declination_rad)*sin(latitude_rad)*cos(HRA))/cos(elevation_rad));
  float azimuth_deg = radToDeg(azimuth);

  if(LT > 12) {
    azimuth_deg = 360 - azimuth_deg;
  }

  //Serial.println("LT " + (String) LT);
  //Serial.println("TC " + (String) TC);
  Serial.println("Days " + (String) d);
  //Serial.println("declination " + (String) declination);
  Serial.println("GPS " + (String) gps.time.value());
  Serial.println("hour " + (String) LT);
  Serial.println("elevation " + (String) elevation_deg); //54
  //Serial.println("EoT " + (String) EoT);
  //Serial.println("LST " + (String) LST);
  //Serial.println("HRA " + (String) radToDeg(HRA/pi));
  Serial.println("azimuth " + (String) azimuth_deg); //162

  Serial.println("");
  setServoAngle(elevation_deg);
  delay(2000);
  
  //move_stepper_to_angle(azimuth_deg);

  delay(3000);

  LT = (LT+1);
  if (LT >= 24) {
    LT -= 24;
  }
}
