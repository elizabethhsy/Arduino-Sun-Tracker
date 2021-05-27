#include <PWMServo.h>
#include "math.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal.h>

// max SERVO pos 140 deg!!

#define STEP_PIN 11
#define DIR_PIN 12
#define EN_PIN A0
#define SERVO_PIN 10

SoftwareSerial ss(A3, A2);
TinyGPSPlus gps;
PWMServo myservo;  // create servo object to control a servo
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
float stepper_angle = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ss.begin(9600);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH);
  
  myservo.attach(SERVO_PIN_B);  // attaches the servo on pin 12 to the servo object

  //LCD
  lcd.begin(16, 2);

  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  lcd.clear();
  lcd.print("Calibrate to");
  lcd.setCursor(0, 1);
  lcd.print("North");

  while(digitalRead(8) != LOW) {
    delay(10);
  }

  digitalWrite(EN_PIN, LOW);
  lcd.clear();
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
  myservo.write((int) safe_angle);
}

void move_stepper_to_angle(float angle) {
  
  float angle_diff = angle - stepper_angle;

  if(angle_diff > 0) {
    digitalWrite(DIR_PIN, LOW);
  } else {
    digitalWrite(DIR_PIN, HIGH);
  }

  int num_steps = (int)(abs(angle_diff / 1.8));

  for (int i = 0; i < num_steps; i ++) {
    digitalWrite(STEP_PIN, HIGH);
    delay(2);
    digitalWrite(STEP_PIN, LOW);
    delay(2);
    if(angle_diff < 0) {
      stepper_angle -= 1.8;
    } else {
      stepper_angle += 1.8;
    }
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
  while (ss.available() > 0)
    gps.encode(ss.read());

    
  if (gps.time.value() != 0) {
  // put your main code here, to run repeatedly:
  // CONST
  float pi = 3.14159265359;

  float LT = gps.time.hour() + (float)gps.time.minute()/60.0;
  
  // values to input
  float latitude = 51.495385;
  float longitude = -0.127149;
  int time_difference = 1;
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
  
  move_stepper_to_angle(azimuth_deg);

  //LT = (LT+1);
  //if (LT >= 24) {
  //  LT -= 24;
  //}
  
  lcd.clear();
  lcd.print("Azimuth: " + (String) azimuth_deg);
  lcd.setCursor(0, 1);
  lcd.print("Elevation: " + (String) elevation_deg);

  delay(5000);
  lcd.clear();
  lcd.print("Time: " + (String) (gps.time.hour() + time_difference) + ":" + (String) gps.time.minute());
  //lcd.setCursor(0, 1);

  delay(3000);
  
  } else {
    lcd.clear();
    lcd.print("GPS can't");
    lcd.setCursor(0, 1);
    lcd.print("find time");
    delay(2000);
  }
}
