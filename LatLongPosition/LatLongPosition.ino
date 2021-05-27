/* 
Arduino Sun Tracker by Elizabth Ho and Niklas Vainio
Started on 22/4/21, completed on 27/5/21

This is the code for a device to track the Sun through the sky, using a motorised needle.
We are using GPS data for time information, a stepper motor for horizontal rotation and a servo motor for vertical rotation.
The stepper motor has no absolute position data, so must be manually calibrated to point north at the start of the program.
The formulas used for the actual tracking of the sun come from (https://www.sunearthtools.com/dp/tools/pos_sun.php)

The Stepper is driven using an A4988 stepper motor driver.
*/

// Including required libraries
// PWM Servo is used for servo control, and math for operations (sin, cosine etc)
// SoftwareSerial and TinyGps++ are used to read data from the GPS module
// LiquidCrystal is used to control the LCD screen
#include <PWMServo.h>
#include "math.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal.h>

// Defining constants for Arduino Pins
#define STEP_PIN 11
#define DIR_PIN 12
#define EN_PIN A0
#define SERVO_PIN 10

// Initialising objects to control components
SoftwareSerial ss(A3, A2);
TinyGPSPlus gps;
PWMServo myservo;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

// Store the current angle of the stepper motor, as it has no position feedback
float stepper_angle = 0;

void setup() {
  // Begin serial communication with computer and GPS module
  Serial.begin(9600);
  ss.begin(9600);

  // Defining all pins for stepper motor driver as outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // Sets enable pin high to release power from the stepper coils
  digitalWrite(EN_PIN, HIGH);
   
  // Configured the servo object to use pin 10
  myservo.attach(SERVO_PIN);  
  // Initialise LCD - 16 columns, 2 rows
  lcd.begin(16, 2);

  // Configure both buttons as inputs with internal pullup resistors
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  // Tell user to point needle to north and wait until button pressed
  lcd.clear();
  lcd.print("Calibrate to");
  lcd.setCursor(0, 1);
  lcd.print("North");

  while(digitalRead(8) != LOW) {
    delay(10);
  }

  // Re-energise stepper coils and enter main program
  digitalWrite(EN_PIN, LOW);
  lcd.clear();
}

// Function to move the servo to a specific angle above the horizontal
void setServoAngle(float angle) {
  // On the servo, 0 is vertical and 90 is horizontal, so 90 - angle is used
  float safe_angle = 90.0 - angle;

  // Constrain angle between 0 and 140 to prevent needle hitting the table
  if(safe_angle < 0) {
    safe_angle = 0;
  }
  
  if(safe_angle > 140) {
    safe_angle = 140;
  }

  // Write angle to the servo
  myservo.write((int) safe_angle);
}

// Function to set the stepper to a specific angle
void move_stepper_to_angle(float angle) {
  // Calculate how much the stepper needs to move from its current position
  float angle_diff = angle - stepper_angle;

  // Work out whether stepper needs to move clockwise or anticlockwise and set DIR pin accordingly
  if(angle_diff > 0) {
    digitalWrite(DIR_PIN, LOW);
  } else {
    digitalWrite(DIR_PIN, HIGH);
  }

  // Calculate how many steps (each one 0.1125)
  int num_steps = (int)(abs(angle_diff / 0.1125));

  // Execute steps and update stepper position
  for (int i = 0; i < num_steps; i ++) {
    digitalWrite(STEP_PIN, HIGH);
    delay(2);
    digitalWrite(STEP_PIN, LOW);
    delay(2);
    if(angle_diff < 0) {
      stepper_angle -= 0.1125;
    } else {
      stepper_angle += 0.1125;
    }
  }
  
}

// Helper functions to convert between degrees and radians
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

// Functions to calculate the number of days since the start of the year - needed for dun tracking
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

// Main program loop
void loop() {
  // Check for any data from the GPS, and decode it
  while (ss.available() > 0)
    gps.encode(ss.read());


  // Wait until time data received from the GPS
  if (gps.time.value() != 0) {
  float pi = 3.14159265359;

  float LT = gps.time.hour() + (float)gps.time.minute()/60.0;
  
  // Latitude and Longitude hardcoded to be accurate for London (could be automated with better GPS signal)
  float latitude = 51.495385;
  float longitude = -0.127149;
  int time_difference = 1;
  int d = daysSinceStartOfYear(); // days since the start of the year
 
  // Calculating the angles using formulae from https://www.pveducation.org/pvcdrom/properties-of-sunlight/the-suns-position
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

  // Elevation_deg and azimuth_deg are the angles to point the needle
  // Correct errors from arccos function if time is past midday
  if(LT > 12) {
    azimuth_deg = 360 - azimuth_deg;
  } 

  // Print statements for debugging
  
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

  // Move servo and stepper to desired angle
  setServoAngle(elevation_deg);
  delay(2000);
  move_stepper_to_angle(azimuth_deg);

  // Print angle data and the time to the LCD
  lcd.clear();
  lcd.print("Azimuth: " + (String) azimuth_deg);
  lcd.setCursor(0, 1);
  lcd.print("Elevation: " + (String) elevation_deg);

  delay(5000);
  lcd.clear();
  lcd.print("Time: " + (String) (gps.time.hour() + time_difference) + ":" + (String) gps.time.minute());

  delay(3000);
  
  } else {

    // If GPS time not found, notify the user on the LCD screen
    lcd.clear();
    lcd.print("GPS can't");
    lcd.setCursor(0, 1);
    lcd.print("find time");
    delay(2000);
  }
}
