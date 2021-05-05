#include "math.h"
#include <TimeLib.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
  int LT = hour(t); // local time (hr)
  int d = daysSinceStartOfYear(t); // days since the start of the year
 
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
  
  Serial.println(declination);
  Serial.println(elevation_deg);
  Serial.println(EoT);
  Serial.println(LST);
  Serial.println(radToDeg(HRA/pi));
  Serial.println(azimuth_deg);
  
}
