#include "math.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // CONST
  float pi = 3.14159265359;
  
  // values to input
  float latitude = 51.500150;
  float latitude_rad = latitude/180*pi;
  float longitude = 1;
  int time_difference = 1;
  int LT = 13; // local time (hr)
  int d = 119; // days since the start of the year
 
  // calculating the angles using https://www.pveducation.org/pvcdrom/properties-of-sunlight/the-suns-position
  int lstm = 15*time_difference;
  float B = (2*pi)/365*(d - 81); // angle in radians
  float EoT = 9.87*sin(2*B)-7.53*cos(B)-1.5*sin(B); // equation of time
  float TC = 4*(longitude - lstm) + EoT; // time correction
  float LST = LT + TC/60; // local solar time
  float HRA = 15*(LST-12)/180*pi; // hour angle (rad)
  float declination = 23.45*sin(B); // declination angle
  float declination_rad = declination/180*pi; // declination angle (rad)
  float elevation_rad = asin(sin(declination_rad)*sin(latitude_rad) + cos(declination_rad)*cos(latitude_rad)*cos(HRA)); // elevation (rad)
  float elevation_deg = elevation_rad/pi*180;
  float azimuth = acos((sin(declination_rad)*cos(latitude_rad) - cos(declination_rad)*sin(latitude_rad)*cos(HRA))/cos(elevation_rad));
  float azimuth_deg = azimuth/pi*180;
  
  Serial.println(declination);
  Serial.println(elevation_deg);
  Serial.println(EoT);
  Serial.println(LST);
  Serial.println(HRA/pi*180);
  Serial.println(azimuth_deg);
  
}
