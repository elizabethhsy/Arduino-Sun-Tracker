#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial ss(A3, A2);
TinyGPSPlus gps;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ss.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (ss.available() > 0)
    gps.encode(ss.read());

  Serial.println(gps.time.value());

}
