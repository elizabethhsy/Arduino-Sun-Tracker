#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);

  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  lcd.clear();
  lcd.print("Test");
  lcd.setCursor(0, 1);
  lcd.print("Second line");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(8) == LOW) {
    lcd.clear();
    lcd.print("Button pressed");
  }
}
