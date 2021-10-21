#include "SevSeg.h"
SevSeg sevseg; //Instantiate a seven segment controller object

void setup() {
  pinMode(A6, INPUT);
  pinMode(10, OUTPUT);
  byte numDigits = 4;
  byte digitPins[] = {A2,A3,3,4}; //Digits: 1,2,3,4 <--put one resistor (ex: 220 Ohms, or 330 Ohms, etc, on each digit pin)
  byte segmentPins[] = {8, A0, 6, A1, 23, 7, 5, 22}; //Segments: A,B,C,D,E,F,G,Period
  bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_ANODE; // See README.md for options
  bool updateWithDelays = false; // Default 'false' is Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected
  
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
  updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(10);
}

void loop() {
  static unsigned long timer = millis();

  if (millis() - timer >= 3000) {
    timer += 3000;
    float val = analogRead(A6);
    float millivolts = val * 4.8875855;
    millivolts *= 3.67857;
    sevseg.setNumber(millivolts/10, 2);
    if(millivolts < 10000) {
      tone(10, 600, 100);
    }
  }

  sevseg.refreshDisplay(); // Must run repeatedly
}
