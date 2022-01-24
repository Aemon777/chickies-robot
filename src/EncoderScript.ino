#include <Encoder.h>

Encoder knobLeft(5, 6);
Encoder knobRight(7, 8);

unsigned long previousMillis = 0;

//1 second
const long interval = 100;

void setup() {
  Serial.begin(9600);
}

long long positionLeft  = 0;
long long positionRight = 0;

void loop() {
  //keep track of milliseconds
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    positionLeft = knobLeft.read();
    positionRight = knobRight.read();
    Serial.print(positionLeft);
    Serial.print(",");
    Serial.print(positionRight);
    Serial.println(",");
    // if a character is sent from the serial monitor,
    // reset both back to zero.
  }
}
