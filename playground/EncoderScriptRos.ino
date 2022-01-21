#include <Encoder.h>

Encoder knobLeft(5, 6);
Encoder knobRight(7, 8);

unsigned long previousMillis = 0;

//1 second
const long interval = 1000;

void setup() {
  Serial.begin(9600);
}

double positionLeft  = 0;
double positionRight = 0;

void loop() {
  //keep track of milliseconds
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    double newLeft, newRight;
    //divide by 215 for converting to centimeters
    newLeft = knobLeft.read() / 215.00;
    newRight = knobRight.read() / 215.00;
    if (newLeft != positionLeft || newRight != positionRight) {
      Serial.print("Left = ");
      Serial.print(newLeft);
      Serial.print(", Right = ");
      Serial.print(newRight);
      Serial.println();
      positionLeft = newLeft;
      positionRight = newRight;
    }
    // if a character is sent from the serial monitor,
    // reset both back to zero.
    if (Serial.available()) {
      Serial.read();
      Serial.println("Reset both knobs to zero");
      knobLeft.write(0);
      knobRight.write(0);
    }
  }
}
