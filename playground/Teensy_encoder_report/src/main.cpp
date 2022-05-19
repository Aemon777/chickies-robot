/*********************************
//Encoder Feedback for ROS
//Designed to run on Teensy 4.0
//Author: Josh Blackburn
//Date Created: 2/10/2022
//Last Updated:
*********************************/

//Libraries to include
#include <Arduino.h>  //required in VsCode
#include "QuadEncoder.h"

//Pin Definitions
#define ApinL 7 //Left encoder A phase
#define BpinL 8 //Left encoder B phase
#define ApinR 2 //you get it
#define BpinR 3
#define buzzerPin 19 //Pin controlling buzzer
#define greenPin 20 //should be obvious
#define yellowPin 21
#define redPin 22
//Constants
#define blinkPeriod 500 //controls how fast the LED blinks when robot is in auto mode

//enc(encoder number (1-4), A pin, B pin, pullups required (0/1), 4=??) //IDK what the 4 does, but it seems necessary...
QuadEncoder encL(1, ApinL, BpinL, 1, 4);
QuadEncoder encR(2, ApinR, BpinR, 1, 4);

void printEncoders() {
  //print the encoder counts with format "E,leftCount,rightCount,**"
  Serial.println("E,"+String(encL.read())+","+String(encR.read())+",**");
  Serial1.println("E,"+String(encL.read())+","+String(encR.read())+",**");
}
//Q,**
int parseSerial() {
  //this is C string stuff, it's confusing -_-
  char incomingString[16];
  byte i = 0;
  while(Serial.available() > 0) {
    incomingString[i] = Serial.read();
    i++;
    delayMicroseconds(50);
  }
  char* pch;
  char* chrt[4];
  int count = 0;
  pch = strtok (incomingString,",");
  while (pch != NULL)  {
    chrt[count] = pch;
    pch = strtok (NULL, ",");
    count++;
  }
  //return a number based on the received message
  char indicator = chrt[0][0];
  if(indicator == 'Q') {
    return 1;
  }
  else if(indicator == 'B') {
    if(atoi(chrt[1]))
      return 2;
    else
      return 3;
  }
  else if(indicator == 'G') {
    if(atoi(chrt[1]))
      return 4;
    else
      return 5;
  }
  else if(indicator == 'Y') {
    if(atoi(chrt[1]))
      return 6;
    else
      return 7;
  }
  else if(indicator == 'R') {
    if(atoi(chrt[1]))
      return 8;
    else
      return 9;
  }
  else {
    return 0; //This should never happen, it means an invalid message was received
  }
}

void setup() {
  pinMode(greenPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  //serial used for USB communications to receive commands and report encoder counts
  Serial.begin(115200);
  //serial 1 used for debugging and tracking stats
  Serial1.begin(115200);
  encL.setInitConfig();
  encL.init();
  encR.setInitConfig();
  encR.init();
}

void loop() {
  static bool blink = false; 
  while(!Serial.available()) {
    //blink the red light without using delays
    static unsigned long startTime = millis();
    if(blink && millis() - startTime >= blinkPeriod) {
      startTime = millis();
      static bool ledState = false;
      ledState = !ledState;
      digitalWrite(redPin, ledState);
    }
  }
  int sig = parseSerial();
  switch (sig) {
  case 0:
    Serial.println("ERROR");
    break;
  case 1:
    printEncoders();
    break;
  case 2:
    digitalWrite(buzzerPin, HIGH);
    break;
  case 3:
    digitalWrite(buzzerPin, LOW);
    break;
  case 4:
    digitalWrite(greenPin, HIGH);
    break;
  case 5:
    digitalWrite(greenPin, LOW);
    break;
  case 6:
    digitalWrite(yellowPin, HIGH);
    break;
  case 7:
    digitalWrite(yellowPin, LOW);
    break;
  case 8:
    blink = true;
    break;
  case 9:
    digitalWrite(redPin, HIGH);
    break;
  default:
    break;
  }
}