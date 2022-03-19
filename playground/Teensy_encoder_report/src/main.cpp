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
#define ApinL 7
#define BpinL 8
#define ApinR 2
#define BpinR 3
//Constants Definitions
#define report_hz 10

//enc(encoder number (1-4), A pin, B pin, pullups required (0/1), 4=??)
QuadEncoder encL(1, ApinL, BpinL, 1, 4);
QuadEncoder encR(2, ApinR, BpinR, 1, 4);

unsigned long reportTimer = 0; //used to ensure encoder count reporting at report_hz
void printEncoders() {
  //print the encoder counts with format "E,leftCount,rightCount,**"
  Serial.println("E,"+String(encL.read())+","+String(encR.read())+",**");
  Serial1.println("E,"+String(encL.read())+","+String(encR.read())+",**");
}

bool parseSerial() {
  char incomingString[16];
  byte i = 0;
  while(Serial.available() > 0) {
    incomingString[i] = Serial.read();
    i++;
    delayMicroseconds(50);
  }//Q,**
  //Serial1.println(incomingString);
  char* pch;
  char* chrt[4];
  int count = 0;
  pch = strtok (incomingString,",");
  while (pch != NULL)  {
    chrt[count] = pch;
    pch = strtok (NULL, ",");
    count++;
  }
  char indicator = chrt[0][0];
  if(indicator == 'Q') {
    return true;
  }
  else {
    return false;
  }
}//*/

void setup() {
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
  while(!Serial.available());
  if(parseSerial()) {
    printEncoders();
  }
}