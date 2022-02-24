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
//#include <SimpleKalmanFilter.h>

//Pin Definitions
#define ApinL 7
#define BpinL 8
#define ApinR 2
#define BpinR 3
//Constants Definitions
#define report_hz 20

//enc(encoder number (1-4), A pin, B pin, pullups required (0/1), 4=??)
QuadEncoder encL(1, ApinL, BpinL, 1, 4);
QuadEncoder encR(2, ApinR, BpinR, 1, 4);

unsigned long reportTimer = 0; //used to ensure encoder count reporting at report_hz
void printEncoders() {
  //print the encoder counts with format "E,leftCount,rightCount,**"
  Serial.println("E,"+String(encL.read())+","+String(encR.read())+",**");
}

void setup() {
  //serial used for USB communications to receive commands and report encoder counts
  Serial.begin(115200);
  //serial 1 & 3 used for debugging and tracking stats
  //Serial1.begin(115200);
  //Serial3.begin(115200);
  encL.setInitConfig();
  encL.init();
  encR.setInitConfig();
  encR.init();
}
void loop() {
  if(millis() - reportTimer > (1000/report_hz)) {
    reportTimer = millis();
    printEncoders();
  }
}//V,0.2,-0.25,**