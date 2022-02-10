/*********************************
//Weird hybrid Motor Speed Controller for Chick-fil-a delivery robot
//Uses built-in encoders on wheel and PWM&direction style motor driver
//Designed to run on Teensy 4.0
//Author: Josh Blackburn
//Date Created: 2/9/2022
//Last Updated:
*********************************/

//Libraries to include
#include <Arduino.h>  //required in VsCode
#include "QuadEncoder.h"
#include <SimpleKalmanFilter.h>

//Pin Definitions
#define ApinL 7
#define BpinL 8
#define pwmPinL 10
#define dirPinL 9
#define ApinR 2
#define BpinR 3
#define pwmPinR 12
#define dirPinR 11
//Constants Definitions
#define ticks_per_mm 138.00
#define serial_dead_time 550

#define left 1
#define right 0
#define forward 1
#define reverse 0

//enc(encoder number (1-4), A pin, B pin, pullups required (0/1), 4=??)
QuadEncoder encL(1, ApinL, BpinL, 1, 4);
QuadEncoder encR(2, ApinR, BpinR, 1, 4);
//Kalman filters smooth out noisy speed data from encoders
//SimpleKalmanFilter(Measurement Uncertaint16_ty, Estimation Uncertaint16_ty, Estimation Uncertaint16_ty)
SimpleKalmanFilter speedFilterL(0.25, 0.25, 0.01);
SimpleKalmanFilter speedFilterR(0.25, 0.25, 0.01);

//Global Variables
unsigned long previousLoopTime = 0;
unsigned long commTimer = 0;
int targetSpeedL = 0; //initial target speed
int targetSpeedR = 0;

bool setMotor(bool side, int pwm) {
  bool dir = 0;
  if(pwm == 1023 || pwm == 1024) {
    pwm = 0;
  }
  else if(pwm >= 0 && pwm < 1023) {
    pwm = 1023 - pwm;
    dir = 0;
  }
  else if(pwm > 1024 && pwm <= 2047) {
    pwm = pwm - 1024;
    dir = 1;
  }
  else {
    pwm = 0;
    digitalWrite(pwmPinL, 0);
    digitalWrite(pwmPinR, 0);
    return 0;
  }
  if(side) {
    digitalWrite(dirPinL, dir);
    analogWrite(pwmPinL, pwm);
  }
  else {
    digitalWrite(dirPinR, dir);
    analogWrite(pwmPinR, pwm);
  }
  return 1;
}
double calculateSpeed(long long c_ticks, long long &p_ticks, unsigned long c_time, unsigned long &p_time) {
  double mm = ((c_ticks - p_ticks) / ticks_per_mm);
  double mmps = (mm*1000) / double(c_time - p_time);
  p_time = c_time;
  p_ticks = c_ticks;
  return mmps;
}
void resetTicks() {
//Reset the encoder counts if for some reason it's necessary
  encL.write(0);
  encR.write(0);
}
int parseSerial() {
//Parses incoming serial commands
  //message structure: | Velocity command: V,leftspeed,rightspeed,** | Reset Command: R,checksum
  //velocities should be in meters/second   example message: "V,0.25,0.5,**" sets rightspeed to 0.25m/s and leftspeed to 0.5m/s
  char incomingString[16];
  byte i = 0;
  while(Serial.available() > 0) {
    incomingString[i] = Serial.read();
    i++;
    delayMicroseconds(25); //prevent reading the serial data faster than it's arriving
  }
  Serial.println(incomingString);
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
  if(indicator == 'R') {
    resetTicks();
    return 0;
  }
  else if(indicator == 'V') {
    targetSpeedL = atol(chrt[1]);
    targetSpeedR = atol(chrt[2]);
  }
  else {
    return -1;
  }
  return 1; //return 1 if velocity command sucessfully parsed
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);
  encL.setInitConfig();
  encL.init();
  encL.setInitConfig();
  encL.init();
  encR.setInitConfig();
  encR.init();
  pinMode(pwmPinL, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  analogWriteResolution(10); //10 bit gives 0-1023 range, higher resolution than the 256 of typical 8 bit
  analogWriteFrequency(pwmPinL, 146484.38); //frequency pulled from https://www.pjrc.com/teensy/td_pulse.html for 10 bit teensy 4.0 @ 600MHz
  pinMode(pwmPinR, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  analogWriteFrequency(pwmPinR, 146484.38);
  commTimer = millis();
}
void loop() {
  /*if(millis() > 5000)
    targetSpeedR = -200;
  if(millis() > 10000)
    targetSpeedR = 400;
  if(millis() > 15000)
    targetSpeedR = -600;
  if(millis() > 20000)
    targetSpeedR = 800;
  if(millis() > 25000)
    targetSpeedR = -1000;
  if(millis() > 30000)
    targetSpeedR = 1200;
  if(millis() > 35000)
    targetSpeedR = 0;//*/
  if(millis() - previousLoopTime > 20) {
    previousLoopTime = millis();
    if(millis() - commTimer > serial_dead_time) {
      //targetSpeedL = targetSpeedR = 0;
    }
    if(Serial.available() > 0) {
      if(parseSerial() == 1) {
        commTimer = millis();
      }
    }
    static double adjustL=0, adjustR=0;
    static unsigned long previousTimeL=0, previousTimeR=0;
    static long long previousTicksL=0, previousTicksR=0;
    double currentSpeedL = speedFilterL.updateEstimate(calculateSpeed(encL.read(), previousTicksL, millis(), previousTimeL));
    double currentSpeedErrorL = currentSpeedL - targetSpeedL;
    double currentSpeedR = speedFilterR.updateEstimate(calculateSpeed(encR.read(), previousTicksR, millis(), previousTimeR));
    double currentSpeedErrorR = currentSpeedR - targetSpeedR;

    adjustL = (abs(currentSpeedL)/12500.0 - 0.18) * currentSpeedErrorL;
    adjustL = constrain(adjustL, -100, 100);
    if(abs(adjustL) < 1) {
      adjustL = 0;
    }

    adjustR = (abs(currentSpeedR)/12500.0 - 0.18) * currentSpeedErrorR;
    adjustR = constrain(adjustR, -100, 100);
    if(abs(adjustR) < 1) {
      adjustR = 0;
    }
    static int PWML=1024, PWMR=1024;
    PWML += adjustL;
    PWML = constrain(PWML, 0, 2047);
    PWMR += adjustR;
    PWMR = constrain(PWMR, 0, 2047);
    setMotor(left, PWML);
    setMotor(right, PWMR);

    //Serial3.println(adjustL);
    /*Serial1.println("Target_Speed"+String(targetSpeedL)+
                    ",Current_Speed"+String(currentSpeedL)+
                    ",Speed_Error"+String(currentSpeedErrorL)+
                    ",PWM"+String(PWML));
    Serial1.print(targetSpeedL);
    Serial1.print(",");
    Serial1.print(currentSpeedL);
    Serial1.print(",");
    Serial1.print(currentSpeedErrorL);
    Serial1.print(",");
    Serial1.println(0);//*/
  }
}