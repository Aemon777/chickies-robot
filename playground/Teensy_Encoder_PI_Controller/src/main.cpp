/*********************************
//PID Motor Speed Controller for Chick-fil-a delivery robot
//Uses built-in encoders on wheel and PWM&direction style motor driver
//Designed to run on Teensy 4.0
//Author: Josh Blackburn
//Date Created: 2/2/2022
//Last Updated:
*********************************/

//Libraries to include
#include <Arduino.h>  //required in VsCode
#include "QuadEncoder.h"
#include <SimpleKalmanFilter.h>
#include <AutoPID.h>

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
//PID Controller Parameters
#define OUTPUT_MIN 0
#define OUTPUT_MAX 1023
#define KP 6//10.0
#define KI 11.0
#define KD 1.0

//enc(encoder number (1-4), A pin, B pin, pullups required (0/1), 4=??)
QuadEncoder encL(1, ApinL, BpinL, 1, 4);
QuadEncoder encR(2, ApinR, BpinR, 1, 4);
//Kalman filters smooth out noisy speed data from encoders
//SimpleKalmanFilter(Measurement Uncertaint16_ty, Estimation Uncertaint16_ty, Estimation Uncertaint16_ty)
SimpleKalmanFilter speedFilterL(10, 10, 0.01);
SimpleKalmanFilter speedFilterR(10, 10, 0.01);

//Global Variables
long previousTicksL = 0;
long previousTicksR = 0;
unsigned long previousTimeL = 0;
unsigned long previousTimeR = 0;
unsigned long previousLoopTime = 0;
int PWML = 0;
//double targetSpeedML = 0.0;
long targetSpeedL = 0; // = targetSpeedML * 1000;
double inputL;
double outputL;
double setPointL;
int PWMR = 0;
//double targetSpeedMR = 0.0;
long targetSpeedR = 0; //targetSpeedMR * 1000;
double inputR;
double outputR;
double setPointR;
unsigned long commTimer = 0;

AutoPID myPIDL(&inputL, &setPointL, &outputL, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID myPIDR(&inputR, &setPointR, &outputR, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void printStuff(long TSL, long FSL, long OPL, int PML, long TSR, long FSR, long OPR, int PMR) {
//This prints all relevant data over Serial1 UART channel (pins RX-0, TX-1) in a format that the Arduino Serial Plotter likes
  static uint8_t cnt = 0;
    if(cnt == 20) {
      Serial1.println("TargetSpeed_L_"+String(TSL)+",Speed_L_"+String(FSL)+",Output_L_"+String(OPL)+",PWM_L_"+String(PML)+"TargetSpeed_R_"+String(TSR)+",Speed_R_"+String(FSR)+",Output_R_"+String(OPR)+",PWM_R_"+String(PMR));
      Serial1.print(TSL);
      Serial1.print(",");
      Serial1.print(FSL);
      Serial1.print(",");
      Serial1.print(OPL);
      Serial1.print(",");
      Serial1.print(PML);
      Serial1.print(",");
      Serial1.print(TSR);
      Serial1.print(",");
      Serial1.print(FSR);
      Serial1.print(",");
      Serial1.print(OPR);
      Serial1.print(",");
      Serial1.println(PMR);
      cnt = 0;
    }
    cnt++;
}
void resetTicks() {
//Reset the encoder counts if for some reason it's necessary
  encL.write(0);
  encR.write(0);
}
long calculateSpeed(long c_ticks, long &p_ticks, unsigned long c_time, unsigned long &p_time) {
//Calculate the current speed in millimeters/second from the encoder counts
  long um = ((c_ticks - p_ticks)*1000 / ticks_per_mm); //distance travelled during last cycle in um
  long umps = (um*1000) / long(c_time - p_time); //average speed during last cycle in um/s
  /*Serial1.print(c_time - p_time);
  Serial1.print("  ");
  Serial1.print(um);
  Serial1.print(" ");
  Serial1.println(umps/1000);*/
  p_time = c_time;
  p_ticks = c_ticks;
  return umps/1000; // mm/second
}
static inline int8_t sgn(int val) {
  if(val < 0) return -1;
  if(val == 0) return 0;
  return 1;
}
static inline int8_t sgn(long val) {
  if(val < 0) return -1;
  if(val == 0) return 0;
  return 1;
}
int parseSerial() {
  //Serial.println("TESTTTT");
//Parses incoming serial commands
  //message structure: | Velocity command: V,leftspeed,rightspeed,** | Reset Command: R,checksum
  //velocities should be in meters/second   example message: "V,0.25,0.5,**" sets rightspeed to 0.25m/s and leftspeed to 0.5m/s
  char incomingString[16];
  byte i = 0;
  while(Serial.available() > 0) {
    //char c = Serial.read();
    incomingString[i] = Serial.read();
    i++;
    delayMicroseconds(25); //prevent reading the serial data faster than it's arriving
  }
  Serial.println(incomingString);
  char* pch;
  char* chrt[4];
  int count = 0;
  //char CS[2] = {0};
  pch = strtok (incomingString,",");
  while (pch != NULL)  {
    chrt[count] = pch;
    pch = strtok (NULL, ",");
    count++;
    //CS += chrt[count]; //checksum not currently used
  }
  char indicator = chrt[0][0];
  if(indicator == 'R') {
    resetTicks();
    return 0;
  }
  else if(indicator == 'V') {
    targetSpeedL = atol(chrt[1]);
    targetSpeedR = atol(chrt[2]);
    //byte checksum = chrt[3][0];
    //Serial1.println("adjusted: " + String(targetSpeedL));
  }
  else {
    return -1;
  }
  return 1; //return 1 if velocity command sucessfully parsed
}
/*void setMotors(int8_t targetDirection, int8_t currentDirection, long loutput, int dirPin, int pwmPin, int &PWM) {
  bool dontReverse = false;
  if((targetDirection == 1 && currentDirection == -1) || (targetDirection == -1 && currentDirection == 1)) {
    loutput = 0;
    dontReverse = true;
  }

  //prevent PWM from changing by more than 6 per 1mS loop, jumping from 0 to 1023 can cause mechanical damage to the robot
  uint8_t step = 6;
  if(PWM < loutput - 20*step) { //ok to jump straight to output level if PWM is within 20*6=120 of output
    PWM += step;
  }
  else if(PWM > loutput + 20*step) {
    PWM -= step;
  }
  else {
    PWM = loutput;
  }

  PWM = constrain(PWM, 0, 1023); //don't try to go over 1023
  
  //set the motor direction, only change if 0 speed state has been reached
  if(targetDirection == 1 && !dontReverse) {
    digitalWrite(dirPin, HIGH);
  }
  else if(targetDirection == -1 && !dontReverse) {
    digitalWrite(dirPin, LOW);
  }
  else {}

  analogWrite(pwmPin, PWM);
}*/
void setup() {
  Serial.begin(115200); //USB serial
  Serial1.begin(115200);//pins 0,1 serial
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

  myPIDL.setBangBang(1000); //set really high so bang-bang never activates
  myPIDL.setTimeStep(5);    //time step set to 1mS so PID runs at 1KHz
  myPIDR.setBangBang(1000); //set really high so bang-bang never activates
  myPIDR.setTimeStep(5);
  commTimer = millis();
}
void loop() {
  myPIDL.run();
  myPIDR.run();
  if(micros() - previousLoopTime > 5000) { //use timer instead of delay to run loop at 200Hz
    previousLoopTime = micros();
    static uint8_t count = 0;
    count++;
    long leftTicks = encL.read();
    long rightTicks = encR.read();
    
    if(millis() - commTimer > serial_dead_time) {
      Serial.println("Communication Lost");
      targetSpeedL = targetSpeedR = 0;
    }
    if(Serial.available() > 0) {
      if(parseSerial() == 1) {
        commTimer = millis();
      }
    }
  
    setPointL = double(abs(targetSpeedL)); //PID only handles positive velocities
    setPointR = double(abs(targetSpeedR));
    int8_t targetDirectionL = sgn(targetSpeedL); //speed and direction are handled separately
    int8_t targetDirectionR = sgn(targetSpeedR);
    //long speedL = calculateSpeed(leftTicks, previousTicksL, millis(), previousTimeL);
    //long speedR = calculateSpeed(rightTicks, previousTicksR, millis(), previousTimeR);
    long filteredSpeedL = speedFilterL.updateEstimate(calculateSpeed(leftTicks, previousTicksL, millis(), previousTimeL)); //kalman filter smooths speed data
    long filteredSpeedR = speedFilterR.updateEstimate(calculateSpeed(rightTicks, previousTicksR, millis(), previousTimeR));
    int8_t currentDirectionL = sgn(filteredSpeedL);
    int8_t currentDirectionR = sgn(filteredSpeedR);
    inputL = double(abs(filteredSpeedL));
    inputR = double(abs(filteredSpeedR));
    long loutputL = long(outputL);
    long loutputR = long(outputR);
    //prevent wheel from going directly forwards to backward, instead going through a 0 speed phase
    bool dontReverseL = false;
    bool dontReverseR = false;
    if((targetDirectionL == 1 && currentDirectionL == -1) || (targetDirectionL == -1 && currentDirectionL == 1)) {
      loutputL = 0;
      dontReverseL = true;
    }
    if((targetDirectionR == 1 && currentDirectionR == -1) || (targetDirectionR == -1 && currentDirectionR == 1)) {
      loutputR = 0;
      dontReverseR = true;
    }

    //prevent PWM from changing by more than 6 per 1mS loop, jumping from 0 to 1023 can cause mechanical damage to the robot
    uint8_t step = 6;
    if(PWML < loutputL - 20*step) { //ok to jump straight to output level if PWM is within 20*6=120 of output
      PWML += step;
    }
    else if(PWML > loutputL + 20*step) {
      PWML -= step;
    }
    else {
      PWML = loutputL;
    }
    if(PWMR < loutputR - 20*step) {
      PWMR += step;
    }
    else if(PWMR > loutputR + 20*step) {
      PWMR -= step;
    }
    else {
      PWMR = loutputR;
    }

    PWML = constrain(PWML, 0, 1023); //don't try to go over 1023
    PWMR = constrain(PWMR, 0, 1023);
    
    //set the motor direction, only change if 0 speed state has been reached
    if(targetDirectionL == 1 && !dontReverseL) {
      digitalWrite(dirPinL, HIGH);
    }
    else if(targetDirectionL == -1 && !dontReverseL) {
      digitalWrite(dirPinL, LOW);
    }
    else {}
    
    if(targetDirectionR == 1 && !dontReverseR) {
      digitalWrite(dirPinR, HIGH);
    }
    else if(targetDirectionR == -1 && !dontReverseR) {
      digitalWrite(dirPinR, LOW);
    }
    else {}

    analogWrite(pwmPinL, PWML);
    analogWrite(pwmPinR, PWMR);

    if(count > 20) { //run at 10Hz
      //print encoder counts for ROS odometry
      Serial.print("E,");
      Serial.print(leftTicks);
      Serial.print(",");
      Serial.print(rightTicks);
      Serial.println(",**");
      count = 0;
    }
    printStuff(targetSpeedL,filteredSpeedL,loutputL,PWML,targetSpeedR,filteredSpeedR,loutputR,PWMR);
  }
}