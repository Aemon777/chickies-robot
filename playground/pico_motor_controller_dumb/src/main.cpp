/*********************************
//Dynamic Protportional Motor Speed Controller for Chick-fil-a delivery robot
//Uses built-in encoders on wheel and PWM&direction style motor driver
//Designed to run on Teensy 4.0
//Author: Josh Blackburn
//Date Created: 2/10/2022
//Last Updated:
*********************************/

//Libraries to include
#include <Arduino.h>  //required in VsCode

//Pin Definitions
#define pwmPinL 2
#define dirPinL 3
#define pwmPinR 4
#define dirPinR 5
//Constants Definitions
#define loop_hz 50
#define serial_dead_time 3350

//Global Variables
unsigned long previousLoopTime = 0; //used to ensure feedback loop is run at loop_hz
unsigned long commTimer = 0; //used to stop robot if communication is lost for longer than serial_dead_time
double targetSpeedL = 0; //initial target speed
double targetSpeedR = 0;

int parseSerial() {
//Parses incoming serial commands
//message structure: | Velocity command: V,leftspeed,rightspeed,** | Reset Command: R,checksum
//velocities should be in millimeters/second   example message: "V,250,-500,**" sets rightspeed to 250mm/s and leftspeed to -500mm/s
  //read incoming serial data into a c string
  char incomingString[16];
  byte i = 0;
  while(Serial.available() > 0) {
    incomingString[i] = Serial.read();
    i++;
    delayMicroseconds(50); //prevent reading the serial data faster than it's arriving
  }
  Serial1.println(incomingString);
  //parse the c string
  char* pch;
  char* chrt[4];
  int count = 0;
  pch = strtok (incomingString,",");
  while (pch != NULL)  {
    chrt[count] = pch;
    pch = strtok (NULL, ",");
    count++;
  }
  //act based on the indicator char
  char indicator = chrt[0][0];
  if(indicator == 'V') {
    targetSpeedL = atof(chrt[1]);
    targetSpeedR = atof(chrt[2]);
  }
  else {
    return -1;
  }
  return 1; //return 1 if velocity command sucessfully parsed
}

void setup() {
  //serial used for USB communications to receive commands and report encoder counts
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(pwmPinL, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  analogWriteResolution(10); //10 bit gives 0-1023 range, higher resolution than the 256 of typical 8 bit
  analogWriteFrequency(pwmPinL, 146484.38); //frequency pulled from https://www.pjrc.com/teensy/td_pulse.html for 10 bit teensy 4.0 @ 600MHz
  pinMode(pwmPinR, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  analogWriteFrequency(pwmPinR, 146484.38);
  commTimer = millis();
  digitalWrite(dirPinL, 1);
  digitalWrite(dirPinR, 1);
}
void loop() {
  if(millis() - previousLoopTime > (1000/loop_hz)) {
    previousLoopTime = millis();
    if(millis() - commTimer > serial_dead_time) {
      targetSpeedL = targetSpeedR = 0;
    }
    if(Serial.available() > 0) {
      if(parseSerial() == 1) {
        commTimer = millis();
      }
    }
    static bool dirL = 1;
    if(targetSpeedL < 0) {
      dirL = 0;
    }
    else if(targetSpeedL > 0) {
      dirL = 1;
    }
    else {
      dirL = dirL;
    }
    int PWML_target = (int)(910.0*abs(targetSpeedL) + 45);
    static int PWML = 0;
    const int step = 30;
    if(PWML < PWML_target - step) {
      PWML += step;
    }
    else if(PWML > PWML_target + step) {
      PWML -= step;
    }
    else {
      PWML = PWML_target;
    }
    static bool dirR = 1;
    if(targetSpeedR < 0) {
      dirR = 0;
    }
    else if(targetSpeedR > 0) {
      dirR = 1;
    }
    else {
      dirR = dirR;
    }
    int PWMR_target = (int)(910.0*abs(targetSpeedR) + 45);
    static int PWMR = 0;
    if(PWMR < PWMR_target - step) {
      PWMR += step;
    }
    else if(PWMR > PWMR_target + step) {
      PWMR -= step;
    }
    else {
      PWMR = PWMR_target;
    }
    if(PWML < 50 && PWML_target < 50) {
      PWML = 0;
    }
    if(PWMR < 50 && PWMR_target < 50) {
      PWMR = 0;
    }
    
    digitalWrite(dirPinL, dirL);
    analogWrite(pwmPinL, PWML);
    digitalWrite(dirPinR, dirR);
    analogWrite(pwmPinR, PWMR);
  }
}//V,0.2,-0.25,**