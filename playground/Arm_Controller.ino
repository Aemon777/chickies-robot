#include <Servo.h>

Servo liftServo; //servo that controls the scissor lift
//Servo buttonServo; //servo that controls the button pushing mechanism

int liftServoInput = 9; //pin for the pwn input for the lift servo
//int buttonServoInput = 10; //pin for the pwm input for the button pushing servo

//Define switches, we should have 4
int switchPinLiftUp = 2; //pin for the limit switch that stops the lift from going too high
//int switchPinLiftDown = 2; //stops the lift from going too low
//int switchPinButtonForward = 7; //pin for limit switch that stops the button pushing device from going too far outward
//int switchPinButtonBack = 7; //stops button device from retracting too much

//I dont know if this is allowed???? Since I change the value of them later
//I am just concered that in the void loop, since it runs thousands of times a second, that if will keep redeclaring the values, and I need them to stay changed
 //inital states for the locations of the mechanisms 
//  int up = 1;
//  int down = 0;
//  int forward = 0;
//  int back = 0;
// complete = 0;

void setup() {
  //setup for the switches
  pinMode(switchPinLiftUp, INPUT_PULLUP); //we want input pullup to prevent strange readings
  //pinMode(switchPinButton, INPUT_PULLUP);
  //pinMode(switchPinButton, INPUT_PULLUP);
  //pinMode(switchPinButton, INPUT_PULLUP);

  //setup for the servos
  liftServo.attach(liftServoInput); //attaches the servo to the correct pint
  //buttonServo.attach(buttonServoInput); 
}

void loop() {
  //continually read the state of the switches
  int switchStateLiftUp = digitalRead(switchPinLiftUp);
  //int switchStateLiftDown = digitalRead(switchPinLiftDown);
 // int switchStateButtonForward = digitalRead(switchPinButtonForward);
 // int switchStateButtonBack = digitalRead(switchPinButtonBack);

//testing
if (switchStateLift == 0){
 // liftServo.write(90);
  //this makes servo go in other direction, we want to cut the power to the servo
  //digitalWrite(liftSeroPowerPin,LOW);
  liftServo.writeMicroseconds(1590);  // 1590 stops the lift servo without any additional jitters
  //buttonServo.writeMicroseconds(1590);
  //adjust above and below until you find it
}
else {
  liftServo.write(120);
}
//end testing


//Arm Controller 
  while(complete != 1){
    //extending the lift
    while (up == 1){
      if (switchStateLiftUp == 0){
        up = 0;
        forward = 1;
        liftServo.writeMicroseconds(1590);
      }
      else {
        liftServo.write(180);//need to check which direction this rotates
      }
    }
  
    //pressing the button
    while (forward == 1){
      if (switchStateButtonForward == 0){ //Need this if statement for checking if the LIDAR sees the door opening
        forward = 0;
        back = 1;
      } //Move that first if statement here and display and error message and retract everything
      else {
        buttonServo.write(180); //Im assuming 180 is the max speed for CW rotation
      }
    }
    
    //retracting the button mechanism 
    while (back == 1){
      if (switchStateButtonBack == 0){
        back = 0;
        down = 1;
        buttonServo.writeMicroseconds(1590);
      }
      else {
        buttonServo.write(0); //max speed for CCW rotation
      }
    }
  
    //compressing the lift
    while (down == 1){
      if (switchStateButtonBack == 0){
        down = 0;
        up = 1;
        complete = 1;
      }
      else {
        liftServo.write(0);
      }
    }
  }
}
