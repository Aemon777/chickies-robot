#include <Servo.h>

Servo liftServo; //servo that controls the scissor lift
Servo buttonServo; //servo that controls the button pushing mechanism

int liftServoPWMPin = 11; //pin for the pwn input for the lift servo
int buttonServoPWMPin = 10; //pin for the pwm input for the button pushing servo

//Define switches, we should have 4
int switchPinLiftUp = 2; //pin for the limit switch that stops the lift from going too high
int switchPinLiftDown = 8; //stops the lift from going too low
int switchPinButtonForward = 12; //pin for limit switch that stops the button pushing device from going too far outward
int switchPinButtonBack = 13; //stops button device from retracting too much

//I dont know if this is allowed???? Since I change the value of them later
//I am just concered that in the void loop, since it runs thousands of times a second, that if will keep redeclaring the values, and I need them to stay changed
 //inital states for the locations of the mechanisms 
//  int up = 0;
//  int down = 0;
//  int forward = 0;
//  int back = 0;
  int complete = 0;
//  int start = 1; //just a place-holder, this will probablly need to be an input or something from the pi/ROS

void setup() {
  //setup for the switches
  pinMode(switchPinLiftUp, INPUT_PULLUP); //we want input pullup to prevent strange readings
  pinMode(switchPinButtonForward, INPUT_PULLUP);
  pinMode(switchPinButtonBack, INPUT_PULLUP);
  pinMode(switchPinLiftDown, INPUT_PULLUP);

  //setup for the servos
  liftServo.attach(liftServoPWMPin); //attaches the servo to the correct pint
  buttonServo.attach(buttonServoPWMPin); 
  Serial.begin(9600);
}

void loop() {
  //continually read the state of the switches
  int switchStateLiftUp = digitalRead(switchPinLiftUp);
  int switchStateLiftDown = digitalRead(switchPinLiftDown);
  int switchStateButtonForward = digitalRead(switchPinButtonForward);
  int switchStateButtonBack = digitalRead(switchPinButtonBack);
  // might need to put these in the while loops

////testing
//if (switchStateLiftUp == 0){
// //liftServo.write(60);
//  //this makes servo go in other direction, we want to cut the power to the servo
//  //digitalWrite(liftSeroPowerPin,LOW);
//  liftServo.writeMicroseconds(1275);  // 1590 stops the lift servo without any additional jitters
//  //1275 stops the button servo without any additional jitters
//  //buttonServo.writeMicroseconds(1590);
//  //adjust above and below until you find it
//}
//else {
//  liftServo.write(0); //this makes the bottom servo turn in the direction to make the lift go up
//}
//end testing

int up = 0;
  int down = 0;
  int forward = 0;
  int back = 0;
  int complete = 0;
  int start = 1; //just a place-holder, this will probablly need to be an input or something from the pi/ROS

//Arm Controller 
// if (start == 1){
  while(complete != 1){
    //up=1;
    //extending the lift
//    while (up == 1){
//      int switchStateLiftUp = digitalRead(switchPinLiftUp);
//      if (switchStateLiftUp == 0){
//        up = 0;
//        down = 1;
//        liftServo.writeMicroseconds(1590);
//       // buttonServo.writeMicroseconds(1270); //button servo should not be moving for up or down part
//      }
//      else {
//        liftServo.write(0);//need to check which direction this rotates
//        //buttonServo.writeMicroseconds(1270);
//      }
//    }
  
    //pressing the button
    forward = 1;
    while (forward == 1){
      int switchStateButtonForward = digitalRead(switchPinButtonForward);
      if (switchStateButtonForward == 0){ //Need this if statement for checking if the LIDAR sees the door opening, switch starts pushed
        buttonServo.writeMicroseconds(1270);
        //liftServo.writeMicroseconds(1590); //lift servo should not be moving for forward or back part
        forward = 0;
        back = 1;
        Serial.print("Done ");
      } 
      else {
        buttonServo.write(0); //Im assuming 180 is the max speed for CW rotation
        //liftServo.writeMicroseconds(1590);
      }
    }
    
    //retracting the button mechanism 
//    while (back == 1){
//      int switchStateButtonBack = digitalRead(switchPinButtonBack);
//      if (switchStateButtonBack == 0){
//        back = 0;
//        down = 1;
//        buttonServo.writeMicroseconds(1270);
//        liftServo.writeMicroseconds(1590);
//      }
//      else {
//        buttonServo.write(180); //max speed for CCW rotation
//        liftServo.writeMicroseconds(1590);
//      }
//    }
  
    //compressing the lift
    while (down == 1){
      int switchStateLiftDown = digitalRead(switchPinLiftDown);
      if (switchStateLiftDown == 0){
        down = 0;
        up = 1;
        complete = 1;
        liftServo.writeMicroseconds(1590);
        buttonServo.writeMicroseconds(1270);
        //add something to cut power to the arduino
      }
      else {
        liftServo.write(180);
        buttonServo.writeMicroseconds(1270);
      }
    }
  }
//}
}
