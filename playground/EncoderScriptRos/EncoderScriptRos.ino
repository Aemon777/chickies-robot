#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <Encoder.h>

#define leftDirPin 30;
#define leftPwmPin 28;
#define rightDirPin 31;
#define rightPwmPin 29;

Encoder knobLeft(5, 6);
Encoder knobRight(7, 8);

ros::NodeHandle nodeHandle;

std_msgs::String encoderMsg;
std_msgs::Int16 leftWheelMsg;
std_msgs::Int16 rightWheelMsg;

void leftCallback(const std_msgs::Int16& leftWheelMsg) {
  int16_t leftPwm = leftWheelMsg.data;
  if (leftPwm < 0) {
    digitalWrite(leftDirPin, LOW);
  }
  else {
    digitalWrite(leftDirPin, HIGH);
  }
  leftMag = (int) abs(leftPwm)*255/100;
  
  analogWrite(leftPwmPin, leftMag);
}

void rightCallback(const std_msgs::Int16& rightWheelMsg) {
  int16_t rightPwm = rightWheelMsg.data;
  if (rightPwm < 0) {
    digitalWrite(rightDirPin, LOW);
  }
  else {
    digitalWrite(rightDirPin, HIGH);
  }
  rightMag = (int) abs(rightPwm)*255/100;
  
  analogWrite(rightPwmPin, rightMag);
}
  
void setup() {
  //

}

void loop() {
  // put your main code here, to run repeatedly:

}
