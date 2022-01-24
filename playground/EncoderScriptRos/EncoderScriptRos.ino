#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <Encoder.h>

#define leftDirPin 30;
#define leftPwmPin 28;
#define rightDirPin 31;
#define rightPwmPin 29;
#define interval 1000;

Encoder knobLeft(5, 6);
Encoder knobRight(7, 8);

unsigned long previousMillis = 0;
double positionLeft = 0;
double positionRight = 0;

ros::NodeHandle nodeHandle;

std_msgs::Int64 leftTicskMsg;
std_msgs::Int64 rightTicskMsg;
std_msgs::Float32 leftVelMsg;
std_msgs::Float32 rightVelMsg;
std_msgs::Int8 encoderResetMsg;

void leftCallback(const std_msgs::Int16& leftVelMsg) {
  int16_t vel = leftVelMsg.data;
  if (vel < 0) {
    digitalWrite(leftDirPin, LOW);
  }
  else {
    digitalWrite(leftDirPin, HIGH);
  }
  pwm = ??
  
  analogWrite(leftPwmPin, pwm);
}

void rightCallback(const std_msgs::Int16& rightVelMsg) {
  int16_t vel = rightVelMsg.data;
  if (vel < 0) {
    digitalWrite(rightDirPin, LOW);
  }
  else {
    digitalWrite(rightDirPin, HIGH);
  }
  pwm = ??
  
  analogWrite(rightPwmPin, pwm);
}

ros::Publisher leftTickPublisher("/Odometry/leftTicks", &leftTicksMessage);
ros::Publisher rightTickPublisher("/Odometry/rightTicks", &encoderMsg);
ros::Subscriber<std_msgs::Int16> leftSubscriber("leftVelocity", &leftVelMsg);
ros::Subscriber<std_msgs::Int16> rightSubscriber("rightVelocity", &rightVelMsg);
  
void setup() {
  pinMode(leftDirPin, OUTPUT);
  pinMode(leftPwmPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  pinMode(rightPwmPin, OUTPUT);

  nodeHandle.initNode();
  nodeHandle.advertise(encoder_publisher);
  nodeHandle.subscribe(left_subscriber);
  nodeHandle.subscribe(right_subscriber);
}

void loop() {
  // put your main code here, to run repeatedly:

}
