#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2);

#define LEFTDIR 9
#define LEFTPWM 10
#define RIGHTDIR 11
#define RIGHTPWM 12

#define TICKSPERMETER 138000
#define MAXSPEED 1.35
#define MAXPWM 255

Encoder knobLeft(7, 8);
long encoderLeft = 0;
Encoder knobRight(5, 6);
long encoderRight = 0;

long leftTicks = 0;
long rightTicks = 0;
unsigned long leftMillis = millis();
unsigned long rightMillis = millis();
float leftCo = MAXPWM/MAXSPEED;
float rightCo = MAXPWM/MAXSPEED;
int curLeftPwm = 0;
int curRightPwm = 0;

ros::NodeHandle nodeHandler;

std_msgs::Int64 leftTicksMsg;
std_msgs::Int64 rightTicksMsg;
std_msgs::Float32 leftVelMsg;
std_msgs::Float32 rightVelMsg;
std_msgs::Int8 encoderResetMsg;

void leftCallback( const std_msgs::Float32& leftVelMsg ) {
  // Update left side coefficient using travel / time and old pwm
  mySerial.println("In leftCallback with msg.data: " + String(leftVelMsg.data));
  long newTicks = knobLeft.read();
  unsigned long newMillis = millis();
  if (curLeftPwm > 15) {
    float curSpeed = (newTicks - leftTicks)/(newMillis - leftMillis);
    curSpeed = curSpeed / TICKSPERMETER * 1000;
    leftCo = (float) curLeftPwm / curSpeed;
  }
  leftTicks = newTicks;
  leftMillis = newMillis;
  //Convert leftVelMsg.data to direction and vel
  float vel = leftVelMsg.data;
  if (vel < 0) {
    digitalWrite(LEFTDIR, LOW);
  }
  else {
    digitalWrite(LEFTDIR, HIGH);
  }
  vel = abs(vel);
  curLeftPwm = round(vel*leftCo);
  if (curLeftPwm > 255) {
    curLeftPwm = 255;
  }
  analogWrite(LEFTPWM, curLeftPwm);
}

void rightCallback( const std_msgs::Float32& rightVelMsg ) {
  // Update right side coefficient using travel / time and old pwm
  mySerial.println("In rightCallback with msg.data: " + String(rightVelMsg.data));
  long newTicks = knobLeft.read();
  unsigned long newMillis = millis();
  if (curRightPwm > 15) {
    float curSpeed = (newTicks - rightTicks)/(newMillis - rightMillis);
    curSpeed = curSpeed / TICKSPERMETER * 1000;
    rightCo = (float) curRightPwm / curSpeed;
  }
  rightTicks = newTicks;
  rightMillis = newMillis;
  
  //Convert leftVelMsg.data to direction and vel
  float vel = rightVelMsg.data;
  if (vel < 0) {
    digitalWrite(RIGHTDIR, LOW);
  }
  else {
    digitalWrite(RIGHTDIR, HIGH);
  }
  vel = abs(vel);
  curRightPwm = round(vel*rightCo);
  if (curRightPwm > 255) {
    curRightPwm = 255;
  }
  analogWrite(RIGHTPWM, curRightPwm);
}

void resetCallback( const std_msgs::Int8& encoderResetMsg ) {
  mySerial.println("In Reset");
  encoderLeft = 0;
  encoderRight = 0;
}

//ros::Publisher publisher("topic", msg);
//ros::Subscriber<namespace::type> subscriber(topic, msg)

ros::Publisher leftTicksPublisher("Odometry/leftTicks", &leftTicksMsg);
ros::Publisher rightTicksPublisher("Odometry/rightTicks", &rightTicksMsg);
ros::Subscriber<std_msgs::Float32, void> leftVelSubscriber("leftVelocity", &leftCallback);
ros::Subscriber<std_msgs::Float32> rightVelSubscriber("rightVelocity", &rightCallback);
ros::Subscriber<std_msgs::Int8> encoderResetSubscriber("encoderReset", &resetCallback);


void setup() {
  // put your setup code here, to run once:
  pinMode(LEFTDIR, OUTPUT);
  pinMode(LEFTPWM, OUTPUT);
  pinMode(RIGHTDIR, OUTPUT);
  pinMode(RIGHTPWM, OUTPUT);

  mySerial.begin(9600);
  
  nodeHandler.initNode();
  nodeHandler.advertise(leftTicksPublisher);
  nodeHandler.advertise(rightTicksPublisher);
  nodeHandler.subscribe(leftVelSubscriber);
  nodeHandler.subscribe(rightVelSubscriber);
  nodeHandler.subscribe(encoderResetSubscriber);

  encoderLeft = knobLeft.read();
  encoderRight = knobRight.read();
}

void loop() {
  for (int j = 0; j < 10; j++) {
    unsigned long startTime = millis();
    while(true) {
      nodeHandler.spinOnce();
      unsigned long current = millis();
      if (startTime - current >= 10) {
        break;
      }
    }
    unsigned long curMillis = millis();
    if (curMillis - leftMillis > 500 || curMillis - rightMillis > 500) {
      analogWrite(LEFTPWM, 0);
      analogWrite(RIGHTPWM, 0);
    }
  }
  if (encoderLeft != knobLeft.read()) {
    encoderLeft = knobLeft.read();
    leftTicksMsg.data = encoderLeft;
    leftTicksPublisher.publish( &leftTicksMsg );
  }
  if (encoderRight != knobRight.read()) {
    encoderRight = knobRight.read();
    rightTicksMsg.data = encoderRight;
    rightTicksPublisher.publish( &rightTicksMsg );
  }
}
