#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>

#define LEFTDIR 9;
#define LEFTPWM 10;
#define RIGHTDIR 11;
#define RIGHTPWM 12;

#define TICKSPERMETER 138000;
#define MAXSPEED 1.35;
#define MAXPWM 255;

Encoder knobLeft(5, 6);
Encoder knobRight(7, 8);

long leftTicks = 0;
long rightTicks = 0;
unsigned long leftMillis = millis();
unsigned long rightMillis = millis();
float leftCo = MAXPWM/MAXSPEED;
float rightCo = MAXPWM/MAXSPEED;
int curLeftPwm = 0;
int curRightPwm = 0;

ros::NodeHandle nodeHandler;

std_msgs::Int64 leftTicskMsg;
std_msgs::Int64 rightTicskMsg;
std_msgs::Float32 leftVelMsg;
std_msgs::Float32 rightVelMsg;
std_msgs::Int8 encoderResetMsg;

void leftCallback( const std_msgs::Float32& msg ) {
  // Update left side coefficient using travel / time and old pwm
  long newTicks = knobLeft.read();
  unsigned long newMillis = millis();
  if (curLeftPwm < 15) {
    float curSpeed = (newTicks - leftTicks)/(newMillis - leftMillis);
    curSpeed = curSpeed / TICKSPERMETER * 1000;
    leftCo = (float) curLeftPwm / curSpeed;
  }
  leftTicks = newTicks;
  leftMillis = newMillis;
  //Convert leftVelMsg.data to direction and vel
  float vel = msg.data;
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

void rightCallback( const std_msgs::Float32& msg ) {
  // Update right side coefficient using travel / time and old pwm
  long newTicks = knobLeft.read();
  unsigned long newMillis = millis();
  if (curRightPwm < 15) {
    float curSpeed = (newTicks - rightTicks)/(newMillis - rightMillis);
    curSpeed = curSpeed / TICKSPERMETER * 1000;
    rightCo = (float) curRightPwm / curSpeed;
  }
  rightTicks = newTicks;
  rightMillis = newMillis;
  
  //Convert leftVelMsg.data to direction and vel
  float vel = msg.data;
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

void resetCallback( const std_msgs::Int8& msg ) {
  int8_t reset = msg.data;
  if (reset == 1) {
    leftTicks = 0;
    leftMillis = millis(0);
    rightTicks = 0;
    rightMillis = millis(0;
  }
}

//ros::Publisher publisher("topic", msg);
//ros::Subscriber<namespace::type> subscriber(topic, msg)

ros::Publisher leftTicksPublisher("Odometry/leftTicks", &leftTicksMsg);
ros::Publisher rightTicksPublisher("Odometry/rightTicks", &rightTicksMsg);
ros::Subscriber<std_msgs::Float32> leftVelSubscriber("leftVelocity", &leftVelMsg);
ros::Subscriber<std_msgs::Float32> rightVelSubscriber("rightVelocity", &rightVelMsg);
ros::Subscriber<std_msgs::Int8> encoderResetSubscriber("encoderReset", &encoderResetMsg);


void setup() {
  // put your setup code here, to run once:
  pinMode(LEFTDIR, OUTPUT);
  pinMode(LEFTPWM, OUTPUT);
  pinMode(RIGHTDIR, OUTPUT);
  pinMode(RIGHTPWM, OUTPUT);

  nodeHandler.initNode();
  nodeHandler.advertise(leftTicksPublisher);
  nodeHandler.advertise(rightTicksPublisher);
  nodeHandler.subscribe(leftVelSubscriber);
  nodeHandler.subscribe(rightVelSubscriber);
  nodeHandler.subscribe(encoderResetSubscriber);
}

void loop() {
  long encoderLeft = leftTicks;
  long encoderRight = rightTicks;
  while(true){
    if (encoderLeft != leftTicks) {
      leftTicksMsg.data = leftTicks;
      leftTicksPublisher.publish( &leftTicksMsg );
      encoderLeft = leftTicks;
    }
    if (encoderRight != rightTicks) {
      rightTicksMsg.data = rightTicks;
      rightTicksPublisher.publish( &rightTicksMsg );
      encoderRight = rightTicks;
    }
    for (j = 0; j < 5; j++) {
      delay(19);
      nodeHandler.spinOnce();
      curMillis = millis();
      if (curMillis - leftMillis > 250 || curMillis - leftMillis > 250) {
        analogWrite(LEFTPWM, 0);
        analogWrite(RIGHTPWM, 0);
      }
    }
  }
}
