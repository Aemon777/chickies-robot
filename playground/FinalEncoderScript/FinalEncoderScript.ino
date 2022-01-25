#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>

#define LEFTDIR 9;
#define LEFTPWM 10;
#define RIGHTDIR 11;
#define RIGHTPWM 12;

#define TICKSPERMETER 21500;
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
  long newMillis = knobLeft.read();
  unsigned long newTime = millis();
  float curSpeed = (newTicks - leftTicks)/(newTime - leftMillis);
  leftTicks = newTicks;
  leftMillis = newMillis;
  curSpeed = curSpeed / TICKSPERMETER * 1000;
  leftCo = (float) curLeftPwm / curSpeed;
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
  long newMillis = knobLeft.read();
  unsigned long newTime = millis();
  float curSpeed = (newTicks - rightTicks)/(newTime - rightMillis);
  rightTicks = newTicks;
  rightMillis = newMillis;
  curSpeed = curSpeed / TICKSPERMETER * 1000;
  rightCo = (float) curRightPwm / curSpeed;
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
    //reset
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
  // put your main code here, to run repeatedly:
  // at 10 Hz, encoders: check for motion
  // if motion, record current ticks and publish
  // this can be with a counter of some sort or we can make this a while true, for i = 1:5 kinda thing
  // at 50 Hz, spin node handler
  // this will handle callbacks for ticks sides
  delay(20);
}
