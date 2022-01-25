#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>

#define LEFTDIR 9;
#define LEFTPWM 10;
#define RIGHTDIR 11;
#define RIGHTPWM 12;

Encoder knobLeft(5, 6);
Encoder knobRight(7, 8);

long leftTicks = 0;
long rightTicks = 0;
unsigned long leftMillis = 0;
unsigned long rightMillis = 0;

std_msgs::Int64 leftTicskMsg;
std_msgs::Int64 rightTicskMsg;
std_msgs::Float32 leftVelMsg;
std_msgs::Float32 rightVelMsg;
std_msgs::Int8 encoderResetMsg;

void leftCallback( const std_msgs::Float32& leftVelMsg ) {
  //Compare current to former ticks
  //Compare current to former time
  //Convert leftVelMsg.data to direction and vel
  //Convert vel to pwm (approximate, assuming 100% duty means 1.35 m/s -- start with this constant as assumption, update accordingly)
  //Compare vel to last vel and adjust velocity accordingly
}

void rightCallback( const std_msgs::Float32& rightVelMsg ) {
  
}

//ros::Publisher publisher("topic", msg);
//ros::Subscriber<namespace::type> subscriber(topic, msg)

void setup() {
  // put your setup code here, to run once:
  pinMode(LEFTDIR, OUTPUT);
  pinMode(LEFTPWM, OUTPUT);
  pinMode(RIGHTDIR, OUTPUT);
  pinMode(RIGHTPWM, OUTPUT);

  //Node handler: initNode()
  //handler.advertise(publishers)
  //handler.subscribe(subscribers)
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
