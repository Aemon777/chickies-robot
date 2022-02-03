#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nodeHandler;

std_msgs::Int8 testMsg;

ros::Publisher testPublisher("arduinoTest", &testMsg);

void setup() {
    nodeHandler.initNode();
    nodeHandler.advertise(testPublisher);
}

void loop() {
    while (true) {
        for (int8_t i = 0; i < 100; i++) {
            testMsg.data = i;
            testPublisher.publish( &testMsg );
            nodeHandler.spinOnce();
            delay(100);
        }
    }
}
