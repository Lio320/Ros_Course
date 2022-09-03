#include "ros/ros.h"
#include "ros_course/IoTSensor.h"
#include <stdlib.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "iot_sensor_publisher");
    ros::NodeHandle n;
    ros::Publisher iot_pub = n.advertise<ros_course::IoTSensor>("iot_publisher_topic", 100);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros_course::IoTSensor message;
        message.id = 1;
        message.name = "IoT_Publisher_01";
        message.humidity = 33.41 + 2*(std::rand() % 10);
        message.temperature = 24.33 + 2*(std::rand() % 10);

        iot_pub.publish(message);
        ROS_INFO("Publishing IoT info");

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}