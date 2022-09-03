#include "ros/ros.h"
#include "ros_course/IoTSensor.h"

void iotCallback(ros_course::IoTSensor msg){
    ROS_INFO("I Heard:\nId: %d\nName: %s\nHumidity %f\nTemperature: %f", msg.id, msg.name.c_str(), msg.humidity, msg.temperature);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "iot_sensor_listener");
    ros::NodeHandle n;
    ros::Subscriber iot_sub = n.subscribe("iot_publisher_topic", 100, iotCallback);
    ros::spin();
    return 0;

}