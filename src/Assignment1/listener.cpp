#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sstream"
#include "string"
#include <iostream>

void listen_callback(std_msgs::String msg){
    ROS_INFO("I Heard %s", msg.data.c_str());

    // std::printf("I heard %s \n", msg.data.c_str());
    
}


int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber subscriber = n.subscribe("chatter", 1000, listen_callback);
    ros::spin();

    return 0;

}