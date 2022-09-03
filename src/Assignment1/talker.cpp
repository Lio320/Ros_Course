#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <sstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter = n.advertise<std_msgs::String>("chatter", 100);

    ros::Rate loop_rate(10);

    int counter = 0;
    while(ros::ok()){
        std_msgs::String message;
        std::stringstream stream;
        stream << "Hello world " << counter;
        message.data = stream.str();
        ROS_INFO("%s", message.data.c_str());
        chatter.publish(message);
        
        ros::spinOnce();
        loop_rate.sleep();
        counter++;
    }
    return 0;
}