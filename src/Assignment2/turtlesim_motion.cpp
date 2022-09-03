#include "ros/ros.h"
#include "iostream"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <stdlib.h>
#include <chrono>
#include <thread>


turtlesim::Pose turtlesim_pose;
ros::Publisher vel_pub;
ros::Subscriber pose_sub;


void poseCallback(const turtlesim::Pose::ConstPtr msg){
    turtlesim_pose.x = msg->x;
    turtlesim_pose.y = msg->y;
    turtlesim_pose.theta = msg->theta;
}

double get_distance(double x, double y, double x_target, double y_target){
    return sqrt(pow(x-x_target, 2.0) + pow(y-y_target, 2.0));
}


void rotate(double angular_speed, double desired_angle, bool clockwise){
    geometry_msgs::Twist vel_message;

    if (clockwise){
        vel_message.angular.z = -abs(angular_speed);
    }
    else{
        vel_message.angular.z = abs(angular_speed);
    }

    double current_angle = 0.0;
    double t0 = ros::Time::now().toSec();
    double t1;
    ros::Rate loop_rate(10);

    while(current_angle < desired_angle && ros::ok()){
        vel_pub.publish(vel_message);
        t1 = ros::Time::now().toSec();
        current_angle = (t1-t0)*angular_speed;
        ros::spinOnce();
        loop_rate.sleep();
    }
    vel_message.angular.z = 0;
    vel_pub.publish(vel_message);
    std::cout << "Rotation angle " << current_angle << "\n";
}



void move(double speed, double distance, bool isForward){
    geometry_msgs::Twist vel_message;
    if (isForward){
        vel_message.linear.x = std::abs(speed);
    }
    else{
        vel_message.linear.x = -std::abs(speed);
    }
    
    double t0 = ros::Time::now().toSec();
    double t1;
    ros::Rate loop_rate(10);
    double curr_distance = 0.0;
    while (curr_distance < distance && ros::ok())
    {
        vel_pub.publish(vel_message);
        t1 = ros::Time::now().toSec();
        curr_distance = (t1-t0)*speed;
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Distance moved " << curr_distance << "\n" << "Time taken " << (t1-t0) << "\n"; 
    vel_message.linear.x = 0;
    vel_pub.publish(vel_message);
}


void go_to_goal(turtlesim::Pose goal_pose, double dist_tolerance){
    geometry_msgs::Twist vel_message;
    double curr_dist = get_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

    double curr_heading;
    double Kp = 0.5;
    double Ki = 0.02;
    ros::Rate loop_rate(100);

    while(curr_dist > dist_tolerance && ros::ok()){
        curr_dist = get_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
        curr_heading = 4*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);
        vel_message.linear.x = Kp*curr_dist;
        vel_message.angular.z = curr_heading;
        vel_pub.publish(vel_message);
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "End of race, target reached \n";

    vel_message.linear.x = 0;
    vel_message.angular.z = 0;
    vel_pub.publish(vel_message);

}

void setDesiredOrientation(double angle, double speed){
    double motion = angle - turtlesim_pose.theta;
    bool clockwise;
    if (motion>0){
        clockwise = false;
    }
    else{
        clockwise = true;
    }
    rotate(speed, abs(motion), clockwise);
}

void spiralMotion(double speed, double angle){
    geometry_msgs::Twist vel_msg;
    ros::Rate loop(1);

    while(turtlesim_pose.x<10.5 && turtlesim_pose.y<10.5){
        speed += 1.0;
        vel_msg.linear.x = speed;
        vel_msg.angular.z = angle;

        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop.sleep();
    }
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
}

void gridClean(){
    ros::Rate loop(0.5);
    turtlesim::Pose pose;
    pose.x = 1;
    pose.y = 1;
    pose.theta = 0;
    go_to_goal(pose, 0.01);
    loop.sleep();
    setDesiredOrientation(0, 1.0);
    loop.sleep();
    move(2.0, 9.0, true);
    rotate(0.2, M_PI/2, false);
    loop.sleep();
    move(2.0, 9.0, true);
    rotate(0.2, M_PI/2, false);
    loop.sleep();
    move(2.0, 9.0, true);
    rotate(0.2, M_PI/2, false);
    loop.sleep();
    move(2.0, 9.0, true);

}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "turtlesim_motion");
    ros::NodeHandle n;
    std::string vel_topic = "/turtle1/cmd_vel";
    std::string pose_topic = "/turtle1/pose";
    turtlesim_pose.x = 0;
    turtlesim_pose.y = 0;
    turtlesim_pose.theta = 0;
    pose_sub = n.subscribe<turtlesim::Pose>(pose_topic, 10, poseCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>(vel_topic, 10);

    double speed = 2.0;
    double distance = 4.0;
    bool isForward = false;

    double angular_speed = 1.0;
    double angle = M_PI/2;
    bool clockwise = true;

    turtlesim::Pose goal_pose;
    double dist_tolerance =0.01;
    if (argc>1){
        goal_pose.x = std::stod(argv[1]);
        goal_pose.y = std::stod(argv[2]);
    }
    else{
        goal_pose.x = 1;
        goal_pose.y = 1;
    }

    // move(speed, distance, isForward);
    // rotate(angular_speed, angle, clockwise);
    // go_to_goal(goal_pose, dist_tolerance);
    // setDesiredOrientation(angle, angular_speed);
    spiralMotion(1.0, 4.0);
    // gridClean();
    return 0;
}