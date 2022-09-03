#!/usr/bin/env python

import ros
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty


def poseCallback(msg):
    global x, y, yaw
    x = msg.x
    y = msg.y
    yaw = msg.theta
    # print("Pose of the robot: \n")
    # print("x: {:f}\ny: {:f}\nyaw: {:f}\n".format(x, y, yaw))


def move(speed, distance):
    # declare turtlesim velocity message
    velocity_message = Twist()

    # retrieve current location of robot
    x0 = x
    y0 = y
    yaw0 = yaw

    # assign the x coordinate of linear velocity to the speed.
    velocity_message.linear.x = speed
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # 10 Hz

    # create a publisher for the velocity message on the appropriate topic.  
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves forward")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        # measure the distance moved
        distance_moved += abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)
        if (distance_moved >= distance):
            rospy.loginfo("Target reached")
            break


if __name__ == "__main__":
    try:
        rospy.init_node("turtlesim_motion_pose", anonymous=True)
        rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
        time.sleep(2)
        print("move: ")
        move (1.0, 5.0)
        time.sleep(2)
        print("start reset: ")
        rospy.wait_for_service("reset")
        reset_turtle = rospy.ServiceProxy("reset", Empty)
        reset_turtle()
        print("end reset: ")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")