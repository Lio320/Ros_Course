from cmath import isnan
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time
import numpy as np


class ScannerSub():
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel_command = Twist()
        self.Kp_mot = 0.1
        self.Kp_rot = 0.3


    def scan_callback(self, msg):
        # Cycle between indices -5, 5
        self.pid_controller(msg.ranges)
        # print(self.collision_sensor_indices(msg.ranges, -5, 0, 0.6))


    def pid_controller(self, ranges):
        loop_rate = rospy.Rate(10)
        min_distance, index = self.get_minimum_range_indices(ranges, -5, 5)
        
        self.vel_command.linear.x = 0.4

        if self.collision_sensor_indices(ranges, -10, 10, 0.6):
            self.vel_command.linear.x = self.Kp_rot*min_distance
            if index < 6:
                self.vel_command.angular.z = +self.Kp_rot/min_distance
            else:
                self.vel_command.angular.z = -self.Kp_rot/min_distance
        else:
            self.vel_command.angular.z = 0.0

        self.cmd_pub.publish(self.vel_command)
        loop_rate.sleep()


    def get_fov(self, angle_min, angle_max):
        return (angle_min-angle_max)*180.0/3.14


    def get_max_range(self, ranges):
        max = 0
        index = 0
        for range in ranges:
            if range>max:
                max = range
                index = ranges.index(max)
        return max, index


    def get_min_range(self, ranges):
        min = np.inf
        index = 0
        for range in ranges:
            print("RANGE", range)
            if range<min and not math.isnan(range):
                min = range
                index = ranges.index(min)
        return min, index


    def get_average_range(self, ranges):
        sum = 0
        count = 0
        for range in ranges:
            if not math.isnan(range):
                count += 1
                sum += range
        return sum/count


    def get_average_range_indices(self, ranges, i, j):
        # This implementation works also for negative indices
        sum = 0
        count = 0
        for index in range(i, j+1):
            if not math.isnan(ranges[index]):
                count += 1
                sum += ranges[index]
        return sum/count


    def collision_sensor_indices(self, ranges, i, j, thr):
        # This implementation works also for negative indices
        for index in range(i, j+1):
            if (ranges[index] < thr):
                return True
        return False

    
    def get_minimum_range_indices(self, ranges, i, j):
        array = []
        for index in range(i, j+1):
            array.append(ranges[index])
        return np.min(array), np.argmin(array)


def main():
    rospy.init_node("scan_subscriber", anonymous=True)
    ScannerSub()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()