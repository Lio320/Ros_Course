from ros_course.srv import AddTwoInts
from ros_course.srv import AddTwoIntsRequest
from ros_course.srv import AddTwoIntsResponse

import rospy
import sys


def add_two_ints_client(x, y):
    rospy.wait_for_service("add_two_ints")
    try:
        add_two_ints = rospy.ServiceProxy("add_two_ints", AddTwoInts)
        response = add_two_ints(x, y)
        return response.sum
    except rospy.ServiceException as exc:
        print("Service call failed: {}".format(exc))



if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])

    else:
        print("The number of arguments doesn't match the message")
        sys.exit(1)
    print("Requesting add {} and {}".format(x, y))
    service = add_two_ints_client(x, y)
    print("Summation is: {}".format(service))