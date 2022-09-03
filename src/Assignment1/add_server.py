from ros_course.srv import AddTwoInts
from ros_course.srv import AddTwoIntsRequest
from ros_course.srv import AddTwoIntsResponse

import time
import rospy


def handle_add_two_ints(req):
    print("Returning {} + {} = {}".format(req.a, req.b, (req.a + req.b)))
    response = AddTwoIntsResponse(req.a + req.b)
    return response 


def add_two_ints_server():
    rospy.init_node("add_two_ints_server")
    service = rospy.Service("add_two_ints", AddTwoInts, handle_add_two_ints)
    print("Ready to add integers")
    rospy.spin()
 

if __name__ == "__main__":
    add_two_ints_server()