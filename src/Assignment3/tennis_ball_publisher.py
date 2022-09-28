import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def main():
    try:
        bridge = CvBridge()
        video_name = "src/Assignment3/video/tennis-ball-video.mp4"
        rospy.init_node("tennis_publisher", anonymous=True)
        tennis_pub = rospy.Publisher("tennis_ball_image", Image, queue_size=10)
        video_capture = cv2.VideoCapture(video_name)
        rate = rospy.Rate(30)
        while video_capture.isOpened():
            ret, frame = video_capture.read()
            frame = np.array(frame)
            if ret == True:
                message = bridge.cv2_to_imgmsg(frame, "bgr8")
                tennis_pub.publish(message)
                rate.sleep()
            else:
                print('no video')
                video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            if rospy.is_shutdown():
                video_capture.release()
                break
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")

if __name__ == "__main__":
    main()