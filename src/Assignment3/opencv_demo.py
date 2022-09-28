import rospy
import cv2
import os
import numpy as np

def read_image(path, as_gray=False):
    if as_gray == True:
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    else:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
    return image

def save_copy(path):
    image = cv2.imread(path)
    cv2.imwrite("images/copy/copy_image.jpg", image)

def draw(image):
    cv2.line(image,(0,0),(511,511),(255,255,255),5)

    cv2.rectangle(image,(384,0),(510,128),(0,255,0),3)
    cv2.ellipse(image,(256,256),(100,50),0,0,180,255,-1)

    pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.polylines(image,[pts],True,(0,255,255))

    font = cv2.FONT_HERSHEY_SIMPLEX
    #cv2.putText(image,'ROS, OpenCV',(10,500), font, 2,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(image,'OpenCV',(10,500), font, 4,(255,255,255),2)
    return image

def tresholding(image, threshold_value, adaptive=False):
    if adaptive == True:
        image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, threshold_value, 2)
    else:
        ret, image = cv2.threshold(image, threshold_value, 255, cv2.THRESH_BINARY_INV)
    return image

def color_filtering(image, lowerBound, upperBound):
    # Convert from BGR to HSV space to make filtering more intuitive
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image = cv2.inRange(image, lowerBound, upperBound)
    return image

def get_contours(image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_contours(image, contours):
    index = -1 #means all contours
    thickness = 10 #thinkess of the contour line
    color = (0, 0, 255) #color of the contour line
    cv2.drawContours(image, contours, index, color, thickness)
    cv2.imshow("Contours Image" ,image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def process_contours(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
        cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
        cx, cy = get_contour_center(c)
        cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
        cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
        print ("Area: {}, Perimeter: {}".format(area, perimeter))
    print ("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    cv2.imshow("Black Image Contours",black_image)

if __name__ == "__main__":
    path = "images/shapes.png"
    rgb_image = read_image(path, False)
    gray_image = read_image(path, True)
    # image = draw(image)
    image = tresholding(gray_image, 125, True)
    # lower_bound = (30, 150, 100)
    # upper_bound = (50, 255, 255)
    # image = color_filtering(image, lower_bound, upper_bound)

    contours = get_contours(image)
    # draw_contours(image, contours)
    process_contours(image, rgb_image, contours)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
