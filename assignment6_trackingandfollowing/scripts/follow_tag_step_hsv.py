#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3

import math

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
        #crop_img = cv_image[340:360][1:640]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/(m['m00']+0.0001), m['m01']/(m['m00']+0.0001)
        except ZeroDivisionError:
            cx, cy = height/2, width/2
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################

        #twist_object = self.moveTurtlebot3_object.last_cmdvel_command
        twist_object = Twist()

        #twist_object.linear.x = 0.2
        twist_object.linear.x = 0.5
        twist_object.angular.z = -(cx - width/2)/100
            #twist_object.angular.z = math.pi/4
        #twist_object.linear.x = 0
        #twist_object.angular.z = 0.001
        #twist_object.angular.z = 0

        self.moveTurtlebot3_object.last_cmdvel_command = twist_object

        rospy.loginfo("")
        rospy.loginfo("cy===>"+str(cx))  #left:960
        rospy.loginfo("width===>"+str(width))

        rospy.loginfo("cy - width/2===>"+str(cx - width/2))

        """
        rospy.loginfo("self.moveTurtlebot3_object===>"+
            str(self.moveTurtlebot3_object.compare_twist_commands(
                self.moveTurtlebot3_object.last_cmdvel_command,
                twist_object)))
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        """

        # Make it start turning
        self.moveTurtlebot3_object.move_robot(twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time2!")
        ctrl_c = True
        quit()

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        if(ctrl_c):
            rospy.loginfo("ctrl_c")
        else:
            rospy.loginfo("no ctrl_c")

        rate.sleep()

if __name__ == '__main__':
        main()
