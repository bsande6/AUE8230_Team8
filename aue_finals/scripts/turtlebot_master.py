#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time
import math
import sys
#from keyboard.msg import Key
#import keyboard
#from keyboard import Key
from pynput import keyboard
from std_msgs.msg import Int32
#from keyboard_press import KeyboardManager



import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
#from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3


move = Twist()
pub = rospy.Publisher("RobotA/cmd_vel",Twist, queue_size=10)


from geometry_msgs.msg import Pose
from keyboard.msg import Key
from std_msgs.msg import Int32
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

from apriltag_ros.msg import AprilTagDetectionArray


class Follow_Tag():
    def __init__(self):
        self.tag_pose_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_pose, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('RobotC/odom', Odometry, self.vehicle_pose)
        self.tag_pose = Pose()
        self.vehicle_pose = Pose()
        self.velocity_publisher = rospy.Publisher('RobotC/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

    def tag_pose(self, msg):
        if (len(msg.detections) > 0):
            self.tag_pose = msg.detections[0].pose.pose.pose
            #self.tag_pose.postion.y = tag_pose.position.z 


    def vehicle_pose(self, msg):
        self.vehicle_pose = msg.pose.pose

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return math.sqrt(pow((goal_pose.position.x - self.vehicle_pose.position.x), 2) +
                    pow((goal_pose.position.y - self.vehicle_pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def follow_tag(self):
        while not rospy.is_shutdown():
            self.vel_msg.linear.x = 0.08
            self.update_control()
            self.velocity_publisher.publish(self.vel_msg)
            rospy.sleep(1)

    def update_control(self):
        #self.vel_msg.linear.x = 0.1 * self.linear_vel(self.tag_pose)
        #self.vel_msg.angular.z = 0.1 * (self.steering_angle(self.tag_pose) + self.vehicle_pose.orientation.x)4
        self.vel_msg.angular.z = 4 * -self.tag_pose.position.x
        #print(self.vehicle_pose.orientation.z)

        # def steering_angle(self, goal_pose):
        #     print(math.atan2((goal_pose.position.y - self.vehicle_pose.position.y),(goal_pose.position.x - self.vehicle_pose.position.x)))
        #     return math.atan2((goal_pose.position.z),(goal_pose.position.x))
                



def PID(error, Kp=0.69): #defing a function for P-Controller
    return Kp*error 
        
def wallfollow(data): #defining a wall following function
    lidar_scan = list(data.ranges[0:359]) #storing LiDAR data 
    scan = []
    for i in range(len(lidar_scan)):
        #if lidar_scan[i]<3:
        if lidar_scan[i]<5:
            scan.append(lidar_scan[i]) #taking only values less than '3'  

    right = scan[-90:-16]
    right = sum(right)/len(right) #average distance of obstacles on the right 
    left = scan[16:90]
    left = sum(left)/len(left) #average distance of obstacles on the left 
        

    #front = scan[16:-16]
    left_front = scan[-45:-1]
    right_front = scan[1:45]

    #print("left_front is %s" % left_front)
    #print("right_front is %s" % right_front)


    front = (sum(left_front)+sum(right_front))/(len(left_front)+len(right_front)) #average distance of obstacles on the infront 

    #print("front is %s" % front)
    #print("right is %s" % right)
    #print("left is %s" % left)

    #linear_vel = 0.45
    #linear_vel = 0.35
    linear_vel = 0.25
    angular_vel = 0

    
    if front < 0.5:
        #linear_vel = -linear_vel*front
        linear_vel = 0
    elif front < 1:
        linear_vel = linear_vel*front
        
    
    #print("linear_vel is %s" % linear_vel)
 
    error = left-right #estimating the error for P-Controller
    
    move.linear.x = linear_vel #linear velocity
    move.angular.z = angular_vel + PID(error) #angular velocity
    #move.angular.z = 1
    #print("Angular Velocity is %s" % move.angular.z)
   
    pub.publish(move)
    
def find_right_side(input_list):
        middle = float(len(input_list))/2
        # if middle % 2 != 0:
        #     quarter = float((middle - 0.5)/2)
        # else:
        #     quarter = float(middle/2)

        right_mean = 0
        count = 0
        for i in range(340, 350):
            count += 1
            right_mean += input_list[int(i)]
        right_mean = right_mean / count
        return right_mean

def find_left_side(input_list):
        left_mean = 0
        count = 0
        for i in range(10, 30):
            count += 1
            left_mean += input_list[int(i)]
        left_mean = left_mean / count
        return left_mean

def find_front_sector(input_list):
    mean = 0
    count = 0
    for i in range(-10, 10):
        count += 1
        mean += input_list[int(i)]
    mean = mean / count
    return mean

def obstacleavoidance(data):
	# #print('Range at 20 degress: {}'.format(data.ranges[20])) # left forwward
	# #print('Range at 55 degress: {}'.format(data.ranges[55]))
	# #print('Range at 90 degress: {}'.format(data.ranges[90]))
	# #print('Range at 270 degress: {}'.format(data.ranges[270]))
	# #print('Range at 305 degress: {}'.format(data.ranges[305]))
	# #print('Range at 340 degress: {}'.format(data.ranges[340])) #right forward
	
	# thresh = 0.3
	# #velocity = 0.5   
	# #velocity = 0.25
	# #velocity = 0.155
	# velocity = 0.1
	# #move.angular.z = 0.155 #move left

	# if 1:

	# 	if data.ranges[20]>thresh and data.ranges[340]>thresh and data.ranges[0]>thresh:
	# 	   move.linear.x = velocity
	# 	   move.angular.z = 0
	# 	elif data.ranges[20]>thresh and data.ranges[55]>thresh and data.ranges[90]>thresh:
	# 	   move.linear.x = 0.0
	# 	   move.angular.z = velocity
	# 	   #if data.ranges[0]>thresh and data.ranges[20]>thresh and data.ranges[340]>thresh:
	# 		  #move.linear.x = velocity
	# 		  ##move.linear.x = 0.25
	# 		  #move.angular.z = 0.0
	# 	elif data.ranges[270]>thresh and data.ranges[305]>thresh and data.ranges[340]>thresh:
	# 	   move.linear.x = 0.0
	# 	   #move.angular.z = 0.85
	# 	   move.angular.z = velocity
	# 	   #if data.ranges[0]>thresh and data.ranges[20]>thresh and data.ranges[340]>thresh:
	# 		  #move.linear.x = velocity
	# 		  ##move.linear.x = 0.25
	# 		  #move.angular.z = 0.0
	# 	else:
	# 		if data.ranges[300]>thresh or data.ranges[40]>thresh:
	# 			move.linear.x = -0.1
	# 			#move.angular.z = 0
	# 			move.angular.z = velocity/2
	# 		else:
	# 			move.linear.x = 0.0
	# 			#move.angular.z = 0.5
	# 			move.angular.z = velocity
	
	# pub.publish(move)
    pass


def stoplidar(data):

    pass


def get_max(range_list):
    max_value = max(range_list)
    max_index = range_list.index(max_value)
    return max_index

class ObstacleAvoidance():
    def __init__(self):
        # Starts a new node
        #rospy.init_node('wall_following', anonymous=True)
        self.sector_degrees = 30
        self.safe_side_distance = 0.3
        self.left_scan_distance = float('inf')
        self.right_scan_distance = float('inf')
        self.front_left_scan_distance = float('inf')
        self.front_right_scan_distance = float('inf')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/RobotA/scan', LaserScan, self.scan_callback)
        self.min_distance = 0.4
        self.min_side_distance = 0.4
        self.front_scan_distance = float('inf')
        self.vel_msg = Twist()
        self.r = rospy.Rate(10)
        
    def wander(self):
        self.vel_msg.linear.x = 0.1
        while not rospy.is_shutdown():
            #values = [self.front_scan_distance, self.front_left_scan_distance, self.left_scan_distance, self.front_right_scan_distance, self.right_scan_distance]
            #max_index = get_max(values)
            
            if self.front_scan_distance > self.min_distance:
                if self.front_left_scan_distance < self.min_distance and self.front_right_scan_distance > self.min_distance:
                    self.turn_right()
                elif self.front_right_scan_distance < self.min_distance and self.front_left_scan_distance > self.min_distance:
                    self.turn_left()
                elif self.left_scan_distance < self.safe_side_distance and self.left_scan_distance < self.right_scan_distance:
                    self.angle_right()
                elif self.right_scan_distance < self.safe_side_distance and self.left_scan_distance > self.right_scan_distance:
                    self.angle_left()
                else:
                    self.go_forward()
            
            # if self.left_scan_distance < self.safe_side_distance and self.right_scan_distance > self.safe_side_distance:
            #         self.angle_right()
            #     elif self.right_scan_distance < self.safe_side_distance and self.left_scan_distance > self.safe_side_distance:
            #         self.angle_left()
            #     else:
            #         self.go_forward()
            
            else:
                if self.right_scan_distance < self.left_scan_distance:
                    self.turn_left()
                   
                else: 
                    self.turn_right()

                self.vel_msg.linear.x = 0

            self.velocity_publisher.publish(self.vel_msg)

    def go_forward(self):
        self.vel_msg.linear.x = 0.1
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
    def angle_right(self):
        self.vel_msg.linear.x = 0.08
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = -0.05
        if self.front_scan_distance < 1:
            self.vel_msg.angular.z += -0.07
        elif self.front_left_scan_distance < self.min_side_distance:
            self.vel_msg.angular.z += -0.05

    def angle_left(self):
        self.vel_msg.linear.x = 0.08
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0.05
        if self.front_scan_distance < 1:
            self.vel_msg.angular.z += 0.07
        elif self.front_right_scan_distance < self.min_side_distance:
            self.vel_msg.angular.z += 0.05
        
    def turn_left(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 1.5

    def turn_right(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = -1.5
        
    def emergency_brake(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0.0


    def scan_callback(self, msg):
        self.right_scan_distance = self.find_right_side(msg.ranges)
        self.left_scan_distance = self.find_left_side(msg.ranges)
        self.front_scan_distance = self.find_front_sector(msg.ranges)
        self.front_left_scan_distance = self.front_left_sector(msg.ranges)
        self.front_right_scan_distance = self.front_right_sector(msg.ranges)
        
    def find_right_side(self, input_list):
        
        middle = float(len(input_list))/2
        if middle % 2 != 0:
            quarter = float((middle - 0.5)/2)
        else:
            quarter = float(middle/2)

        right_mean = 0
        count = 0
        for i in range(int(quarter - self.sector_degrees), int(quarter + self.sector_degrees)):
            count += 1
            right_mean += input_list[int(i)]
        right_mean = right_mean / count
        return right_mean
       

    def find_left_side(self, input_list):
        middle = float(len(input_list))/2
        if middle % 2 != 0:
            quarter = float((middle/2  - 0.5))
        else:
            quarter = float(middle/2)
        
        left_mean = 0
        count = 0
        for i in range(int(quarter - self.sector_degrees), int(quarter + self.sector_degrees)):
            count += 1
            left_mean += input_list[int(i)]
        left_mean = left_mean / count
        return left_mean

    def front_left_sector(self, input_list):
        front_left_mean = 0
        count = 0
        for i in range(int(45 - self.sector_degrees), int(45 + self.sector_degrees)):
            count += 1
            front_left_mean += input_list[int(i)]
        front_left_mean = front_left_mean / count
        return front_left_mean

    def front_right_sector(self, input_list):
        front_right_mean = 0
        count = 0
        for i in range(int(315 - self.sector_degrees), int(315 + self.sector_degrees)):
            count += 1
            front_right_mean += input_list[int(i)]
        front_right_mean = front_right_mean / count
        return front_right_mean

    def find_front_sector(self, input_list):
        front_sector_mean = 0
        count = 0
        for i in range(int(0 - self.sector_degrees), int(0 + self.sector_degrees)):
            count += 1
            front_sector_mean += input_list[int(i)]

        front_sector_mean = front_sector_mean / count
        return front_sector_mean

class RealLineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        
        #cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        
        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        #crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
        #crop_img = cv_image[340:360][1:640]
        crop_img = cv_image[height-20:height][1:640]



        

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        

        # Define the Yellow Colour in HSV

        
        #To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        

        # Threshold the HSV image to get only yellow colors

        #lower_yellow = np.array([20,100,100])
        #upper_yellow = np.array([50,255,255])
        
        #lower_yellow = np.array([20,100,100])
        #lower_yellow = np.array([0, 50, 50])
        #lower_yellow = np.array([0, 75, 75])
        #lower_yellow = np.array([0, 50, 50])
        ##lower_yellow = np.array([20, 50, 50])
        lower_yellow = np.array([50, 50, 20])

        #upper_yellow = np.array([255,255,255])
        #lower_yellow = np.array([50, 50,0])
        upper_yellow = np.array([255,255,255])

       
        #rosrun assignment6_trackingandfollowing follow_line_step_hsv.py
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        #"""
        
        
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        
        try:
            cx, cy = m['m10']/(m['m00']+0.0001), m['m01']/(m['m00']+0.0001)
        except ZeroDivisionError:
            cx, cy = height/2, width/2
        

        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        #"""

        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################

        #twist_object = self.moveTurtlebot3_object.last_cmdvel_command
        twist_object = Twist()

        #rosrun assignment6_trackingandfollowing real_follow_line_step_hsv.py

        if 0: # previous movement
            #twist_object.linear.x = 0.05
            twist_object.linear.x = 0.1
            
            #twist_object.linear.x = 0.03
            #twist_object.linear.x = 0
            #twist_object.linear.x = 0.00
            #twist_object.linear.x = 0.5
            #twist_object.angular.z = -0.25*(cx - width/2)/100
            twist_object.angular.z = -0.1*(cx - width/2)/100
                #twist_object.angular.z = math.pi/4
            #twist_object.linear.x = 0
            #twist_object.angular.z = 0.001
            #twist_object.angular.z = 0
        
        else: # modified spins until line is found and then moves 

            if self.lane_follow_pause_count > 0:
                self.lane_follow_pause_count = self.lane_follow_pause_count-1
                twist_object.angular.z = 0
                twist_object.linear.x = 0
            elif cx == 0:
                twist_object.angular.z = 0.1
                twist_object.linear.x = 0
            else:
                #twist_object.linear.x = 0.25
                twist_object.linear.x = 0.1
                #twist_object.angular.z = -(cx - width/2)/(100)
                #twist_object.angular.z = -(cx - width/2)/(150)

                if (cx - width/2) > 100:
                    twist_object.angular.z = -0.5
                elif (cx - width/2) < -100:
                    twist_object.angular.z = 0.5
                else:
                    #twist_object.angular.z = 0
                    twist_object.angular.z = -(cx - width/2)/(150)

        self.moveTurtlebot3_object.last_cmdvel_command = twist_object

        rospy.loginfo("")
        rospy.loginfo("cy===>"+str(cx))  #left:960
        rospy.loginfo("width===>"+str(width))
        rospy.loginfo("height===>"+str(height))
        rospy.loginfo("cy - width/2===>"+str(cx - width/2))

        #time.sleep(2.4)

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


class LineFollower(object):

	def __init__(self):

		print("Running version 2")

		self.bridge_object = CvBridge()
		#self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		self.image_sub = rospy.Subscriber("RobotA/camera/rgb/image_raw",Image,self.camera_callback)
		#self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
		self.moveTurtlebot3_object = MoveTurtlebot3()

		sub = rospy.Subscriber("RobotA/scan",LaserScan, stoplidar)

		move.linear.x = 0
		move.angular.z = 0

		pub.publish(move)


		self.lane_follow_pause_count = 100


	def camera_callback(self, data):
		# We select bgr8 because its the OpneCV encoding by default
		
		#cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

		try:
			# We select bgr8 because its the OpneCV encoding by default
			cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)

		
		# We get image dimensions and crop the parts of the image we dont need
		height, width, channels = cv_image.shape
		#crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
		crop_img = cv_image[height-20:height][1:640]
		#crop_img = cv_image[340:360][1:640]



		

		# Convert from RGB to HSV
		hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

		# Define the Yellow Colour in HSV

		
		#To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
		

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

		#twist_object.linear.x = 0.05
		#twist_object.linear.x = 0.00
		
		#if cx == 1920:
		if self.lane_follow_pause_count > 0:
			self.lane_follow_pause_count = self.lane_follow_pause_count-1
			twist_object.angular.z = 0
			twist_object.linear.x = 0
		elif cx == 0:
			twist_object.angular.z = 0.1
			twist_object.linear.x = 0
		else:
			#twist_object.linear.x = 0.25
			twist_object.linear.x = 0.1
			#twist_object.angular.z = -(cx - width/2)/(100)
			#twist_object.angular.z = -(cx - width/2)/(150)

			if (cx - width/2) > 100:
				twist_object.angular.z = -0.5
			elif (cx - width/2) < -100:
				twist_object.angular.z = 0.5
			else:
				#twist_object.angular.z = 0
				twist_object.angular.z = -(cx - width/2)/(150)

			#twist_object.angular.z = math.pi/4
		#twist_object.linear.x = 0
		#twist_object.angular.z = 0.001
		#twist_object.angular.z = 0

		self.moveTurtlebot3_object.last_cmdvel_command = twist_object

		rospy.loginfo("")
		rospy.loginfo("self.lane_follow_pause_count===>"+str(self.lane_follow_pause_count)) 
		rospy.loginfo("cy===>"+str(cx))  #left:960
		rospy.loginfo("width===>"+str(width))
		rospy.loginfo("-(cy - width/2)/100===>"+str(-(cx - width/2)/100))

		
		rospy.loginfo("self.moveTurtlebot3_object===>"+
			str(self.moveTurtlebot3_object.compare_twist_commands(
				self.moveTurtlebot3_object.last_cmdvel_command,
				twist_object)))
		rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))


		# Make it start turning
		self.moveTurtlebot3_object.move_robot(twist_object)

	def clean_up(self):
		self.moveTurtlebot3_object.clean_class()
		cv2.destroyAllWindows()



class Master():
    def __init__(self):
        rospy.init_node('master', anonymous=True)
        #self.launch_subscriber = rospy.Publisher('/launch_abort', Int32, self.launch_callback)
        #self.wall_follow_publisher = rospy.Publisher('/wall_follow', String, queue_size=10)
        #self.keyboard_sub = rospy.Subscriber('/keyboard/keydown', keyboard.Key, self.get_key, queue_size=1)
        self.key_code = -1

        self.mainloop()
    
    def get_key(self, msg):
        self.key_code = msg.code
        rospy.loginfo(self.key_code)
   
    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(5)
        # While ROS is still running
        while not rospy.is_shutdown():
            # Publish the abort message
            #self.abort_pub.publish(self.key_code)

            value = input("Please enter a string:\n")

            # Check if any key has been pressed
            #if self.key_code ==  keyboard.Key.KEY_w:
            if value == "w":
                print("w key was pressed!")
                #wall_follow = WallFollowing()
                #wall_follow.wall_following()
                #rospy.init_node('wall_follower')
                #pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
                sub = rospy.Subscriber("/RobotA/scan",LaserScan, wallfollow)
                #while not rospy.is_shutdown():
                #    pub.publish(move)
                #    pass                

            #elif self.key_code ==  keyboard.Key.KEY_o:
            elif value == "o":
                print("o key was pressed!")
                obstacle_avoidance = ObstacleAvoidance()
                obstacle_avoidance.wander()
                #rospy.init_node('obstacle_avoidance')
                #pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
                #sub = rospy.Subscriber("/RobotA/scan",LaserScan, obstacleavoidance)
                sub = rospy.Subscriber("/RobotA/scan",LaserScan, obstacle_avoidance)
                
                #rospy.spin() 

            elif value == "l":
                print("l key was pressed!")
                line_follower_object = LineFollower()
                rate = rospy.Rate(5)

            elif value == "r":
                print("r key was pressed!")
                line_follower_object = RealLineFollower()
                rate = rospy.Rate(5)

            elif value == "a":
                print("a key was pressed!")
                apriltag_follow = Follow_Tag()
                apriltag_follow.follow_tag()                

            # Reset the code
            if self.key_code != -1:
                self.key_code = -1

            # Sleep for the remainder of the loop
            rate.sleep()
       
    def control_switch(self):
        pass
        


"""
def on_press(key):
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    #if k in ['w', 'o', 'left', 'right']:  # keys of interest
    if k in ['w', 'o']:  # keys of interest
        # self.keys.append(k)  # store it in global-like variable
        print('Key pressed: ' + k)

        if k == 'w':
            ;
        elif k == 'o':
            ;


        return False  # stop listener; remove this if want more keys

listener = keyboard.Listener(on_press=on_press)
listener.start()  # start to listen on a separate thread
listener.join()  # remove if main thread is polling self.keys
"""


if __name__ == '__main__':
    try:


        master = Master()
        rospy.spin()
    except rospy.ROSInterruptException: pass

