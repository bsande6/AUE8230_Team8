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
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)





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
		

	#front_average = scan[16:-16]
	left_front = scan[-45:-1]
	right_front = scan[1:45]

	#print("left_front is %s" % left_front)
	#print("right_front is %s" % right_front)


	front_average = (sum(left_front)+sum(right_front))/(len(left_front)+len(right_front)) #average distance of obstacles on the infront 

	#print("front_average is %s" % front_average)
	#print("right is %s" % right)
	#print("left is %s" % left)

	#linear_vel = 0.45
	#linear_vel = 0.35
	linear_vel = 0.25
	angular_vel = 0

	
	if front_average < 0.5:
		#linear_vel = -linear_vel*front_average
		linear_vel = 0
	elif front_average < 1:
		linear_vel = linear_vel*front_average
	
	#print("linear_vel is %s" % linear_vel)
 
	error = left-right #estimating the error for P-Controller
	
	move.linear.x = linear_vel #linear velocity
	move.angular.z = angular_vel + PID(error) #angular velocity
	#move.angular.z = 1
	#print("Angular Velocity is %s" % move.angular.z)
	#print("")
	pub.publish(move)



def obstacleavoidance(data):
	#print('Range at 20 degress: {}'.format(data.ranges[20])) # left forwward
	#print('Range at 55 degress: {}'.format(data.ranges[55]))
	#print('Range at 90 degress: {}'.format(data.ranges[90]))
	#print('Range at 270 degress: {}'.format(data.ranges[270]))
	#print('Range at 305 degress: {}'.format(data.ranges[305]))
	#print('Range at 340 degress: {}'.format(data.ranges[340])) #right forward
	
	thresh = 0.3
	#velocity = 0.5   
	#velocity = 0.25
	#velocity = 0.155
	velocity = 0.1
	#move.angular.z = 0.155 #move left

	if 1:

		if data.ranges[20]>thresh and data.ranges[340]>thresh and data.ranges[0]>thresh:
		   move.linear.x = velocity
		   move.angular.z = 0
		elif data.ranges[20]>thresh and data.ranges[55]>thresh and data.ranges[90]>thresh:
		   move.linear.x = 0.0
		   move.angular.z = velocity
		   #if data.ranges[0]>thresh and data.ranges[20]>thresh and data.ranges[340]>thresh:
			  #move.linear.x = velocity
			  ##move.linear.x = 0.25
			  #move.angular.z = 0.0
		elif data.ranges[270]>thresh and data.ranges[305]>thresh and data.ranges[340]>thresh:
		   move.linear.x = 0.0
		   #move.angular.z = 0.85
		   move.angular.z = velocity
		   #if data.ranges[0]>thresh and data.ranges[20]>thresh and data.ranges[340]>thresh:
			  #move.linear.x = velocity
			  ##move.linear.x = 0.25
			  #move.angular.z = 0.0
		else:
			if data.ranges[300]>thresh or data.ranges[40]>thresh:
				move.linear.x = -0.1
				#move.angular.z = 0
				move.angular.z = velocity/2
			else:
				move.linear.x = 0.0
				#move.angular.z = 0.5
				move.angular.z = velocity
	
	pub.publish(move)


def avoidanceOld(data):
	#print('Range at 20 degress: {}'.format(data.ranges[20])) # left forwward
	#print('Range at 55 degress: {}'.format(data.ranges[55]))
	#print('Range at 90 degress: {}'.format(data.ranges[90]))
	#print('Range at 270 degress: {}'.format(data.ranges[270]))
	#print('Range at 305 degress: {}'.format(data.ranges[305]))
	#print('Range at 340 degress: {}'.format(data.ranges[340])) #right forward
	
	thresh = 0.3
	#velocity = 0.5   
	#velocity = 0.25
	#velocity = 0.155
	velocity = 0.1
	#move.angular.z = 0.155 #move left


	if 0:

		if data.ranges[20]>thresh and data.ranges[340]>thresh and data.ranges[0]>thresh:
		   move.linear.x = velocity
		   move.angular.z = 0
		elif data.ranges[20]>thresh and data.ranges[55]>thresh and data.ranges[90]>thresh:
		   move.linear.x = 0.0
		   move.angular.z = velocity
		   #if data.ranges[0]>thresh and data.ranges[20]>thresh and data.ranges[340]>thresh:
			  #move.linear.x = velocity
			  ##move.linear.x = 0.25
			  #move.angular.z = 0.0
		elif data.ranges[270]>thresh and data.ranges[305]>thresh and data.ranges[340]>thresh:
		   move.linear.x = 0.0
		   #move.angular.z = 0.85
		   move.angular.z = velocity
		   #if data.ranges[0]>thresh and data.ranges[20]>thresh and data.ranges[340]>thresh:
			  #move.linear.x = velocity
			  ##move.linear.x = 0.25
			  #move.angular.z = 0.0
		else:
			if data.ranges[300]>thresh:
				move.linear.x = -0.1
				#move.angular.z = 0
				move.angular.z = velocity*0.5
			elif data.ranges[40]>thresh:
				move.linear.x = -0.1
				#move.angular.z = 0
				move.angular.z = -velocity*0.5
			else:
				move.linear.x = 0.0
				#move.angular.z = 0.5
				move.angular.z = velocity
	else:
		
		rospy.loginfo("")

		rospy.loginfo("1 = " + str(data.ranges[1]))
		rospy.loginfo("90 = " + str(data.ranges[90]))
		rospy.loginfo("right_most = " + str(data.ranges[270]))
		rospy.loginfo("180 = " + str(data.ranges[180]))

		#front_average = scan[16:-16]
		left_front = data.ranges[-45:-1]
		right_front = data.ranges[1:45]

		right_average = sum(right_front)/len(right_front)
		left_average = sum(left_front)/len(left_front)

		rospy.loginfo("right_average = " + str(right_average))
		rospy.loginfo("left_average = " + str(left_average))

		right_most = data.ranges[270] 



		#print("left_front is %s" % left_front)
		#print("right_front is %s" % right_front)


		#front_average = (sum(left_front)+sum(right_front))/(len(left_front)+len(right_front)) #average distance of obstacles on the infront 

		front_average = (right_average+left_average)/2

		rospy.loginfo("front_average = " + str(front_average))

		front_list = data.ranges[-85:-1]+data.ranges[1:85]


		max_front = max(front_list)
		rospy.loginfo("max_front = " + str(max_front))

		min_front = min(front_list)
		rospy.loginfo("min_front = " + str(min_front))


		min_right_most = min(data.ranges[270:315]+data.ranges[225:270])
		rospy.loginfo("min_right_most = " + str(min_front))

		min_right = min(data.ranges[270:359])
		rospy.loginfo("min_right = " + str(min_right))


		min_full_front = min(data.ranges[270:359] + data.ranges[1:90])
		rospy.loginfo("min_full_front = " + str(min_full_front))


		if 0:
			if 0 and data.ranges[90] > 0.4:
				#move.linear.x = 0
				if data.ranges[1] > 0.3:
					move.linear.x = 0.05
				move.angular.z = 0.1 # left

			elif 0 and data.ranges[90] > 0.5:

				move.angular.z = -0.1 # right


			#elif data.ranges[270] < 0.2 or data.ranges[1] < 0.3:
			#elif data.ranges[270] < 0.2 or data.ranges[1] < 0.3:
			elif (right_most < 0.6 and 
				  (right_most < 0.2 or

				  	data.ranges[1] < 0.2 and (sum(right_front) > sum(left_front))

				  	or 

				  #right_average < 0.3 or

				  (front_average < 0.3 and front_average < 1000)

				  

				  )
				  ):

				rospy.loginfo("1.")

				#if data.ranges[1] < 0.3:
				move.linear.x = 0
				#else:
					#move.linear.x = 0.05

				move.angular.z = 0.1 # left

			#elif data.ranges[90] > 0.4:
			elif 1000 > right_most and right_most > 0.4:

				rospy.loginfo("2.")

				#if data.ranges[1] < 0.2:
				#	move.linear.x = -0.05
				#else:
				#	move.linear.x = 0.05
				move.linear.x = 0

				move.angular.z = -0.1 # right

			else:
				rospy.loginfo("3.")

				move.angular.z = 0
				move.linear.x = 0.1

		elif 0:


			#if min_front < 0.12:
			if 0 and min_front < 0.1:

				rospy.loginfo("Backward")

				move.linear.x = -0.1
				#move.angular.z = 0.1
				move.angular.z = 0.0

			elif (#min_front > 0.13 and
				 #go forward if parallel
				# data.ranges[280] > right_most and right_most < data.ranges[290] and 
				 
				#data.ranges[1] > 0.4 and front_average > 0.4 and right_average > 0.4 and left_average > 0.4

				#min_full_front > 0.4
				min_front > 0.13
				):

				rospy.loginfo("Forward")

				move.linear.x = 0.1
				move.angular.z = 0

			elif (
				#1000 > right_most and 
				#right_most > 0.4
				min_right_most > 0.4
				and 
				min_right > 0.12
				):

				rospy.loginfo("Right")

				move.linear.x = 0
				move.angular.z = 0.1 # right

			else:

				rospy.loginfo("Left")

				move.linear.x = 0
				move.angular.z = 0.2 # left

		else:

			if (front_average > 0.2
				#and
				#data.ranges[0] > 0.2
				#min_front > 0.12
				):

				rospy.loginfo("Forward")

				move.linear.x = 0.1

				if (left_average - right_average > 0.5):
					rospy.loginfo("Left")
					move.angular.z = -0.1 # left
				elif (left_average - right_average < -0.5):
					rospy.loginfo("Right")
					move.angular.z = 0.1 # left	

			else:

				rospy.loginfo("Left")

				move.linear.x = 0
				move.angular.z = 0.2 # left	



	pub.publish(move)


def avoidance(data):
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
		

	#front_average = scan[16:-16]
	left_front = scan[-45:-1]
	right_front = scan[1:45]

	right_most = scan[45:90]

	#print("left_front is %s" % left_front)
	#print("right_front is %s" % right_front)


	front_average = (sum(left_front)+sum(right_front))/(len(left_front)+len(right_front)) #average distance of obstacles on the infront 

	#print("front_average is %s" % front_average)
	#print("right is %s" % right)
	#print("left is %s" % left)

	#linear_vel = 0.45
	#linear_vel = 0.35
	linear_vel = 0.1
	angular_vel = 0

	error = left-right #estimating the error for P-Controlle

	if (#front_average < 0.5 
		front_average < 0.3
	    or scan[1] < 0.3
	    ):
		#linear_vel = -linear_vel*front_average
		linear_vel = 0


	elif front_average < 1:
		linear_vel = linear_vel*front_average
	
	#print("linear_vel is %s" % linear_vel)
	
	## tries to turn right when to far away from right wall
	## and when the bot is tilted away from the wall
	if (
		#0 and 
		front_average > 0.3
	    and scan[1] > 0.3
		):
		if (
			#scan[270] > 0.37
			data.ranges[270] > 0.37
			and
			#scan[280] - scan[270] > 0.07
			#scan[100] - scan[90] > 0.07
			data.ranges[270] - data.ranges[260] > 0.07
			):
			error = -0.2 # turn right
			linear_vel = 0

	if (
	0 and
	1
	):
		error = 0
		linear_vel = 0

	#move.angular.z = -0.1 # right
	
	rospy.loginfo("")
	rospy.loginfo("")
	rospy.loginfo("error = " + str(error))
	#rospy.loginfo("left_most = " + str(scan[90]))
	#rospy.loginfo("right_most = " + str(scan[270]))
	#rospy.loginfo("ahead_right_most = " + str(scan[280]))
	#rospy.loginfo("sub = " + str(scan[280]-scan[270]))

	rospy.loginfo("260 = " + str(data.ranges[260]))
	rospy.loginfo("270 = " + str(data.ranges[270]))
	rospy.loginfo("280 = " + str(data.ranges[280]))
	rospy.loginfo("270-260 = " + str(data.ranges[270]-data.ranges[260]))
	rospy.loginfo("")
	#rospy.loginfo("80 = " + str(data.ranges[80]))
	#rospy.loginfo("90 = " + str(data.ranges[90]))
	#rospy.loginfo("100 = " + str(data.ranges[100]))
	#rospy.loginfo("90-100 = " + str(data.ranges[90]-data.ranges[100]))
	rospy.loginfo("")
	#rospy.loginfo("front = " + str(scan[1]))
	#rospy.loginfo("left = " + str(left))
	#rospy.loginfo("right = " + str(right))
	#rospy.loginfo("linear_vel = " + str(linear_vel))
	#rospy.loginfo("front_average = " + str(front_average))

	#move.angular.z = -0.1 # right

	move.linear.x = linear_vel #linear velocity
	move.angular.z = angular_vel + PID(error) #angular velocity


	rospy.loginfo("move.linear.x = " + str(move.linear.x))
	rospy.loginfo("move.angular.z = " + str(move.angular.z))

	#move.angular.z = 1
	#print("Angular Velocity is %s" % move.angular.z)
	#print("")
	pub.publish(move)


def stoplidar(data):

	pass

def stop(data):

	move.linear.x = 0
	move.angular.z = 0
	pub.publish(move)

	pass



def follow(data): #defining a wall following function
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
		

	#front_average = scan[16:-16]
	left_front = scan[-45:-1]
	right_front = scan[1:45]

	#print("left_front is %s" % left_front)
	#print("right_front is %s" % right_front)


	front_average = (sum(left_front)+sum(right_front))/(len(left_front)+len(right_front)) #average distance of obstacles on the infront 

	#print("front_average is %s" % front_average)
	#print("right is %s" % right)
	#print("left is %s" % left)

	#linear_vel = 0.45
	#linear_vel = 0.35
	#linear_vel = 0.25
	linear_vel = 0.1
	angular_vel = 0

	
	if front_average < 0.5:
		#linear_vel = -linear_vel*front_average
		linear_vel = 0
	elif front_average < 1:
		linear_vel = linear_vel*front_average
	
	#print("linear_vel is %s" % linear_vel)
 
	error = left-right #estimating the error for P-Controller
	
	move.linear.x = linear_vel #linear velocity
	move.angular.z = angular_vel + PID(error) #angular velocity
	#move.angular.z = 1
	#print("Angular Velocity is %s" % move.angular.z)
	#print("")
	pub.publish(move)



class RealLineFollower(object):

	def __init__(self):
		self.bridge_object = CvBridge()
		#self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		#self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
		self.moveTurtlebot3_object = MoveTurtlebot3()
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
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		#self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
		self.moveTurtlebot3_object = MoveTurtlebot3()

		sub = rospy.Subscriber("/scan",LaserScan, stoplidar)

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

class CamFollower(object):

	def __init__(self):

		print("Running version 3")

		self.bridge_object = CvBridge()
		#self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		#self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
		self.moveTurtlebot3_object = MoveTurtlebot3()

		sub = rospy.Subscriber("/scan",LaserScan, stoplidar)

		move.linear.x = 0
		move.angular.z = 0

		pub.publish(move)


		self.lane_follow_pause_count = 10


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
		lower_yellow = np.array([10,25,25])
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
			twist_object.linear.x = 0.01
			#twist_object.angular.z = -(cx - width/2)/(100)
			#twist_object.angular.z = -(cx - width/2)/(150)

			if 0:
				if (cx - width/2) > 100:
					twist_object.angular.z = -0.25
				elif (cx - width/2) < -100:
					twist_object.angular.z = 0.25
				else:
					#twist_object.angular.z = 0
					twist_object.angular.z = -(cx - width/2)/(150)
			else:
				if (cx - width/2) > 200:
					twist_object.angular.z = -0.25
				elif (cx - width/2) < -200:
					twist_object.angular.z = 0.25
				else:
					#twist_object.angular.z = 0
					twist_object.angular.z = -(cx - width/2)/(250)


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
				sub = rospy.Subscriber("/scan",LaserScan, wallfollow)
				#while not rospy.is_shutdown():
				#    pub.publish(move)
				#    pass                

			#elif self.key_code ==  keyboard.Key.KEY_o:
			elif value == "o":
				print("o key was pressed!")
				
				#rospy.init_node('obstacle_avoidance')
				#pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
				sub = rospy.Subscriber("/scan",LaserScan, obstacleavoidance)

			#elif self.key_code ==  keyboard.Key.KEY_o:
			elif value == "a":
				print("a key was pressed!")
				
				#rospy.init_node('obstacle_avoidance')
				#pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
				sub = rospy.Subscriber("/scan",LaserScan, avoidance)


			elif value == "f":
				print("f key was pressed!")
				#wall_follow = WallFollowing()
				#wall_follow.wall_following()
				#rospy.init_node('wall_follower')
				
				#pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
				sub = rospy.Subscriber("/scan",LaserScan, follow)
				#while not rospy.is_shutdown():
				#    pub.publish(move)
				#    pass         

			elif value == "c":
				print("c key was pressed!")
				cam_follower_object = CamFollower()
				rate = rospy.Rate(5)

			elif value == "l":
				print("l key was pressed!")
				line_follower_object = LineFollower()
				rate = rospy.Rate(5)

			elif value == "r":
				print("r key was pressed!")
				line_follower_object = RealLineFollower()
				rate = rospy.Rate(5)                

			elif value == "s":

				sub = rospy.Subscriber("/scan",LaserScan, stop)

				rate = rospy.Rate(5)  

				print("s key was pressed!")

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

