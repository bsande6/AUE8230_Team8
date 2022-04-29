#!/usr/bin/env python3
	
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
	
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

    print("front is %s" % front)
    print("right is %s" % right)
    print("left is %s" % left)

    #linear_vel = 0.45
    linear_vel = 0.35
    angular_vel = 0

    
    if front < 0.5:
        #linear_vel = -linear_vel*front
        linear_vel = 0
    elif front < 1:
        linear_vel = linear_vel*front
    
    print("linear_vel is %s" % linear_vel)
 
    error = left-right #estimating the error for P-Controller
    
    move.linear.x = linear_vel #linear velocity
    move.angular.z = angular_vel + PID(error) #angular velocity
    #move.angular.z = 1
    print("Angular Velocity is %s" % move.angular.z)
    print("")

rospy.init_node('wall_follower')
move = Twist()
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
sub = rospy.Subscriber("/scan",LaserScan, wallfollow)
while not rospy.is_shutdown():
    pub.publish(move)
    pass
rospy.spin()
