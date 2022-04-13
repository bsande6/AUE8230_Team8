#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time
import math
import sys
from keyboard.msg import Key
from std_msgs.msg import Int32
#from keyboard_press import KeyboardManager


class Master():
    def __init__(self):
        rospy.init_node('master', anonymous=True)
        #self.launch_subscriber = rospy.Publisher('/launch_abort', Int32, self.launch_callback)
        self.wall_follow_publisher = rospy.Publisher('/wall_follow', String, queue_size=10)
        self.keyboard_sub = rospy.Subscriber('/keyboard/keydown', Key, self.get_key, queue_size=1)
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

            # Check if any key has been pressed
            if self.key_code ==  Key.KEY_w:
                print("w key was pressed!")
                wall_follow = WallFollowing()
                wall_follow.wall_following()

            # Reset the code
            if self.key_code != -1:
                self.key_code = -1

            # Sleep for the remainder of the loop
            rate.sleep()
       
    def control_switch(self):
        pass
        

class WallFollowing():
    def __init__(self):
        #rospy.init_node('wall_following', anonymous=True)
        self.Kp = .15
        self.Kd = 0
        self.Ki = 0.01
        self.sector_degrees = 5
        self.goal_distance = 0.2
        self.wall_distance = 1
        self.left_scan_distance = self.goal_distance
        self.theta_scan_distance = self.goal_distance
        self.theta = math.radians(70)
        self.right_scan_distance = self.goal_distance
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        #self.wall_follow_subscriber = rospy.Subscriber('/wall_follow', String, self.control_callback)
        self.min_distance = 1.0
        self.timestamp_diff = 0
        self.stamp = 0
        self.prev_stamp = 0
        self.min_side_distance = 0.2
        self.front_distance = float('inf')
        self.vel_msg = Twist()
        self.r = rospy.Rate(10)
        self.prev_error = 0
        
    def wall_following(self):

        while not rospy.is_shutdown():
            self.vel_msg.linear.x = 0.05
            # if self.left_scan_distance < 0.5:
            #     self.goal_distance = (self.left_scan_distance + self.right_scan_distance)/2
            # if self.goal_distance > 1:
            #     self.goal_distance = 1
            self.update_control()
            if self.right_scan_distance < 0.5:
                self.vel_msg.angular.z += 0.2
            if self.left_scan_distance < 0.5:
                self.vel_msg.angular.z -= 0.2
            if self.left_scan_distance >= 4 and self.right_scan_distance >= 4:
                self.vel_msg.angular.z = 0
            
            self.goal_distance = 0.4
            self.velocity_publisher.publish(self.vel_msg)
            rospy.sleep(1)

    def update_control(self):
        self.vel_msg.angular.z =  math.atan2(self.theta_scan_distance - self.goal_distance, (self.theta_scan_distance * math.cos(self.theta) + self.wall_distance - self.left_scan_distance))
    
        if math.isnan(self.vel_msg.angular.z):
            self.vel_msg.angular.z = 0
        if self.vel_msg.angular.z > 0.6:
            self.vel_msg.angular.z = 0.6

    def scan_callback(self, msg):
        self.left_scan_distance = self.find_left_side(msg.ranges)
        self.theta_scan_distance = self.find_theta_sector(msg.ranges)
        self.right_scan_distance = self.find_right_side(msg.ranges)
           
    def findMiddle(self, input_list):
        middle = float(len(input_list))/2
        if middle % 2 != 0:
            return input_list[int(middle - .5)]
        else:
            return (input_list[int(middle)], input_list[int(middle-1)])

        if(self.data_left > limit):
            new_twist.angular.z = 1
            rospy.loginfo('Data-Left['+str(self.data_left)+'] > ' + str(limit))
        else:
            new_twist.angular.z = 0
            rospy.loginfo('Data-Left['+str(self.data_left)+'] < ' + str(limit))

    def find_right_side(self, input_list):
        middle = float(len(input_list))/2
        if middle % 2 != 0:
            quarter = float((middle/2  + middle/4 - 0.5))
        else:
            quarter = float(middle/2 + middle/4)
        
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

    def find_theta_sector(self, input_list):
        left_theta_mean = 0
        count = 0
        for i in range(int(math.degrees(self.theta) - self.sector_degrees), int(math.degrees(self.theta) + self.sector_degrees)):
            count += 1
            left_theta_mean += input_list[int(i)]
        left_theta_mean = left_theta_mean / count
        return left_theta_mean

if __name__ == '__main__':
    try:
        master = Master()
        rospy.spin()
    except rospy.ROSInterruptException: pass