#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math

class WallFollowing():
    def __init__(self):
        # Starts a new node
        rospy.init_node('wall_following', anonymous=True)
        self.Kp = .15
        self.Kd = 0
        self.Ki = 0.01
        self.sector_degrees = 5
        self.goal_distance = 0.90
        self.wall_distance = 2.7
        self.left_scan_distance = self.goal_distance
        self.theta_scan_distance = self.goal_distance
        self.theta = math.radians(70)
        self.right_scan_distance = self.goal_distance
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
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
        self.vel_msg.linear.x = 0.0
        time.sleep(5)
    
        while not rospy.is_shutdown():
            self.vel_msg.linear.x = 0.3
            self.update_control()
            self.velocity_publisher.publish(self.vel_msg)

    def update_control(self):
        self.vel_msg.angular.z =  math.atan2(self.theta_scan_distance - self.goal_distance, (self.theta_scan_distance * math.cos(self.theta) + self.wall_distance - self.left_scan_distance))
        
        if math.isnan(self.vel_msg.angular.z):
            exit()

    def scan_callback(self, msg):
        self.left_scan_distance = self.find_left_side(msg.ranges)
        self.theta_scan_distance = self.find_theta_sector(msg.ranges)
           
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
            quarter = float((middle - 0.5)/2)
        else:
            quarter = float(middle/2)

        if quarter % 2 != 0:
            return input_list[int(quarter - 0.5 * 3)]
        else:
            return input_list[int(quarter)* 3]

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
        wall_following = WallFollowing()
        wall_following.wall_following()
    except rospy.ROSInterruptException: pass















