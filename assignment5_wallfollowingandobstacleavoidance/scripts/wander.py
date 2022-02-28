#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class WallFollowing():
    def __init__(self):
        # Starts a new node
        rospy.init_node('wall_following', anonymous=True)
        self.safe_side_distance = 0.7
        self.left_scan_distance = float('inf')
        self.right_scan_distance = float('inf')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.min_distance = 1.0
        self.min_side_distance = 0.2
        self.front_scan_distance = float('inf')
        self.vel_msg = Twist()
        self.r = rospy.Rate(10)
        
    def wall_following(self):
        self.vel_msg.linear.x = 0.1
        
        while not rospy.is_shutdown():
            if self.front_scan_distance > self.min_distance:
                if self.left_scan_distance < self.safe_side_distance and self.right_scan_distance > self.safe_side_distance:
                    self.angle_right()
                elif self.right_scan_distance < self.safe_side_distance and self.left_scan_distance > self.safe_side_distance:
                    self.angle_left()
                else:
                    self.go_forward()

            else:
                if self.right_scan_distance < self.left_scan_distance:
                    self.turn_left()
                else: 
                    self.turn_right()

            self.velocity_publisher.publish(self.vel_msg)

    def go_forward(self):
        self.vel_msg.linear.x = 0.1
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
    def angle_right(self):
        self.vel_msg.linear.x = 0.1
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = -0.03
        if self.front_scan_distance < 1:
            self.vel_msg.angular.z += -0.07
        elif self.left_scan_distance < self.min_side_distance:
            self.vel_msg.angular.z += -0.03

    def angle_left(self):
        self.vel_msg.linear.x = 0.1
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0.03
        if self.front_scan_distance < 1:
            self.vel_msg.angular.z += 0.07
        elif self.right_scan_distance < self.min_side_distance:
            self.vel_msg.angular.z += 0.03
        
    def turn_left(self):
        self.vel_msg.linear.x = 0.1
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0.9

    def turn_right(self):
        self.vel_msg.linear.x = 0.1
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = -0.9

    
        
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
        self.front_scan_distance = msg.ranges[0]
        #self.front_scan = self.find_front(msg.ranges)
        # self.left_scan = self.find_left_side(msg.ranges)
        # self.right_scan = self.find_right_side(msg.ranges)
      
    # def find_front(self, input_list):
    #     sum = 0
    #     for i in range (0, 10):
    #         sum += input_list.ranges[i]
    #     for i in range (350, 360):
    #         sum += input_list.ranges[i]
    #     return sum/20
        
    def find_right_side(self, input_list):
        
        middle = float(len(input_list))/2
        if middle % 2 != 0:
            quarter = float((middle - 0.5)/2)
        else:
            quarter = float(middle/2)

        if quarter % 2 != 0:
            quarter = int(quarter - 0.5)
            return input_list[int(quarter - 0.5 * 3)]
        else:
            return input_list[int(quarter)* 3]
        # sum = 0
        # for i in range(quarter, quarter+10):
        #     sum += input_list.ranges[i]
        # for i in range()
        #     sum += input_list.ranges[i]

        # return sum/20
       

    def find_left_side(self, input_list):
        middle = float(len(input_list))/2
        if middle % 2 != 0:
            quarter = float((middle - 0.5)/2)
        else:
            quarter = float(middle/2)

        if quarter % 2 != 0:
            quarter = int(quarter - 0.5)

        # sum = 0
        # for i in range(quarter, quarter+10):
        #     sum += input_list.ranges[i]
        # for i in range()
        #     sum += input_list.ranges[i]

        # return sum/20
       
            return input_list[int(quarter - 0.5)]
        else:
            return input_list[int(quarter)]

        # if(self.data_left > limit):
        #     new_twist.angular.z = 1
        #     rospy.loginfo('Data-Left['+str(self.data_left)+'] > ' + str(limit))
        # else:
        #     new_twist.angular.z = 0
        #     rospy.loginfo('Data-Left['+str(self.data_left)+'] < ' + str(limit))


if __name__ == '__main__':
    try:
        wall_following = WallFollowing()
        wall_following.wall_following()
    except rospy.ROSInterruptException: pass















