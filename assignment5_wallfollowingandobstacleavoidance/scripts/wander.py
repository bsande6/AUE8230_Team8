#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

def get_max(range_list):
    max_value = max(range_list)
    max_index = range_list.index(max_value)
    return max_index

class ObstacleAvoidance():
    def __init__(self):
        # Starts a new node
        rospy.init_node('wall_following', anonymous=True)
        self.sector_degrees = 5
        self.safe_side_distance = 0.7
        self.left_scan_distance = float('inf')
        self.right_scan_distance = float('inf')
        self.front_left_scan_distance = float('inf')
        self.front_right_scan_distance = float('inf')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.min_distance = 1.2
        self.min_side_distance = 0.2
        self.front_scan_distance = float('inf')
        self.vel_msg = Twist()
        self.r = rospy.Rate(10)
        
    def wander(self):
        self.vel_msg.linear.x = 0.2
        while not rospy.is_shutdown():
            #values = [self.front_scan_distance, self.front_left_scan_distance, self.left_scan_distance, self.front_right_scan_distance, self.right_scan_distance]
            #max_index = get_max(values)
            rospy.loginfo(self.left_scan_distance)
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

            # if max_index == 0:
            #     self.go_forward()
            # elif max_index == 1:
            #     self.angle_left()
            # elif max_index == 2:
            #     self.turn_left()
            # elif max_index == 3:
            #     self.angle_right()
            # else:  
            #     self.turn_right()
            
                # if self.front_left_scan_distance < self.min_distance and front_right_scan_distance > self.min_distance:
                #     self.vel_msg.angular.z =  math.atan2(self.front_left_scan_distance - self.goal_distance, (self.front_left_scan_distance * math.cos(45) + self.wall_distance - self.left_scan_distance))

                # elif self.front_right_scan_distance < self.min_distance and front_left_scan_distance > self.min_distance:
                
                #     self.vel_msg.angular.z =  math.atan2(self.front_right_scan_distance - self.goal_distance, (self.front_right_scan_distance * math.cos(315) + self.wall_distance - self.right_scan_distance))

                # elif self.front_scan_distance > self.min_distance:
                #     self.go_forward()

            #self.vel_msg.angular.z = self.front__scan_distance
            self.velocity_publisher.publish(self.vel_msg)

    def go_forward(self):
        self.vel_msg.linear.x = 0.2
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
    def angle_right(self):
        self.vel_msg.linear.x = 0.2
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = -0.03
        if self.front_scan_distance < 1:
            self.vel_msg.angular.z += -0.07
        elif self.front_left_scan_distance < self.min_side_distance:
            self.vel_msg.angular.z += -0.05

    def angle_left(self):
        self.vel_msg.linear.x = 0.2
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0.03
        if self.front_scan_distance < 1:
            self.vel_msg.angular.z += 0.07
        elif self.front_right_scan_distance < self.min_side_distance:
            self.vel_msg.angular.z += 0.05
        
    def turn_left(self):
        self.vel_msg.linear.x = 0.2
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0.9

    def turn_right(self):
        self.vel_msg.linear.x = 0.2
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

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.wander()
    except rospy.ROSInterruptException: pass















