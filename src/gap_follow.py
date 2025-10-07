#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class WallFollowNode:
    def __init__(self):
        self.current_x = 0
        self.current_y = 0
        self.current_heading = 0
        self.ranges_array = []
        self.steering_smoothing_factor = 0.1
        self.previous_steering_angle = 0.0
        self.alignment_complete = False
        self.kp = 1  

        rospy.init_node('wall_follow_node', anonymous=True)
        self.drive_pub = rospy.Publisher('/car_1/command', AckermannDrive, queue_size=10)
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/car_1/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/car_1/scan', LaserScan, self.laser_callback)

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y

        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
        _, _, self.current_heading = euler_from_quaternion(quaternion)

    def laser_callback(self, laser_msg):
        self.ranges_array = laser_msg.ranges

    def run(self):
        while not rospy.is_shutdown():
            if len(self.ranges_array) == 0:
                continue

            if not self.alignment_complete:
                steering_angle = self.align_to_center()
                if steering_angle is None:
                    self.alignment_complete = True
                    continue
            else:
                target_distance = 1.0  
                distance_from_wall = np.mean(self.ranges_array[len(self.ranges_array)//4:len(self.ranges_array)//2])  
                error = distance_from_wall - target_distance
                steering_angle = self.kp * error 

            drive_msg = AckermannDrive()
            drive_msg.speed = 1.5 
            drive_msg.steering_angle = float(steering_angle)  
            self.drive_pub.publish(drive_msg)

            self.rate.sleep()

    def align_to_center(self):
        left_dist = np.mean(self.ranges_array[len(self.ranges_array)//4:len(self.ranges_array)//2])
        right_dist = np.mean(self.ranges_array[len(self.ranges_array)//2:3*len(self.ranges_array)//4])
        print(left_dist)
        print(right_dist)

        error = left_dist - right_dist
        desired_heading = np.arctan2(error, 1.0)  
        heading_change = desired_heading - self.current_heading

        L = 0.324  
        Lf = 0.5  
        max_steering_angle = np.radians(30)  
        steering_angle = np.arctan2(2 * L * np.sin(heading_change), Lf)
        steering_angle = np.clip(steering_angle, -max_steering_angle, max_steering_angle)
        
        if abs(error) < 0.1:
            return None  
        else:
            return steering_angle

if __name__ == '__main__':
    wall_follow_node = WallFollowNode()
    wall_follow_node.run()