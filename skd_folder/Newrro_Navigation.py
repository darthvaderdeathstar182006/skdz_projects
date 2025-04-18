#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
import math
from time import sleep

yaw_precision_      = math.pi / 20 # +/- 2 degrees of error is allowed when robot is navigating in s straight line
dist_precision_     = 0.05         # Goal point radius in meters

state_              = 0  #Tells the state of the robot if state_ == 0: robot is rotating , if state_ == 1: Robot moving straight , if state == 2: Reached goal point

cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

########################## ROBOT ORIENTATION AND GO STRAIGHT LOGICS ########################################

def stop():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmd_pub.publish(twist_msg)
    
def fix_yaw(des_pos , yaw_ , position_ , ang):
    global yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = ang if err_yaw > 0 else -ang
    
    cmd_pub.publish(twist_msg)
    
    if math.fabs(err_yaw) <= yaw_precision_:
        state_ = 1
        print("Robot State : " + str(state_))
    return state_

def go_straight_ahead(des_pos , yaw_ , position_ , lin):
    global  yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = lin
        cmd_pub.publish(twist_msg)
    else:
        state_ = 2
        print("Robot State : " + str(state_))
    
    if math.fabs(err_yaw) > yaw_precision_:
        state_ = 0
        print("Robot State : " + str(state_))

def move_straight(Linear_velocity):
    global cmd_pub
    twist_msg = Twist()
    twist_msg.linear.x = Linear_velocity
    cmd_pub.publish(twist_msg)

def turn_left(Angular_velocity):
    global cmd_pub
    twist_msg = Twist()
    twist_msg.angular.z = Angular_velocity
    cmd_pub.publish(twist_msg)

def turn_right(Angular_velocity):
    global cmd_pub
    twist_msg = Twist()
    twist_msg.angular.z = -Angular_velocity
    cmd_pub.publish(twist_msg)
