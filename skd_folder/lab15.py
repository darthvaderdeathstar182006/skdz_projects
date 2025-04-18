#! /usr/bin/env python
import rospy
from Newrro_Navigation import *
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
import math
from time import sleep

# Linear and angular velocity of the robot
Linear_velocity = 0.1  # Linear velocity in m/s
Angular_velocity = 0.8  # Angular velocity in rad/s

# Global variables for robot state
position_ = Point()  # Stores live odometry of the robot
yaw_precision_ = math.pi / 20  # +/- 2 degrees of error allowed
dist_precision_ = 0.05  # Goal point radius in meters
yaw_ = 0  # Stores the live orientation of the robot
state_ = 0  # Stores the state of the robot
x_goal = 0  # X coordinate of the goal point
y_goal = 0  # Y coordinate of the goal point
cmd_pub = None  # Publisher name declared globally

def clbk_odom(msg):
    global position_, yaw_
    # Position
    position_ = msg.pose.pose.position
    # Yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

# def fix_yaw(des_pos):
#     global yaw_, cmd_pub, yaw_precision_, state_
#     desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
#     err_yaw = desired_yaw - yaw_
#     twist_msg = Twist()
#     if math.fabs(err_yaw) > yaw_precision_:
#         twist_msg.angular.z = Angular_velocity if err_yaw > 0 else -Angular_velocity
#         cmd_pub.publish(twist_msg)
#     if math.fabs(err_yaw) <= yaw_precision_:
#         change_state(1)

# def go_straight_ahead(des_pos):
#     global yaw_, cmd_pub, yaw_precision_, state_
#     desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
#     err_yaw = desired_yaw - yaw_
#     err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
#     if err_pos > dist_precision_:
#         twist_msg = Twist()
#         twist_msg.linear.x = Linear_velocity
#         cmd_pub.publish(twist_msg)
#     else:
#         change_state(2)
#     if math.fabs(err_yaw) > yaw_precision_:
#         change_state(0)

def move_straight():
    twist_msg = Twist()
    twist_msg.linear.x = Linear_velocity
    cmd_pub.publish(twist_msg)

def turn_left():
    twist_msg = Twist()
    twist_msg.angular.z = Angular_velocity
    cmd_pub.publish(twist_msg)

def turn_right():
    twist_msg = Twist()
    twist_msg.angular.z = -Angular_velocity
    cmd_pub.publish(twist_msg)

# def stop():
#     twist_msg = Twist()
#     twist_msg.linear.x = 0
#     twist_msg.angular.z = 0
#     cmd_pub.publish(twist_msg)

def change_state(state):
    global state_
    state_ = state
    print('State changed to [%s]' % state_)

def goal(desired_pose):
    global yaw_, cmd_pub, yaw_precision_, state_
    print("-------------------------")
    print("Goal_point : " + str(round(desired_pose.x, 1)) + ", " + str(round(desired_pose.y, 1)))
    print("-------------------------")
    print("*** NAVIGATION STARTED ***")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state_ == 0:
            fix_yaw(desired_pose)
        elif state_ == 1:
            go_straight_ahead(desired_pose,yaw_,position_,Linear_velocity)
        elif state_ == 2:
            stop()
            break
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()

def navigation():
    global x_goal, y_goal
    x_goal = float(input("Enter x coordinate: "))
    y_goal = float(input("Enter Y coordinate: "))
    print("")
    desired_position = Point()
    desired_position.x = x_goal
    desired_position.y = y_goal
    goal(desired_position)

def main():
    global Linear_velocity, Angular_velocity, cmd_pub
    rospy.init_node('NR_B1')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, clbk_odom)
    print("Publishers: /cmd_vel")
    print("Subscribers: /robot_pose_ekf/odom_combined")
    print("Linear_velocity: " + str(Linear_velocity) + " m/s")
    print("Angular_velocity: " + str(Angular_velocity) + " rad/s")
    print("")
    sleep(1)
    
    

if __name__ == '__main__':
    main()
    navigation()
