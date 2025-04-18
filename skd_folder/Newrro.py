#! /usr/bin/env python3

import rospy
from time import sleep
from STservo_sdk import * 
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist        
from time import sleep     

selector_r  = 0
selector_l  = 0

Left_ticks  = 0
Right_ticks = 0

L_velocity     = 0
R_velocity    = 0

prev_left_ticks   = 0
prev_right_ticks  = 0

total_left_ticks  = 0
total_right_ticks = 0

encoder_maximum = 32000
BAUDRATE        = 115200

# def check_motor_port(id,check_baudrate):
#     ports = ["/dev/ttyUSB0","/dev/ttyUSB1"]
#     for port in ports:
#         print("checking port : ", port)
#         port_handler = PortHandler(port)
#         if not port_handler.openPort():
#             print("failed to openport : ",port)
#             continue
#         if not port_handler.setBaudRate(check_baudrate):
#             print("failed to set baudrate : ",port)
#             continue
#         packet_handler = sts(port_handler)
#         try:
#             check_result = packet_handler.ping(id)
#             if check_result[1] == 0:
#                 print("Motor found on port : ",port)
#                 port_handler.closePort()
#                 return port
#             else:
#                 print("Ping failed on port : ",port)
#         except Exception as e:
#             print("error found : ",e)
#         finally:
#             port_handler.closePort()
#             del port_handler
#             del packet_handler
#     print("No motor Found")
#     return None

# DEVICENAME    = check_motor_port(1,BAUDRATE)

DEVICENAME    = "/dev/ttyUSB1"
portHandler   = PortHandler(DEVICENAME) 
packetHandler = sts(portHandler)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

def wheel_mode(Motor_ID):
    Result, Error = packetHandler.WheelMode(Motor_ID)
    if Result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(Result))
    if Error != 0:
        print("%s" % packetHandler.getRxPacketError(Error))

def Run_Motor(Motor_ID , Motor_Speed , Motor_Accel):
    Result, Error = packetHandler.WriteSpec(Motor_ID, Motor_Speed, Motor_Accel)
    if Result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(Result))
    if Error != 0:
        print("%s" % packetHandler.getRxPacketError(Error))

def present_pos(Motor_ID):
    Position, Speed, Result, Error = packetHandler.ReadPosSpeed(Motor_ID)
    if Result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(Result))
    if Error != 0:
        print(packetHandler.getRxPacketError(Error))
    return Position

def ENABLE_WHEEL_MODE():
    wheel_mode(1)
    wheel_mode(2)
    wheel_mode(3)
    wheel_mode(4)

def right_tick_count(R_velocity):
    global Right_ticks , total_right_ticks ,prev_right_ticks , selector_r
    Right_ticks = int(present_pos(2)/2.5)

    if R_velocity > 0:
        selector_r = 1
    elif R_velocity < 0:
        selector_r = 2

    if selector_r == 1 and prev_right_ticks < Right_ticks:
        if total_right_ticks >= encoder_maximum:
            total_right_ticks = 0
        else:
            total_right_ticks += abs(Right_ticks - prev_right_ticks)
    elif selector_r == 2 and prev_right_ticks > Right_ticks:
        if total_right_ticks <= 0:
            total_right_ticks = encoder_maximum
        else:
            total_right_ticks += (Right_ticks - prev_right_ticks)

    prev_right_ticks = Right_ticks
    Right_ticks = total_right_ticks
    # right_ticks_pub.publish(Right_ticks)
    return Right_ticks

def left_tick_count(L_velocity):
    global  Left_ticks , total_left_ticks , prev_left_ticks , selector_l
    Left_ticks = int(present_pos(1)/2.5)
    if L_velocity > 0:
        selector_l = 1
    elif L_velocity < 0:
        selector_l = 2

    if selector_l == 1 and prev_left_ticks > Left_ticks:
        if total_left_ticks >= encoder_maximum:
            total_left_ticks = 0
        else:
            total_left_ticks += abs(Left_ticks - prev_left_ticks)
    elif selector_l == 2 and prev_left_ticks < Left_ticks:
        if total_left_ticks <= 0:
            total_left_ticks = encoder_maximum
        else:
            total_left_ticks -= (Left_ticks - prev_left_ticks)
    
    prev_left_ticks = Left_ticks
    Left_ticks = total_left_ticks
    # left_ticks_pub.publish(Left_ticks)   
    return Left_ticks
