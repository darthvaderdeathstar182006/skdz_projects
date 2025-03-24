#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String


class Serial_Publisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout = 1)
        self.pub = self.create_publisher(String, 'serial_publisher_data', 10)
        self.timer = self.create_timer(0.01, self.arduino_publish)
    
    def arduino_publish(self):
        
        if self.serial_port.in_waiting > 0:
           # self.get_logger().info(f'serial arduino info has published data:')
            try:
                info = self.serial_port.readline().decode().strip()
                #self.get_logger().info(f'serial arduino info has published data:')
                msg = String()
                msg.data = info
                self.get_logger().info(f'serial arduino data has published data: {msg.data}')
                self.pub.publish(msg)
                
            except ValueError:
                self.get_logger().info("dude wtf is this!!!")
                

def main(args=None):
    rclpy.init(args=args)
    node = Serial_Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

