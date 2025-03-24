#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

class Serial_arduino_sender(Node):
    def __init__(self):
        super().__init__('serial_data_sender')
        self.data = 0
        self.serial_port = None
        self.connect_to_arduino()
        self.timer = self.create_timer(0.01, self.arduino_send)
    
    def connect_to_arduino(self):
        ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
        for port in ports:
            try:
                self.serial_port = serial.Serial(port,115200,timeout=1)
                self.get_logger().info("connected to port " + port )
                return
            except serial.SerialException:
                pass
    
    def arduino_send(self):
        self.serial_port.write((str(self.data)+'\n').encode())
        self.get_logger().info(f"Sent: {str(self.data).strip()}")
        self.data += 1
    
def main(args=None):
    rclpy.init(args=args)
    node = Serial_arduino_sender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
