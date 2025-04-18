#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import websockets
import json
from std_msgs.msg import Int32MultiArray 
import threading

class espconnect(Node):
    def __init__(self):
        super().__init__("__sender_node__")
        self.subs = self.create_subscription(Twist, "/turtle1/cmd_vel", self.callb2, 10)
        self.loop = asyncio.get_event_loop()
        
        self.websocket_clients = set()
        self.get_logger().info("Websocket Server running on ws://localhost:8765")
        self.last_ticks = {"left_ticks": 0, "right_ticks": 0}
        self.encoder_publisher = self.create_publisher(Int32MultiArray, "encoder_ticks", 10) 
        self.tutu_publisher = self.create_publisher(Twist, "bro_direction", 10) 
        #self.timer = self.create_timer(0.1, self.turner)


    async def websocket_handler(self, websocket, path):
        self.websocket_clients.add(websocket)
        # x = 2
        # z = 0
        # cmd = f'{{"linear_x": {x}, "angular_z": {z}}}'
        # asyncio.create_task(self.send_to_clients(cmd))
        self.get_logger().info("Esp32 connecteddddd")
        try:
            async for message in websocket:
                self.process_encoder_data(message)
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("esp32 disconnected")
        finally:
            self.websocket_clients.remove(websocket)
        

    def process_encoder_data(self, message):
        try:
            self.get_logger().info(f"Raw message received: {message}")  # Print raw data
            data = json.loads(message)
            self.last_ticks["left_ticks"] = data.get("left_tick", 0)
            self.last_ticks["right_ticks"] = data.get("right_ticks", 0)
            self.get_logger().info(f"Received Encoder Data: {self.last_ticks}")
            encoder_msg = Int32MultiArray()
            encoder_msg.data = [self.last_ticks["left_ticks"], self.last_ticks["right_ticks"]]
        
            self.encoder_publisher.publish(encoder_msg)
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse incoming JSON")
        
        
    
    async def send_to_clients(self, msg):
        self.tutu_publisher.publish(msg)
        if self.websocket_clients:
            x = msg.linear.x
            z = msg.angular.z
            cmd = f'{{"linear_x": {x}, "angular_z": {z}}}'
                      
            await asyncio.gather(*(ws.send(cmd) for ws in self.websocket_clients))

    async def run_server(self):
        server = await websockets.serve(self.websocket_handler,'0.0.0.0', 8765)
        self.get_logger().info("WebSocket server started on port 8765")
        await server.wait_closed()
    
    def callb2(self, msg):
        # self.get_logger().info("startedddd turtleeeeeeeeeeee.......")
        # x = msg.linear.x
        # z = msg.angular.z
        # self.tutu_publisher.publish(msg)
        # self.get_logger().info(f"{str(x)} and {str(z)}")
        asyncio.run_coroutine_threadsafe(self.send_to_clients(msg), self.loop)

        # asyncio.create_task(self.send_to_clients(msg))





    



def main(args=None):
    rclpy.init(args=args)
    node = espconnect()

    # Store the asyncio loop
    loop = asyncio.get_event_loop()
    node.loop = loop

    # Run ROS in separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        loop.run_until_complete(node.run_server())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
   main()




