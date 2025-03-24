#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from my_intf.msg import TuurtleArray
from my_intf.srv import TutuCatch
from geometry_msgs.msg import Twist
import math
from functools import partial
#pose gives the current orientation of turtel1. we gotta dump dat to "twist" which guides that thingy to required position

class turtle_destroyer(Node):
    def __init__(self):
        super().__init__("bad_boy_node")
        self.pose = None
        self.tutu_catch = None
        self.declare_parameter("turtle_catch_first", True)
        self.turtle_catch_first = self.get_parameter("turtle_catch_first").value
        self.i = 0
        self.subs = self.create_subscription(Pose, "turtle1/pose", self.turtle_cont_callb, 10)
        self.pubs = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.subs2 = self.create_subscription(TuurtleArray,"tutu_array", self.tutu_callb,10)
        self.timer = self.create_timer(0.01, self.control_loop)
        


    def turtle_cont_callb(self, msg):
        self.pose = msg
    
    def tutu_callb(self,msg):
        if not msg.turtles:
            return
        if self.turtle_catch_first:
            turtle_first_ = None
            turtle_distance = None

            for turtle in msg.turtles:
                dist_x = turtle.x - self.pose.x
                dist_y = turtle.y - self.pose.y
                distance = math.sqrt((dist_x**2) + (dist_y**2))
                if turtle_first_ == None or distance < turtle_distance:
                    turtle_first_ = turtle
                    turtle_distance = distance
                self.tutu_catch = turtle_first_

        else:
            self.tutu_catch = msg.turtles[0]
            self.get_logger().info("turtle has reached "+ self.tutu_catch.name)
        


    
    def control_loop(self):
        if self.pose == None or self.tutu_catch == None:
            return
        
        dist_x = self.tutu_catch.x - self.pose.x
        dist_y = self.tutu_catch.y - self.pose.y
        
        distance = math.sqrt((dist_x**2) + (dist_y**2))
        msg = Twist()

        if distance > 0.5:
            #msg.linear.x = min(20* distance, 1.5)
            msg.linear.x = 3*distance

            goal_theta = math.atan2(dist_y,dist_x)
            diff = goal_theta - self.pose.theta
            
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            #msg.angular.z = min(10* diff, 1.8)
            msg.angular.z = 6*diff
        

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.turtle_catcher_client(self.tutu_catch.name)
            self.tutu_catch = None

        self.pubs.publish(msg)

    def turtle_catcher_client(self, turtle_name):
        client = self.create_client(TutuCatch,'tutucatcher')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server")
        req = TutuCatch.Request()
        req.name = turtle_name
        future = client.call_async(req)
        future.add_done_callback(partial(self.callb, turtle_name=turtle_name))
    
    def callb(self,future,turtle_name):
        try:
            resp = future.result()
            resp.success = True
            return resp
        except Exception as e:
            self.get_logger().error("service call failed %r"%(e,))
            resp.success = False
            return resp

def main(args=None):
   rclpy.init(args=args)
   node = turtle_destroyer()
   rclpy.spin(node)
   rclpy.shutdown()
if __name__ == "__main__":
   main()
