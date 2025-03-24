#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from my_intf.msg import TuurtleArray
from my_intf.msg import Tutu
from my_intf.srv import TutuCatch
import random
import math
from functools import partial

class turtle_spawner(Node):
    def __init__(self):
       super().__init__("robo_spawnner")
       self.pub = self.create_publisher(TuurtleArray,"tutu_array",10)
       self.declare_parameter("turtle_name", "turtle")
       self.turtle_prefix = self.get_parameter("turtle_name").value
       self.declare_parameter("spawn_frequency", 1.0)
       period = self.get_parameter("spawn_frequency").value
       self.counter = 1
       self.alive_turtle = []
       self.tutu_service = self.create_service(TutuCatch,"tutucatcher",self.tutu_del_name)
       self.timer = self.create_timer(1/period,self.new_turtle_spawnner)     
       
    def alive_callb(self):
       msg = TuurtleArray()
       msg.turtles = self.alive_turtle
       self.pub.publish(msg)

    def new_turtle_spawnner(self):
       x = random.uniform(1.0,10.0)
       y = random.uniform(1.0,10.0)
       theta = random.uniform(0,2*math.pi)
       self.turtle_enters(x,y,theta)

    def turtle_enters(self, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        self.counter += 1
        name = self.turtle_prefix+str(self.counter)
        while not client.wait_for_service(1.0):
          self.get_logger().warn("waiting for server.....")
        req = Spawn.Request()
        req.name = name 
        req.x = x
        req.y = y
        req.theta = theta
        future = client.call_async(req)
        future.add_done_callback(partial(self.callb, name=name, x=x, y=y, theta=theta))
    
    def callb(self,future,name,x,y,theta):
       try:
          resp = future.result()
          if resp.name != '':
             self.get_logger().info(f"{self.turtle_prefix}and{str(self.counter)}" +resp.name+" is aliveeeee")
             tutu = Tutu()
             tutu.x = x
             tutu.y = y
             tutu.theta = theta
             tutu.name = name
             self.alive_turtle.append(tutu)
             
             self.alive_callb()

        
       except Exception as e:
          self.get_logger().error("service call failed %r"%(e,))

    def tutu_del_name(self,req,resp):
       self.tutu_kill_done(req.name)
       resp.success = True
       return resp  
    
    def tutu_kill_done(self,tutu_name):
       client2 = self.create_client(Kill, "kill")
       client3 = self.create_client(Empty, "/clear")     
       while not client2.wait_for_service(1.0) or not client3.wait_for_service(1.0):
          self.get_logger().warn("waiting for server.....")
       req = Kill.Request()
       req1 = Empty.Request()
       req.name = tutu_name
       future = client2.call_async(req)
       future1 = client3.call_async(req1)
       future.add_done_callback(partial(self.call2b, tutu_name=tutu_name))
       future1.add_done_callback(self.call3b)

    def call2b(self,future,tutu_name):
       
      try:
          resp = future.result()
          for (i,tutu2) in enumerate(self.alive_turtle):
                if tutu_name == tutu2.name:
                   del self.alive_turtle[i]
                   self.alive_callb()
                   break
          return resp
      
      except Exception as e:
            self.get_logger().error("service call failed %r"%(e,))
            resp.success = False
            return resp
    
    def call3b(self,future1):
       try:
          future1.result()
      
       except Exception as e:
            self.get_logger().error("service call failed %r"%(e,))
                
def main(args=None):
   rclpy.init(args=args)
   node = turtle_spawner()
   rclpy.spin(node)
   rclpy.shutdown()
if __name__ == "__main__":
   main()