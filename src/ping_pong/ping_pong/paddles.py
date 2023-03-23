#! usr/bin/env python3

## library imports
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen, Kill, Spawn
from std_srvs.srv import Empty
from functools import partial

class Paddles(Node):
    
    def __init__(self):
        super().__init__("paddles")

        self.get_logger().info("Beginning game")
        self._initiate()
          

    def _initiate(self):

        client = self.create_client(Empty,"turtlesim1/clear")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("...Waiting for service clear...")

        request = Empty.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_service))

        client = self.create_client(Kill,"turtlesim1/kill")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("...Waiting for service kill...")

        request = Kill.Request()
        request.name = 'turtle1'
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_service))

        self.spawn_player_turtle(1.0,5.5,1.57,'player_1')
        self.spawn_player_turtle(10.0,5.5,1.57,'player_2')

    def spawn_player_turtle(self,x,y,theta,name):
        
        client = self.create_client(Spawn,"turtlesim1/spawn")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("...Waiting for service spawn...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(
            self.callback_service))
        
        client = self.create_client(SetPen,"turtlesim1/"+name+"/set_pen")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("...Waiting for service set pen...")
        
        request = SetPen.Request()
        request.off = 1

        future = client.call_async(request)
        future.add_done_callback(partial(
            self.callback_service))
        

    def callback_service(self, future):
        
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % {e,})


def main(args=None):
    rclpy.init(args=args)
    node= Paddles()
    rclpy.spin(node)
    rclpy.shutdown()
