#! usr/bin/env python3

import numpy as np
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn, TeleportAbsolute
from functools import partial

class Ball(Node):

    def __init__(self):
        super().__init__("ball")
        self.get_logger().info("Adding Ball turtle")
        self._initiate()
        
        self.ball_pose = None
        self.p1_pose = None
        self.p2_pose = None

        self.cmd_vel_publisher = self.create_publisher(
             Twist,'turtlesim1/ball/cmd_vel',10)
        self.timer = self.create_timer(0.2, self.timer_callback)
    
    def timer_callback(self):
        
        self.pose_subs_ball = self.create_subscription(
            Pose,"turtlesim1/ball/pose",self.get_ball_pose,10)
        self.pose_subs_p1 = self.create_subscription(
            Pose,"turtlesim1/player_1/pose",self.get_p1_pose,10)
        self.pose_subs_p2 = self.create_subscription(
            Pose,"turtlesim1/player_2/pose",self.get_p2_pose,10)
        # if (self.ball_pose is not None):
        #     self.get_logger().info(f'p1: x={self.p1_pose.x},y={self.p1_pose.y},') 
        #     self.get_logger().info(f'ball: x={self.ball_pose.x},y={self.ball_pose.y},') 

    
    def _initiate(self):

        client = self.create_client(Spawn,"turtlesim1/spawn")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("...Waiting for service spawn...")

        request = Spawn.Request()
        request.x = 5.5
        request.y = 5.5
        request.theta = np.random.rand(1).item()*6.28
        request.name = "ball"

        future = client.call_async(request)
        future.add_done_callback(partial(
            self.callback_service))
        
        client = self.create_client(SetPen,"turtlesim1/ball/set_pen")
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

    
    def check_player_collision(self,pose1:Pose,pose2:Pose,pose_b:Pose):
        
        ball_x = pose_b.x
        ball_y = pose_b.y
        ball_theta = pose_b.theta

        left_x = pose1.x
        left_y = pose1.y

        right_x = pose2.x
        right_y = pose2.y

        delta_x = 0.2
        delta_y = 0.5
        paddle_size = 2.0* delta_y
        max_bounce_angle = 5*3.14/12

        if (left_x - delta_x < ball_x and ball_x < left_x + delta_x):

            if (left_y - delta_y < ball_y and ball_y < left_y + delta_y):

                rel_intersect_y = (left_y + delta_y) - ball_y
                norm_rel_intersect_y = rel_intersect_y / paddle_size
                bounce_angle = norm_rel_intersect_y * max_bounce_angle
                self.set_abs_pose(ball_x, ball_y, bounce_angle)


        if (right_x - delta_x < ball_x and ball_x < right_x + delta_x):

            if (right_y - delta_y < ball_y and ball_y < right_y + delta_y):

                rel_intersect_y = (right_y + delta_y) - ball_y
                norm_rel_intersect_y = rel_intersect_y / paddle_size
                bounce_angle = norm_rel_intersect_y * max_bounce_angle
                
                if (bounce_angle > 0):
                    bounce_angle += 1.57
                else:
                    bounce_angle -= 1.57
                
                self.set_abs_pose(ball_x, ball_y, bounce_angle)


    def update_direction(self,pose):
    
        RIGHT=1;UP_RIGHT=2;UP_LEFT=3;LEFT=4;DOWN_LEFT=5;DOWN_RIGHT=6
        theta = pose.theta
        
        if theta==0.0:
            direction = RIGHT
        elif (0.0 < theta and theta < 1.57):
            direction = UP_RIGHT
        elif (1.57 < theta and theta < 3.14):
            direction = UP_LEFT
        elif (theta==3.14):
            direction = LEFT
        elif (-3.14 < theta and theta < -1.57):
            direction = DOWN_LEFT
        elif (-1.57 < theta and theta < 0.0):
            direction = DOWN_RIGHT
        return direction


    def set_abs_pose(self,x,y,theta):

        client = self.create_client(TeleportAbsolute,"turtlesim1/ball/teleport_absolute")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("...Waiting for service abs_tele...")

        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = client.call_async(request)
        future.add_done_callback(partial(
            self.callback_service))

    def set_vel(self,linear,angular,change=True):

        if change:
            cmd=Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.cmd_vel_publisher.publish(cmd)

    def get_p1_pose(self,pose:Pose):
        self.p1_pose = pose
        #self.get_logger().info(f'{self.p1_pose.x}')

    def get_p2_pose(self,pose:Pose):
        self.p2_pose = pose
    
    def get_ball_pose(self,pose:Pose):
        self.ball_pose = pose
        #self.get_logger().info(f'x={self.ball_pose.x},y={self.ball_pose.y},theta={self.ball_pose.theta}')
        

    def ball_move(self):

        pose1 = self.p1_pose
        pose2 = self.p2_pose
        pose_b = self.ball_pose

        if (pose_b is not None and pose1 is not None and pose2 is not None):

            x = pose_b.x
            y = pose_b.y
            theta = pose_b.theta

            #self.get_logger().info("inside ball move")
            self.set_vel(2.0, 0.0)
            self.check_player_collision(pose1,pose2,pose_b)
            direction = self.update_direction(pose_b)

            ## Update pose if ball hits the wall
            new_theta = 0.0
            RIGHT=1;UP_RIGHT=2;UP_LEFT=3;LEFT=4;DOWN_LEFT=5;DOWN_RIGHT=6

            if (y>11): ## hitting top wall
                if (direction == UP_LEFT):
                    new_theta = 2.0*3.14 - theta
                    self.set_abs_pose(x, y, new_theta)

                elif (direction == UP_RIGHT):
                    new_theta = 2.0*3.14-theta
                    self.set_abs_pose(x, y, new_theta)


            if ( y< 1e-3): ## hitting bottom wall
                if (direction == DOWN_LEFT):
                    new_theta = -theta
                    self.set_abs_pose(x, y, new_theta)

                elif (direction == DOWN_RIGHT):
                    new_theta = -theta
                    self.set_abs_pose(x, y, new_theta)


            if (x> 11.0): ## hitting right wall
                if (direction == UP_RIGHT):
                    new_theta = 3.14 - theta
                    self.set_abs_pose(x, y, new_theta)

                if (direction == DOWN_RIGHT):
                    new_theta = -3.14-theta
                    self.set_abs_pose(x, y, new_theta)


            if (pose_b.x < 1e-3): ## hitting left wall
                if (direction == UP_LEFT):
                    new_theta = 3.14 - theta
                    self.set_abs_pose(x, y, new_theta)

                if (direction == DOWN_LEFT):
                    new_theta = -(3.14+ theta)
                    self.set_abs_pose(x, y, new_theta)


def main(args=None):
    
    rclpy.init(args=args)
    node = Ball()
    thread = threading.Thread(target=rclpy.spin,args=(node,),daemon=True)
    thread.start()
    rate = node.create_rate(5)
    
    try:
        while rclpy.ok():
            node.ball_move()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()