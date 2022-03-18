#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
from math import pow, atan2, sqrt

class Turtlebotogoal(Node):
    def __init__(self):
        super().__init__('turtle_to_goal')
        self.cmd_vel_pub = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.pose_subs = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.move2goal)
        self.pose=Pose()
        self.flag= False
        self.distance_e=0.0;self.angle_e= 0.0

    def pose_callback(self,data):
        self.pose.x=data.x
        self.pose.y=data.y
        self.pose.theta= data.theta

    def ecludian_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2) + pow((goal_pose.y - self.pose.y),2))

    def linear_vel(self,goal_pose , constant =2 ):
        self.distance_e=self.ecludian_distance(goal_pose)
        return constant*self.distance_e;


    def steering_angle(self,goal_pose):
        return atan2(goal_pose.y-self.pose.y,goal_pose.x-self.pose.x)

    def angular_vel(self,goal_pose,constant=2):
        self.angle_e=self.steering_angle(goal_pose)-self.pose.z
        return constant*( self.angle_e)

    def move2goal(self):
        goal_pose=Pose()
        goal_pose.x = float(sys.argv[1])
        goal_pose.y = float(sys.argv[2])
        goal_pose.theta = float(sys.argv[3])
        distance_tolerance = 0.1
        angular_tolerance= 0.01
        vel_msg=Twist()

        if abs(self.steering_angle(goal_pose) - self.pose.theta)>angular_tolerance:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)
        else:
            vel_msg.angular.z = 0.0
            if self.ecludian_distance(goal_pose)>=distance_tolerance:
                vel_msg.linear.x = self.linear_vel(goal_pose)
            else:
                vel_msg.linear.x = 0.0
                self.flag=True

        if self.flag:
            vel_msg.angular.z = goal_pose.theta-self.pose.theta
            if abs(goal_pose.theta - self.pose.theta) <= angular_tolerance:
                quit()
        self.get_logger().info('X: {:.3f}, Y: {:.3f}, Theta: {:.3f} De {:.3f} Ae {:.3f}' . format(self.pose.x,self.pose.y,self.pose.z,self.distance_e,self.angle_e))

        self.cmd_vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebotogoal()
    rclpy.spin(node)
    rclpy.shutdown()

















# '''
# > Purpose :
# This Node is going to move the robot to a specified 2D location in gazebo using the command velocity topic.
# Node is applying mathemetical operation to current and goal locations for the robot to compute remaining error to desired Pose
# > Usage :
# You need to write below command in terminal where your pacakge is sourced
# - ros2 run maze_bot go_to_goal

# Note : Name of the node is actually name of executable file described in setup.py file of our package and not the name of python file

# > Inputs:
# This node is subscribing odometery on "/odom" topic to get location of the robot

# > Outputs:
# This node publishes on topic "/cmd_vel" , the required velocity ( linear and angular ) to move to specified 3D goal location.


# > Instructor Comments :
# I wish , students make below changes into the code
# - More Robust implementation for Go to Goal behaviour
# - Use proper names for the class,node, call back functions and variables
# - Add ROS2 logging functionality
# - Add comments into functions

# Author :
# M.Luqman

# Date :
# 16/03/22
# '''

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Vector3
# from math import pow , atan2,sqrt , degrees ,pi
# import math

# class mazebot_GTG(Node):
#     def __init__(self):
#         super().__init__("go_to_goal")
#         self.velocity_publisher = self.create_publisher(Twist,'/cmd_vel',10)
#         self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.get_mazebot_pose,10)
#         self.timer = self.create_timer(0.2, self.goal_movement)
#         self.pose = Vector3()
#         self.vel_msg = Twist()
#         self.goal_x=4.0
#         self.goal_y=4.0
#         self.goal_not_reached_flag=True

#     def get_mazebot_pose(self,data):
#         self.pose.x=data.pose.pose.position.x
#         self.pose.y=data.pose.pose.position.y
#         quaternions = data.pose.pose.orientation
#         (roll,pitch,yaw)=self.euler_from_quaternion(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
#         self.pose.z = yaw
#         # self.get_logger().info('X: {:.3f}, Y: {:.3f}, Theta: {:.3f}' . format(self.pose.x,self.pose.y,self.pose.z))

#     def bound(self,low, high, value):
#         return max(low, min(high, value))

#     def goal_movement(self):

#         error_x= self.goal_x - self.pose.x
#         error_y=self.goal_y - self.pose.y
#         # distance_to_goal = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )

#         angle_to_goal=atan2(error_y,error_x)

#         if abs(angle_to_goal - self.pose.z) > 0.5:
#             state = 0
#             self.vel_msg.linear.x = 0.0
#             self.vel_msg.angular.z = 0.3
#         else:
#             state = 1
#             self.vel_msg.linear.x = 0.5
#             self.vel_msg.angular.z = 0.0



#         self.get_logger().info('EX: {:.2f} EY: {:.2f} ATG: {:.2f} RA:{:.2f} state:{:.2f}'. format(error_x,error_y,angle_to_goal,self.pose.z,state))

#         # if( angle_to_turn == 0.0):
#         #     vel_msg.linear.x = 1.0
#         #     vel_msg.angular.z = 0.0
#         # vel_msg.angular.z = 0.3
#         # if self.goal_not_reached_flag:
#         #     self.get_logger().info('Moving')

#         # if (distance_to_goal>=0.5):
#         #     self.goal_not_reached_flag=False

#         # self.velocity_publisher.publish(self.vel_msg)


#     def euler_from_quaternion(self,x, y, z, w):
#             t0 = +2.0 * (w * x + y * z)
#             t1 = +1.0 - 2.0 * (x * x + y * y)
#             roll_x = math.atan2(t0, t1)

#             t2 = +2.0 * (w * y - z * x)
#             t2 = +1.0 if t2 > +1.0 else t2
#             t2 = -1.0 if t2 < -1.0 else t2
#             pitch_y = math.asin(t2)

#             t3 = +2.0 * (w * z + x * y)
#             t4 = +1.0 - 2.0 * (y * y + z * z)
#             yaw_z = math.atan2(t3, t4)

#             return roll_x, pitch_y, yaw_z # in radians

# def main(args =None):
#     rclpy.init()
#     node =mazebot_GTG()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()