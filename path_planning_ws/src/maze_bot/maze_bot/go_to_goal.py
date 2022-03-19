'''
> Purpose :
This Node is going to move the robot to a specified 2D location in gazebo using the command velocity topic.
Node is applying mathemetical operation to current and goal locations for the robot to compute remaining error to desired Pose
> Usage :
You need to write below command in terminal where your pacakge is sourced
- ros2 run maze_bot go_to_goal

Note : Name of the node is actually name of executable file described in setup.py file of our package and not the name of python file

> Inputs:
This node is subscribing odometery on "/odom" topic to get location of the robot

> Outputs:
This node publishes on topic "/cmd_vel" , the required velocity ( linear and angular ) to move to specified 3D goal location.


> Instructor Comments :
I wish , students make below changes into the code
- More Robust implementation for Go to Goal behaviour
- Use proper names for the class,node, call back functions and variables
- Add ROS2 logging functionality
- Add comments into functions

Author :
M.Luqman

Date :
16/03/22
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist ,Point
from nav_msgs.msg import Odometry
from math import pow , atan2,sqrt
import math
import sys

class mazebot_GTG(Node):
    def __init__(self):
        super().__init__("go_to_goal")
        self.velocity_publisher    = self.create_publisher(Twist,'/cmd_vel',10)
        self.robot_pose_subscriber = self.create_subscription(Odometry,'/odom',self.get_mazebot_pose,10)
        self.timer                 = self.create_timer(0.2, self.goal_movement)
        self.robot_pose            = Point()
        self.goal_pose             = Point()
        self.vel_msg               = Twist()
        self.distance_to_goal      = 0.0
        self.angle_to_goal         = 0.0
        self.angle_to_goal         = 0
        self.angle_offset          = 0

    def get_mazebot_pose(self,data):
        self.robot_pose.x     = data.pose.pose.position.x
        self.robot_pose.y     = data.pose.pose.position.y
        quaternions     = data.pose.pose.orientation
        (roll,pitch,yaw)= self.euler_from_quaternion(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
        self.robot_pose.z = yaw



    def goal_movement(self):

        self.goal_pose.x = float(sys.argv[1])
        self.goal_pose.y = float(sys.argv[2])
        self.angle_offset= float(sys.argv[3])


        self.distance_to_goal = sqrt(pow((self.goal_pose.x - self.robot_pose.x),2) + pow((self.goal_pose.y - self.robot_pose.y),2))
        self.angle_to_goal =  atan2(self.goal_pose.y-self.robot_pose.y,self.goal_pose.x-self.robot_pose.x) + self.angle_offset
        self.angle_to_goal=self.angle_to_goal - self.robot_pose.z

        if abs(self.angle_to_goal) > 0.1:
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = self.angle_to_goal

        else:
            self.vel_msg.linear.x = self.distance_to_goal
        # msg = 'DG:{:.3f} AG:{:.3f}' . format(self.distance_to_goal,self.angle_to_goal)
        # self.get_logger().info(msg)

        self.velocity_publisher.publish(self.vel_msg)




    def euler_from_quaternion(self,x, y, z, w):
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)

            return roll_x, pitch_y, yaw_z

def main(args =None):
    rclpy.init()
    node =mazebot_GTG()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
