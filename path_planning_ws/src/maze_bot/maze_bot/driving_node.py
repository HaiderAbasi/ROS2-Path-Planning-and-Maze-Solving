#!/usr/bin/env python3

'''
> Purpose :
This Node is going to send velocity commands to our differential drive robot
commands structure contains :
- Linear  ( x,y,z )
- Angular ( x,y,z )

in our case we will only be utilizing 
- linear  x
- angular z

> Usage :
You need to write below command in terminal where your pacakge is sourced
- ros2 run maze_bot driving_node

Note : Name of the node is actually name of executable file described in setup.py file of our package and not the name of python file

> Inputs:
This node is a Publisher only so input are giving

> Outputs:
This node publishes on topic "/cmd_vel" with a buffer size of 10 every half a second
You can manupilate the time of publishing through *time_period* variable

> Instructor Comments :
I wish , students make below changes into the code 
- Use proper names for the class and node
- Add ROS2 logging functionality 
- Callback functions should have proper name
- Add comments into functions

Author :
M.Luqman

Date :
16/03/22
'''


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x=0.3;
        msg.angular.z=0.5
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()