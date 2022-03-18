'''
> Purpose :
Node to perform the actual (worthy of your time) task of maze solving ;) 
- Robot velocity interface
- Upper Video camera as well

> Usage :
You need to write below command in terminal where your pacakge is sourced
- ros2 run maze_bot maze_solver

Note : Name of the node is actually name of executable file described in setup.py file of our package and not the name of python file

> Inputs:
This node is subscribing video feed from (Satellite or DroneCam)

> Outputs:
This node publishes on topic "/cmd_vel" , the required velocity ( linear and angular ) to move the robot

Author :
Haider Abbasi

Date :
18/03/22
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

import numpy as np
class maze_solver(Node):
    def __init__(self):
        super().__init__("maze_solving_node")
        self.velocity_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.videofeed_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.get_video_feed_cb,10)

        self.timer = self.create_timer(0.2, self.maze_solving)
        self.bridge = CvBridge()
        self.vel_msg=Twist()

        self.sat_view = np.zeros((100,100))

    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.sat_view = frame
        cv2.imshow("sat_view", self.sat_view)
        cv2.waitKey(1)



    def maze_solving(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

        self.velocity_publisher.publish(self.vel_msg)





def main(args =None):
    rclpy.init()
    node_obj =maze_solver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()