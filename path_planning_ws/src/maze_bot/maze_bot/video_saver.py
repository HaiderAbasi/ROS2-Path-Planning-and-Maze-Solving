#!/usr/bin/env python3

'''
> Purpose :
This Node is going to save video feed from the upper camera from the simulation .
Which is going to be utilized for furthur image processing to solve the maze .


> Usage :
You need to write below command in terminal where your pacakge is sourced
- ros2 run maze_bot video_recorder

Note : name of the node is actually name of executable file described in setup.py file of our package and not the name of python file

> Inputs:
This node is a Subscriber of '/upper_camera/image_raw' topic which is 30 fps video from camera above the maze
and image is of the size 1280x720 ( RGB )

> Outputs:
This node is just a scriber so output is is not in terms of ROS topic but a video is going to be saved on the disk.


> Instructor Comments :
I wish , students make below changes into the code 
- Remove Opencv Dependency
- Resize images and save different sized video 
- - Use proper names for the class,node, call back functions and variables
- Add ROS2 logging functionality 
- Add comments into functions

Author :
M.Luqman

Date :
16/03/22
'''

import rclpy 
import cv2 
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import os

class Video_get(Node):
  def __init__(self):
    super().__init__('video_subscriber')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.process_data,10)
    ## setting for writing the frames into a video
    vid_path = os.path.join(os.getcwd(),"output.avi")

    self.out = cv2.VideoWriter(vid_path,cv2.VideoWriter_fourcc('M','J','P','G'), 30, (1280,720))
    self.bridge = CvBridge() # converting ros images to opencv data
 
  def process_data(self, data): 
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    self.out.write(frame)# write the frames to a video
    cv2.imshow("output", frame) # displaying what is being recorded 
    cv2.waitKey(1) # will save video until it is interrupted
  

  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()