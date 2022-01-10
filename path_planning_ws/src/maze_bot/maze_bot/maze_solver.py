import rclpy 
import cv2 
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import os
import numpy as np
from .bot_localization import bot_localizer
from .bot_mapping import maze_converter

class Video_get(Node):
  def __init__(self):
    super().__init__('video_subscriber')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
    self.bridge = CvBridge() # converting ros images to opencv data
    self.bot_localizer = bot_localizer()
    self.maze_converter = maze_converter()

  def process_data(self, data): 
    
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    
    # Saving frame to display roi's
    frame_disp = frame.copy()
    
    # Stage 1 => Localizing the Bot
    self.bot_localizer.localize_bot(frame,frame_disp)

    # Stage 2 => Converting maze (image) into maze (matrix)
    self.maze_converter.graphify(self.bot_localizer.extracted_maze,self.bot_localizer.unit_dim)

    cv2.imshow("Maze (Live)", frame_disp) # displaying what is being recorded 
    cv2.waitKey(10)
  
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()