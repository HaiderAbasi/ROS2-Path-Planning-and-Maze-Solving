import rclpy 
import cv2 
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import os
import numpy as np
from .bot_localization import bot_localizer
from .bot_mapping import maze_converter

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .bot_control import Control

cv2.namedWindow("maze (Shortest Path + Car Loc)",cv2.WINDOW_FREERATIO)


class Video_get(Node):
  def __init__(self):
    super().__init__('video_subscriber')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
    self.bridge = CvBridge() # converting ros images to opencv data
    self.bot_localizer = bot_localizer()
    self.maze_converter = maze_converter()
    
    self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
    self.velocity = Twist()
    self.control = Control()
    self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.control.get_pose,10)

  def process_data(self, data):
    
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    
    # Saving frame to display roi's
    frame_disp = frame.copy()
    
    # Stage 1 => Localizing the Bot
    self.bot_localizer.localize_bot(frame,frame_disp)

    # Stage 2 => Converting maze (image) into maze (matrix)
    self.maze_converter.graphify(self.bot_localizer.extracted_maze,self.bot_localizer.unit_dim)

    path_i = self.control.path_iter
    path = self.maze_converter.shortest_path

    if (type(path)!=int):
      if (self.control.path_iter==0):
        self.control.goal_pose_x = self.maze_converter.shortest_path[path_i][0]
        self.control.goal_pose_y = self.maze_converter.shortest_path[path_i][1]

    self.control.move(self.maze_converter.shortest_path,self.bot_localizer.loc_car,self.velocity,self.publisher)
    

    
    img_shortest_path = (self.maze_converter.img_shortest_path).copy()
    img_shortest_path = cv2.circle(img_shortest_path, self.bot_localizer.loc_car, 3, (0,0,255))

    if (type(path)!=int):
      curr_goal = path[path_i]

      if path_i!=0:
        img_shortest_path = cv2.circle(img_shortest_path, path[path_i-1], 3, (0,255,0))
      img_shortest_path = cv2.circle(img_shortest_path, curr_goal, 3, (0,140,255))
      
    cv2.imshow("maze (Shortest Path + Car Loc)",img_shortest_path)

    cv2.imshow("Maze (Live)", frame_disp) # displaying what is being recorded 
    cv2.waitKey(10)
  
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()