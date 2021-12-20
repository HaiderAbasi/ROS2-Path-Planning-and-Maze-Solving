import rclpy 
import cv2 
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import os
import numpy as np

mouseX = 0
mouseY = 0
copy_frame = True
frozen_maze = 0
original_points = []
transformed_points = []

gathering_pairs = True
gathered_points = 0
paris_to_match = 8

def get_TransformedPoints(event,x,y,flags,param):

    global mouseX,mouseY,gathering_pairs,original_points,transformed_points,gathered_points
    
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX,mouseY = x,y
        print (mouseX,mouseY)
        
        gathered_points += 1
        if gathering_pairs:
            if ((gathered_points %2) != 0):
              cv2.circle(frozen_maze,(x,y),5,(0,0,255),-1)#RED
              original_points.append([mouseX,mouseY])
            else:
              cv2.circle(frozen_maze,(x,y),5 ,(0,255,0),-1)#Green
              transformed_points.append([mouseX,mouseY])

            if ( int(gathered_points/2)==paris_to_match ):

              original_points = np.array(original_points)
              transformed_points = np.array(transformed_points)
              print("Points_gathered!!!")
              print("original_points = ",original_points)
              print("transformed_points = ",transformed_points)
              gathering_pairs = False
            


class Video_get(Node):
  def __init__(self):
    super().__init__('video_subscriber')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
    self.bridge = CvBridge() # converting ros images to opencv data
    cv2.namedWindow('Gather_points')
    cv2.setMouseCallback('Gather_points',get_TransformedPoints)
 
  def process_data(self, data): 
    global copy_frame,frozen_maze
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    if copy_frame:
      frozen_maze = frame.copy()
      copy_frame = False
    while(1):
        cv2.imshow('Gather_points',frozen_maze)
        k = cv2.waitKey(20) & 0xFF
        if (k == 27) or not gathering_pairs:
            break
    h,status = cv2.findHomography(original_points,transformed_points)
    undistorted_frame = np.zeros_like(frame)
    undistorted_frame = cv2.warpPerspective(frame, h, (undistorted_frame.shape[1],undistorted_frame.shape[0]))
    cv2.imshow("[Undistored] maze", undistorted_frame) # displaying what is being recorded 
    cv2.waitKey(0)

    cv2.imshow("maze", frame) # displaying what is being recorded 
    cv2.waitKey(1) # will save video until it is interrupted
  

  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()