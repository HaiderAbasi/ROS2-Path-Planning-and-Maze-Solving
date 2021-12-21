import rclpy 
import cv2 
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import os
import numpy as np

copy_frame = True
frozen_bg = 0
filled_maze_withoutCar = 0

def extract_maze(frozen_maze,filled_maze):
  global frozen_bg,filled_maze_withoutCar
  gray_maze = cv2.cvtColor(frozen_maze,cv2.COLOR_BGR2GRAY)
  #Lane_gray_Smoothed = cv2.GaussianBlur(Lane_gray,(11,11),1)# Smoothing out the edges for edge extraction later
  edge_maze = cv2.Canny(gray_maze,50,150, None, 3) # Extracting the Edge of Canny
  cnts = cv2.findContours(edge_maze, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]# OpenCV 4.2
  for idx,_ in enumerate(cnts):
    cv2.drawContours(filled_maze, cnts, idx, 255,-1)

  filled_maze_withoutCar = filled_maze.copy()
  CarExtracted = np.zeros_like(filled_maze)
  Min_Cntr_area = 1000
  Min_Cntr_idx= -1
  for index, cnt in enumerate(cnts):
      area = cv2.contourArea(cnt)
      if (area < Min_Cntr_area) and (area > 10):
          Min_Cntr_area = area
          Min_Cntr_idx = index
          SmallestContour_Found = True
  print("min_area" , Min_Cntr_area)
  if (Min_Cntr_idx!=-1):
      filled_maze_withoutCar = cv2.drawContours(filled_maze_withoutCar, cnts, Min_Cntr_idx, 0, -1)
      #hull = cv2.convexHull(cnts[Min_Cntr_idx])
      CarExtracted = cv2.drawContours(CarExtracted, cnts, Min_Cntr_idx, 255, -1) 
      CarExtracted = cv2.drawContours(CarExtracted, cnts, Min_Cntr_idx, 255, 3) 
      frozen_Onlymaze_carless = cv2.bitwise_and(frozen_maze, frozen_maze,mask=filled_maze_withoutCar)
      frozen_maze_OnlyCar = cv2.bitwise_and(frozen_maze, frozen_maze,mask=CarExtracted)
      CarExtractedNot = cv2.bitwise_not(CarExtracted)
      frozen_maze_carless = cv2.bitwise_and(frozen_maze, frozen_maze,mask=CarExtractedNot)
      base_clr = frozen_maze_carless[0][0]
      print("base_clr = ",base_clr)
      bg = np.ones_like(frozen_maze)*base_clr
      frozen_bg = cv2.bitwise_and(bg, bg,mask=CarExtracted)
      frozen_bg = cv2.bitwise_or(frozen_bg,frozen_maze_carless)
  #cv2.imshow('gray_maze',gray_maze)
  #cv2.imshow('edge_maze',edge_maze)
  cv2.imshow('filled_maze',filled_maze)
  cv2.imshow('filled_maze_withoutCar',filled_maze_withoutCar)
  cv2.imshow('frozen_Onlymaze_carless',frozen_Onlymaze_carless)
  cv2.imshow('frozen_maze_OnlyCar',frozen_maze_OnlyCar)
  cv2.imshow('frozen_maze_carless',frozen_maze_carless)
  cv2.imshow('bg',bg)
  cv2.imshow('frozen_bg',frozen_bg)
  cv2.waitKey(0)
  #for idx,_ in enumerate(cnts):
    #if idx != Min_Cntr_idx:
      #cv2.drawContours(filled_maze_withoutCar, cnts, idx, 255,10)

def fillcnts(image):
  cnts = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]# OpenCV 4.2
  for idx,_ in enumerate(cnts):
    cv2.drawContours(image, cnts, idx, 255,-1)

def ret_largestObject(img):
    #Find the two Contours for which you want to find the min distance between them.
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
    Max_Cntr_area = 0
    Max_Cntr_idx= -1
    for index, cnt in enumerate(cnts):
        area = cv2.contourArea(cnt)
        if area > Max_Cntr_area:
            Max_Cntr_area = area
            Max_Cntr_idx = index
    img_largestobject = np.zeros_like(img)
    if (Max_Cntr_idx!=-1):
        img_largestobject = cv2.drawContours(img_largestobject, cnts, Max_Cntr_idx, 255, -1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
        img_largestobject = cv2.drawContours(img_largestobject, cnts, Max_Cntr_idx, 255, 2) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
    return img_largestobject,cnts[Max_Cntr_idx]

  
class Video_get(Node):
  def __init__(self):
    super().__init__('video_subscriber')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
    self.bridge = CvBridge() # converting ros images to opencv data
 
  def process_data(self, data): 
    
    global copy_frame
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

    if copy_frame:
      frozen_maze = frame.copy()
      filled_maze = np.zeros((frame.shape[0],frame.shape[1]),dtype=np.uint8)
      copy_frame = False
      cv2.imshow('Frozen_maze',frozen_maze)
      cv2.waitKey(20) & 0xFF
      extract_maze(frozen_maze,filled_maze)

    #frame_smoothed = cv2.GaussianBlur(frame,(11,11),3)# Smoothing out the edges for edge extraction later
    
    change = cv2.absdiff(frame, frozen_bg)

    #change = frame_smoothed - frozen_bg
    #frame_smoothed_u2 = frame_smoothed.astype('u2')
    #frozen_bg_u2 = frozen_bg.astype('u2')
    #tmp = frame_smoothed - frozen_bg
    #result = tmp.clip(0,255).astype('u1')

    change_gray = cv2.cvtColor(change,cv2.COLOR_BGR2GRAY)
    change_bin = cv2.threshold(change_gray, 15, 255, cv2.THRESH_BINARY)[1]
    car_located,cnt_largest = ret_largestObject(change_bin)

    car_ROI = np.zeros_like(car_located)
    center, radii = cv2.minEnclosingCircle(cnt_largest)
    car_ROI = cv2.circle(car_ROI, (int(center[0]), int(center[1])), int(radii+(radii*0.4)), 255, -1)
    car_ROI = cv2.bitwise_xor(car_ROI, car_located)

    change_filled = change_bin.copy()
    fillcnts(change_filled)

    #frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #frame_edge = cv2.Canny(frame_gray,50,150, None, 3) # Extracting the Edge of Canny
    #fillcnts(frame_edge)
    
    #car_spotted = cv2.absdiff(change_filled,filled_maze_withoutCar)
    car_spotted = cv2.subtract(change_filled, filled_maze_withoutCar)
    #frame[np.where(car_spotted==True)] = frame[np.where(car_spotted==True)] + (0,128,0)
    car_identified = frame.copy()
    #car_identified[car_spotted==True]  = car_identified[car_spotted==True] + (0,128,0)
    car_identified[car_spotted>0]  = (0,128,0)
    car_identified[car_ROI>0]  = (128,0,128)
    #cv2.bitwise_and(maze, green,car_spotted)
    cv2.imshow("maze", frame) # displaying what is being recorded 
    cv2.imshow("change", change_bin) # displaying what is being recorded
    cv2.imshow("change_filled", change_filled) # displaying what is being recorded
    cv2.imshow("car_located", car_located) # displaying what is being recorded
    cv2.imshow("car_spotted", car_spotted) # displaying what is being recorded
    cv2.imshow("car_identified", car_identified) # displaying what is being recorded
    cv2.waitKey(1) # will save video until it is interrupted
  

  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()