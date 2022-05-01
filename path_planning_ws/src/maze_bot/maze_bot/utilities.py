import cv2
import numpy as np

from . import config

def imfill(image):
  cnts = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]# OpenCV 4.2
  for idx,_ in enumerate(cnts):
    cv2.drawContours(image, cnts, idx, 255,-1)

def ret_largest_obj(img):
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

def ret_smallest_obj(cnts, noise_thresh = 10):
  Min_Cntr_area = 1000
  Min_Cntr_idx= -1
  for index, cnt in enumerate(cnts):
      area = cv2.contourArea(cnt)
      if (area < Min_Cntr_area) and (area > 10):
          Min_Cntr_area = area
          Min_Cntr_idx = index
          SmallestContour_Found = True
  print("min_area" , Min_Cntr_area)
  return Min_Cntr_idx

class Debugging:

    def __init__(self): 
       self.time_elasped = 0
       self.Live_created = False


    def nothing(self,x):
        pass

    cv2.namedWindow('CONFIG')
    # create switch for ON/OFF functionality
    debugging_SW = 'Debug'
    cv2.createTrackbar(debugging_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingLoc_SW = 'Debug Loc'
    cv2.createTrackbar(debuggingLoc_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingMapping_SW = 'Debug Mapp.'
    cv2.createTrackbar(debuggingMapping_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingPathPlanning_SW = 'Debug Path P.'
    cv2.createTrackbar(debuggingPathPlanning_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingMotionPlanning_SW = 'Debug Motion P.'
    cv2.createTrackbar(debuggingMotionPlanning_SW, 'CONFIG',False,True,nothing)

    # create switch for ON/OFF functionality
    debugging_Live = 'Debug_Live'
    cv2.createTrackbar(debugging_Live, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality

    def setDebugParameters(self):

        if (self.time_elasped >5):
            # get current positions of four trackbars
            debug = cv2.getTrackbarPos(self.debugging_SW,'CONFIG')
            debug_localization = cv2.getTrackbarPos(self.debuggingLoc_SW,'CONFIG')
            debug_mapping = cv2.getTrackbarPos(self.debuggingMapping_SW,'CONFIG')
            debug_pathplanning = cv2.getTrackbarPos(self.debuggingPathPlanning_SW,'CONFIG')
            debug_motionplanning = cv2.getTrackbarPos(self.debuggingMotionPlanning_SW,'CONFIG')
            debug_live = cv2.getTrackbarPos(self.debugging_Live,'CONFIG')

            if debug:
                config.debug = True
            else:
                config.debug = False

            if debug_localization:
                config.debug_localization = True
            else:
                config.debug_localization = False    
            if debug_mapping:
                config.debug_mapping = True
            else:
                config.debug_mapping = False           
            if debug_pathplanning:
                config.debug_pathplanning = True
            else:
                config.debug_pathplanning = False
            if debug_motionplanning:
                config.debug_motionplanning = True
            else:
                config.debug_motionplanning = False
            if debug_live:
                config.debug_live = True
            else:
                config.debug_live = False
        else: 

            self.time_elasped +=1


        
        if config.debug_live:
            debuggingLIVEConfig_SW = 'Debug (Live)'
            debuggingMAPLIVEConfig_SW = 'Debug_map (Live)'
            debuggingPathLIVEConfig_SW = 'Debug_path (Live)'
            if not self.Live_created:
                self.Live_created = True
                cv2.namedWindow('CONFIG_LIVE')
                cv2.createTrackbar(debuggingLIVEConfig_SW, 'CONFIG_LIVE',0,100,self.nothing)
                cv2.createTrackbar(debuggingMAPLIVEConfig_SW, 'CONFIG_LIVE',0,100,self.nothing)
                cv2.createTrackbar(debuggingPathLIVEConfig_SW, 'CONFIG_LIVE',0,100,self.nothing)

            debug_live_amount = cv2.getTrackbarPos(debuggingLIVEConfig_SW,'CONFIG_LIVE')
            debug_map_live_amount = cv2.getTrackbarPos(debuggingMAPLIVEConfig_SW,'CONFIG_LIVE')
            debug_path_live_amount = cv2.getTrackbarPos(debuggingPathLIVEConfig_SW,'CONFIG_LIVE')

            config.debug_live_amount = (debug_live_amount/100)
            config.debug_map_live_amount = (debug_map_live_amount/100)
            config.debug_path_live_amount = (debug_path_live_amount/100)

        else:
            self.Live_created = False
            try:
                cv2.destroyWindow('CONFIG_LIVE')
            except:
                pass  


