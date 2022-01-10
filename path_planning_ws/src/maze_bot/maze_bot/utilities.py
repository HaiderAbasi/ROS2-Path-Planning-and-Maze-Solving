import cv2
import numpy as np

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