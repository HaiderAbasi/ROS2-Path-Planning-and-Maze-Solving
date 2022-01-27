import cv2
import numpy as np
from .utilities import ret_smallest_obj,ret_largest_obj,imfill


class bot_localizer():

    def __init__(self):
        # State Variables
        self.is_bg_extracted = False
        self.is_base_unit_extracted = False
        self.is_maze_extracted = False

        # Transfomation(Crop + Rotated) Variables
        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.orig_rot = 0
        self.transform_arr = []
        self.rot_mat = 0

        # Parameters storing bg information for using bg-subtraction
        self.bg_model = 0
        self.filled_maze_withoutCar = 0

        # Unit Dimension for each node in maze
        self.unit_dim = 0
        self.extracted_maze = 0 
        self.loc_car = 0


    def extract_bg(self,frozen_maze):

        gray_maze = cv2.cvtColor(frozen_maze,cv2.COLOR_BGR2GRAY)
        edge_maze = cv2.Canny(gray_maze,50,150, None, 3) # Extracting the Edge of Canny
        cnts = cv2.findContours(edge_maze, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]# OpenCV 4.2
        filled_maze = np.zeros((frozen_maze.shape[0],frozen_maze.shape[1]),dtype=np.uint8)
        for idx,_ in enumerate(cnts):
            cv2.drawContours(filled_maze, cnts, idx, 255,-1)

        # Removing car from edge-detected obj's by removing the smallest object
        self.filled_maze_withoutCar = filled_maze.copy()
        Min_Cntr_idx = ret_smallest_obj(cnts)
        if (Min_Cntr_idx!=-1):
            self.filled_maze_withoutCar = cv2.drawContours(self.filled_maze_withoutCar, cnts, Min_Cntr_idx, 0, -1)  
            CarExtracted = np.zeros_like(filled_maze)
            CarExtracted = cv2.drawContours(CarExtracted, cnts, Min_Cntr_idx, 255, -1) 
            CarExtracted = cv2.drawContours(CarExtracted, cnts, Min_Cntr_idx, 255, 3) 
            CarExtracted_inv = cv2.bitwise_not(CarExtracted)
            frozen_maze_carless = cv2.bitwise_and(frozen_maze, frozen_maze,mask=CarExtracted_inv)
            base_clr = frozen_maze_carless[0][0]
            bg = np.ones_like(frozen_maze)*base_clr
            self.bg_model = cv2.bitwise_and(bg, bg,mask=CarExtracted)
            self.bg_model = cv2.bitwise_or(self.bg_model,frozen_maze_carless)

        # Cropping the maze ROI from the rest
        filled_maze_dilated = np.zeros_like(filled_maze)

        if cnts:
            cnts_ = np.concatenate(cnts)
            cnts_ = np.array(cnts_)
            cv2.fillConvexPoly(filled_maze_dilated, cnts_, 255)

        cnts_largest = cv2.findContours(filled_maze_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]# OpenCV 4.2
        hull = cv2.convexHull(cnts_largest[0])
        cv2.drawContours(filled_maze_dilated, [hull], 0, 255)
        [X, Y, W, H] = cv2.boundingRect(hull)
        temp = self.filled_maze_withoutCar
        temp = temp[Y:Y+H, X:X+W]
        self.orig_X = X
        self.orig_Y = Y
        self.orig_rows = H
        self.orig_cols = W
        self.transform_arr = [X,Y,W,H]
        # Rotation Matrix
        self.orig_rot = 90 # 90 degree counterClockwise
        self.rot_mat = np.array(
                                [
                                 [ np.cos(np.deg2rad(self.orig_rot)) , np.sin(np.deg2rad(self.orig_rot))],
                                 [-np.sin(np.deg2rad(self.orig_rot)) , np.cos(np.deg2rad(self.orig_rot))]
                                ]
                               )
        self.rot_mat_rev = np.array(
                                [
                                 [ np.cos(np.deg2rad(-self.orig_rot)) , np.sin(np.deg2rad(-self.orig_rot))],
                                 [-np.sin(np.deg2rad(-self.orig_rot)) , np.cos(np.deg2rad(-self.orig_rot))]
                                ]
                               )
        #================================= Testing if decreasing convexhull size would eliminate faulty boundary selection ==============
        # >>>>> It did not !!! Entry and exit point still pose a problem
        #per_change = -2 # 2 percent decrease in size 
        #row_chng = int( H*(per_change/100) )
        #col_chng = int( W*(per_change/100) )
        #temp = temp[ Y-row_chng:Y+H+row_chng , X-col_chng:X+W+col_chng ]
        maze_extracted = cv2.bitwise_not(temp)
        maze_extracted = cv2.rotate(maze_extracted, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if not self.is_maze_extracted:
            self.extracted_maze = maze_extracted
            self.is_maze_extracted = True


        cv2.imshow('self.extracted_maze',self.extracted_maze)
        cv2.imshow('filled_maze',filled_maze)
        cv2.imshow('bg',bg)
        cv2.imshow('self.bg_model',self.bg_model)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    @staticmethod
    def get_centroid(cnt):
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cy,cx)

    def get_car_loc(self,car_contour,img_loc_car):

        bot_center_pt = self.get_centroid(car_contour)

        # point --> (col(x),row(y)) XY-Convention For Rotation And Translated To MazeCrop (Origin)
        bot_center_ = np.array( [bot_center_pt[1]-self.orig_X, bot_center_pt[0]-self.orig_Y] )
        
        # Rot Matrix (For Normal XY Convention Around Z axis = [cos0 -sin0]) But for Image convention [ cos0 sin0]
        #                                                      [sin0  cos0]                           [-sin0 cos0]
        rot_center = (self.rot_mat @ bot_center_.T).T# [x,y]
        
        # Translating Origin If neccasary (To get whole image)
        rot_cols = self.orig_rows
        rot_rows = self.orig_cols
        rot_center[0] = rot_center[0] + (rot_cols * (rot_center[0]<0) )  
        rot_center[1] = rot_center[1] + (rot_rows * (rot_center[1]<0) )   
        
        img_loc_car = cv2.cvtColor(img_loc_car, cv2.COLOR_GRAY2BGR)
        img_loc_car = cv2.circle(img_loc_car,tuple(bot_center_),3,(255,128,0),-1)
        cv2.imshow("img_loc_car",img_loc_car)

        img_loc_car_crop = img_loc_car[self.orig_Y:self.orig_Y+self.orig_rows,self.orig_X:self.orig_X+self.orig_cols]
        img_loc_car_crop_rot = cv2.rotate(img_loc_car_crop, cv2.ROTATE_90_COUNTERCLOCKWISE)

        self.loc_car = (int(rot_center[0]),int(rot_center[1]))

        img_loc_car_crop_rot = cv2.circle(img_loc_car_crop_rot,self.loc_car,3,(0,0,255),-1)

        cv2.imshow("img_loc_car_crop_rot",img_loc_car_crop_rot)
        #cv2.waitKey(0)

    def localize_bot(self,curr_frame,frame_disp):

        if not self.is_bg_extracted:
            self.extract_bg(curr_frame.copy())
            self.is_bg_extracted = True

        # Performing Background subtraction to localize bot
        change = cv2.absdiff(curr_frame, self.bg_model)
        change_gray = cv2.cvtColor(change,cv2.COLOR_BGR2GRAY)
        change_bin = cv2.threshold(change_gray, 15, 255, cv2.THRESH_BINARY)[1]
        change_filled = change_bin.copy()
        imfill(change_filled)
        car_isolated,cnt_largest = ret_largest_obj(change_bin)

        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>EXTRACTING CAR LOCATIOn>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        self.get_car_loc(cnt_largest,car_isolated)

        # Extracting circular bounding roi
        car_circular_roi = np.zeros_like(car_isolated)
        center, radii = cv2.minEnclosingCircle(cnt_largest)
        car_circular_roi = cv2.circle(car_circular_roi, (int(center[0]), int(center[1])), int(radii+(radii*0.4)), 255, -1)
        
        _,bounding_circle_cnt = ret_largest_obj(car_circular_roi)        
        [X, Y, W, H] = cv2.boundingRect(bounding_circle_cnt)
        car_circular_roi = cv2.bitwise_xor(car_circular_roi, car_isolated)
        
        car_unit = np.zeros_like(car_isolated)
        per_ext = int(0.15 * H) # (15 % larger then the circle) on all sides
        car_unit = cv2.rectangle(car_unit, (X-per_ext,Y-per_ext), ((X+W+per_ext),(Y+H+per_ext)), 255,-1)
        prev_circleNCar = cv2.bitwise_or(car_circular_roi, car_isolated)
        car_unit = cv2.bitwise_xor(car_unit, prev_circleNCar)
        # 1 pixel = 0.022988506m
        # Extracting Base Unit ==> (required for conversion to data)
        if not self.is_base_unit_extracted:
             self.unit_dim = W + per_ext
             print("Dim of Base unit are [ {} x {} ] pixels ".format(self.unit_dim,self.unit_dim))
             print("Dim of Maze is [ {} x {} ] pixels ".format(self.extracted_maze.shape[0],self.extracted_maze.shape[1]))
             self.is_base_unit_extracted = True

        # Displaying localized car and spotlight in the frame_disp
        frame_disp[car_isolated>0]  = frame_disp[car_isolated>0] + (0,64,0)
        frame_disp[car_circular_roi>0]  = (128,0,128)
        frame_disp[car_unit>0]  = (128,128,128)

        cv2.imshow("change_filled", change_filled) # displaying what is being recorded
        cv2.imshow("car_isolated", car_isolated) # displaying what is being recorded
        cv2.imshow("car_localized", frame_disp) # displaying what is being recorded


        
        