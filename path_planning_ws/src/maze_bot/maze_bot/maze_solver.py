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
from math import pi,cos,sin

from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper
from .bot_pathplanning import bot_pathplanner
from .bot_motionplanning import bot_motionplanner

from nav_msgs.msg import Odometry

import numpy as np
from numpy import interp

from .utilities import Debugging
from . import config

class maze_solver(Node):

    def __init__(self):
        
        super().__init__("maze_solving_node")
        
        self.velocity_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.videofeed_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.get_video_feed_cb,10)
        
        # Visualizing what the robot sees by subscribing to bot_camera/Image_raw
        self.bot_subscriber = self.create_subscription(Image,'/Botcamera/image_raw',self.process_data_bot,10)

        self.timer = self.create_timer(0.2, self.maze_solving)
        self.bridge = CvBridge()
        self.vel_msg = Twist()
        
        # Creating objects for each stage of the robot navigation
        self.bot_localizer = bot_localizer()
        self.bot_mapper = bot_mapper()
        self.bot_pathplanner = bot_pathplanner()
        self.bot_motionplanner = bot_motionplanner()

        # Subscrbing to receive the robot pose in simulation
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.bot_motionplanner.get_pose,10)
        self.vel_subscriber = self.create_subscription(Odometry,'/odom',self.get_bot_speed,10)
        self.bot_speed = 0
        self.bot_turning = 0

        self.sat_view = np.zeros((100,100))

        self.debugging = Debugging()

    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.sat_view = frame
    
    def process_data_bot(self, data):
      self.bot_view = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

    def get_bot_speed(self,data):
        # We get the bot_turn_angle in simulation Using same method as Gotogoal.py
        self.bot_speed = -(data.twist.twist.linear.x)
        if self.bot_speed<0.1:
            self.bot_speed = 0.05

        self.bot_turning = data.twist.twist.angular.z

    # Overlay detected regions over the bot_view
    def overlay(self,image,overlay_img):

        gray = cv2.cvtColor(overlay_img, cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
        mask_inv = cv2.bitwise_not(mask)


        roi = image
        img2 = overlay_img
        # Now black-out the area of logo in ROI
        img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
        # Take only region of logo from logo image.
        img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
        
        image = img1_bg + img2_fg
        return image

    # Overlay detected regions (User-specified-amount) over the frame_disp
    def overlay_cropped(self,frame_disp,image_rot,crop_loc_row,crop_loc_col,overlay_cols):
        
        image_rot_cols = image_rot.shape[1]
        gray = cv2.cvtColor(image_rot[:,image_rot_cols-overlay_cols:image_rot_cols], cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(gray, 5, 255, cv2.THRESH_BINARY)[1]
        mask_inv = cv2.bitwise_not(mask)

        frame_overlay_cols = crop_loc_col + image_rot_cols
        roi = frame_disp[crop_loc_row:crop_loc_row + image_rot.shape[0],frame_overlay_cols-overlay_cols:frame_overlay_cols]            
        img2 = image_rot[:,image_rot_cols-overlay_cols:image_rot_cols]

        # Now black-out the area of logo in ROI
        img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
        # Take only region of logo from logo image.
        img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
        
        frame_disp[crop_loc_row:crop_loc_row + image_rot.shape[0],frame_overlay_cols-overlay_cols:frame_overlay_cols] = img1_bg + img2_fg


    def overlay_live(self,frame_disp,overlay,overlay_map,overlay_path):

        overlay_rot = cv2.rotate(overlay, cv2.ROTATE_90_CLOCKWISE)
        map_rot = cv2.rotate(overlay_map, cv2.ROTATE_90_CLOCKWISE)
        image_rot = cv2.rotate(overlay_path, cv2.ROTATE_90_CLOCKWISE)

        crop_loc_col = self.bot_localizer.transform_arr[0]+self.bot_mapper.crp_amt
        #crop_loc_endCol = self.bot_localizer.transform_arr[0]+self.bot_localizer.transform_arr[2]+self.bot_mapper.crp_amt
        crop_loc_row = self.bot_localizer.transform_arr[1]+self.bot_mapper.crp_amt

        new_cols = int(overlay_rot.shape[1]*config.debug_live_amount)
        new_path_cols = int(overlay_rot.shape[1]*config.debug_path_live_amount)
        new_map_cols = int(overlay_rot.shape[1]*config.debug_map_live_amount)


        frame_disp[crop_loc_row:crop_loc_row + overlay_rot.shape[0],crop_loc_col:crop_loc_col + new_cols] = overlay_rot[:,0:new_cols]
        
        if config.debug_map_live_amount>0:
            self.overlay_cropped(frame_disp,map_rot,crop_loc_row,crop_loc_col,new_map_cols)
        if config.debug_path_live_amount>0:
            self.overlay_cropped(frame_disp,image_rot,crop_loc_row,crop_loc_col,new_path_cols)

    # Draw speedometer and arrows indicating bot speed and direction at given moment
    def draw_bot_speedo(self,image,bot_speed):
        height, width = image.shape[0:2]
        # Ellipse parameters
        radius = 100
        center = (int(width / 2), height - 25)
        axes = (radius, radius)
        angle = 0
        startAngle = 180
        endAngle = 360
        thickness = 10

        # http://docs.opencv.org/modules/core/doc/drawing_functions.html#ellipse
        cv2.ellipse(image, center, axes, angle, startAngle, endAngle, (0,0,0), thickness)

        Estimted_line = np.zeros_like(image)
        max_speed = 1.5
        angle = -(((self.bot_speed/max_speed)*180)+90)
        speed_mph = int((self.bot_speed/max_speed)*200)
        length = 300
        P1 = center
        
        P2 = ( 
               int(P1[0] + length * sin(angle * (pi / 180.0) ) ),
               int(P1[1] + length * cos(angle * (pi / 180.0) ) ) 
             )
     
        cv2.line(Estimted_line,center, P2, (255,255,255),3)
        meter_mask = np.zeros((image.shape[0],image.shape[1]),np.uint8)

        cv2.ellipse(meter_mask, center, axes, angle, 0, endAngle, 255, -1)

        Estimted_line = cv2.bitwise_and(Estimted_line, Estimted_line,mask = meter_mask)

        speed_clr = (0,0,0)
        if speed_mph<20:
            speed_clr = (0,255,255)
        elif speed_mph<40:
            speed_clr = (0,255,0)
        elif speed_mph<60:
            speed_clr = (0,140,255)
        elif speed_mph>=60:
            speed_clr = (0,0,255)
        cv2.putText(image, str(speed_mph), (center[0]+30,center[1]-20), cv2.FONT_HERSHEY_PLAIN, 2, speed_clr,3)


        image = self.overlay(image,Estimted_line)
        if self.bot_turning>0.2:
            image = cv2.arrowedLine(image, (40,int(image.shape[0]/2)), (10,int(image.shape[0]/2)),
                                            (0,140,255), 13,tipLength=0.8)
        elif self.bot_turning<-0.2:
            image = cv2.arrowedLine(image, (image.shape[1]-40,int(image.shape[0]/2)), (image.shape[1]-10,int(image.shape[0]/2)),
                                            (0,140,255), 13,tipLength=0.8)

        return image


    def maze_solving(self):
        
        self.debugging.setDebugParameters()

        # Creating frame to display current robot state to user        
        frame_disp = self.sat_view.copy()
        
        # [Stage 1: Localization] Localizing robot at each iteration        
        self.bot_localizer.localize_bot(self.sat_view, frame_disp)

        # [Stage 2: Mapping] Converting Image to Graph
        self.bot_mapper.graphify(self.bot_localizer.maze_og)

        # [Stage 3: PathPlanning] Using {User Specified PathPlanner} to find path to goal        
        start = self.bot_mapper.Graph.start
        end = self.bot_mapper.Graph.end
        maze = self.bot_mapper.maze

        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="dijisktra")
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="a_star")
        if config.debug and config.debug_pathplanning:
            print("\nNodes Visited [Dijisktra V A-Star*] = [ {} V {} ]".format(self.bot_pathplanner.dijisktra.dijiktra_nodes_visited,self.bot_pathplanner.astar.astar_nodes_visited))


        # [Stage 4: MotionPlanning] Reach the (maze exit) by navigating the path previously computed
        bot_loc = self.bot_localizer.loc_car
        path = self.bot_pathplanner.path_to_goal
        self.bot_motionplanner.nav_path(bot_loc, path, self.vel_msg, self.velocity_publisher)

        # Displaying bot solving maze  (Live)
        img_shortest_path = self.bot_pathplanner.img_shortest_path
        self.bot_motionplanner.display_control_mechanism_in_action(bot_loc, path, img_shortest_path, self.bot_localizer, frame_disp)
        
        # View bot view on left to frame Display
        bot_view = cv2.resize(self.bot_view, (int(frame_disp.shape[0]/2),int(frame_disp.shape[1]/2)))
        bot_view = bot_view[0:int(bot_view.shape[0]/1.5),:]

        # Draw & Display [For better Understanding of current robot state]
        center_frame_disp = int(frame_disp.shape[0]/2)
        center_bot_view = int(bot_view.shape[0]/2)
        bot_offset = center_frame_disp - center_bot_view
        center_img_shortest_path = int(img_shortest_path.shape[0]/2)
        isp_offset = center_frame_disp - center_img_shortest_path

        bot_view = self.draw_bot_speedo(bot_view,self.bot_motionplanner.curr_speed)

        if config.debug_live:
            self.overlay_live(frame_disp,img_shortest_path,self.bot_mapper.maze_interestPts,self.bot_pathplanner.choosen_route)

        orig_col = 40 + int(bot_view.shape[1]/4)
        orig = (orig_col,bot_offset-10)
        cv2.putText(frame_disp, "Bot View", orig, cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0),3)
        frame_disp = cv2.rectangle(frame_disp, (20,bot_offset), (bot_view.shape[1]+20,(bot_view.shape[0]+bot_offset)), (0,0,255),12)
        frame_disp[bot_offset:(bot_view.shape[0]+bot_offset),20:bot_view.shape[1]+20] = bot_view
 
        cv2.imshow("Maze (Live)", frame_disp)
        cv2.waitKey(1)

def main(args =None):
    rclpy.init()
    node_obj =maze_solver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()