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

from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper
from .bot_pathplanning import bot_pathplanner
from .bot_motionplanning import bot_motionplanner

from nav_msgs.msg import Odometry

import numpy as np
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

        self.sat_view = np.zeros((100,100))

    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.sat_view = frame
    
    def process_data_bot(self, data):
      self.bot_view = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

    def maze_solving(self):

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

        if not self.bot_pathplanner.dijisktra.shortestpath_found:
            self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="dijisktra")

        if not self.bot_pathplanner.astar.shortestpath_found:
            self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="a_star")
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
        frame_disp[0:bot_view.shape[0],0:bot_view.shape[1]] = bot_view
        frame_disp[0:img_shortest_path.shape[0],frame_disp.shape[1]-img_shortest_path.shape[1]:frame_disp.shape[1]] = img_shortest_path
        
        cv2.imshow("Maze (Live)", frame_disp) 
        cv2.waitKey(1)

def main(args =None):
    rclpy.init()
    node_obj =maze_solver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()