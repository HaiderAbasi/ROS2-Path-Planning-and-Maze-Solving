import rclpy 
import cv2 
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import os
from numpy import interp
import numpy as np
from .bot_localization import bot_localizer
from .bot_mapping import maze_converter

from geometry_msgs.msg import Twist
from math import pow , atan2,sqrt , degrees,asin
from nav_msgs.msg import Odometry


cv2.namedWindow("maze (Shortest Path + Car Loc)",cv2.WINDOW_FREERATIO)

class Video_get(Node):
  def __init__(self):
    super().__init__('video_subscriber')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
    self.bridge = CvBridge() # converting ros images to opencv data
    self.bot_localizer = bot_localizer()
    self.maze_converter = maze_converter()
    
    self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.get_pose,10)

    self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
    self.velocity = Twist()
    self.count = 0
    self.pta_taken = False
    self.car_angle_extracted = False
    self.car_angle_s = 0
    self.car_angle_rel = 0
    self.loc_i = 0

    self.goal_not_reached_flag = True

    self.goal_pose_x = 270
    self.goal_pose_y = 140

  @staticmethod
  def angle_n_dist(pt_a,pt_b):
    #print("pt_a {} and pt_b {} ".format(pt_a,pt_b))
    error_x= pt_b[0] - pt_a[0]
    error_y= pt_b[1] - pt_a[1]

    #distance_to_goal = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )
    angle_to_goal=atan2(error_y,error_x)
    #print(distance_to_goal)
    return(degrees(angle_to_goal))
    
  @staticmethod
  def euler_from_quaternion(x, y, z, w):
          t0 = +2.0 * (w * x + y * z)
          t1 = +1.0 - 2.0 * (x * x + y * y)
          roll_x = atan2(t0, t1)

          t2 = +2.0 * (w * y - z * x)
          t2 = +1.0 if t2 > +1.0 else t2
          t2 = -1.0 if t2 < -1.0 else t2
          pitch_y = asin(t2)

          t3 = +2.0 * (w * z + x * y)
          t4 = +1.0 - 2.0 * (y * y + z * z)
          yaw_z = atan2(t3, t4)

          return roll_x, pitch_y, yaw_z # in radians
  
  def get_pose(self,data):

      quaternions = data.pose.pose.orientation
      (roll,pitch,yaw)=self.euler_from_quaternion(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
      if (degrees(yaw)<0):
        self.car_angle_s = degrees(yaw)
      else:
          # 160  -360 = -200, 180 -360 = -180 .  90 - 360 = -270
        self.car_angle_s = degrees(yaw) - 360
      
      print("Car Angle (Simulation) = {}".format(self.car_angle_s))
      
  def bound(self,low, high, value):
    return max(low, min(high, value))
    
  def goal_movement(self,car_loc,car_angle):

      error_x = self.goal_pose_x - car_loc[0]
      error_y = self.goal_pose_y - car_loc[1]

      distance_to_goal = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )
      angle_to_goal = degrees(atan2(error_y,error_x))
      if(car_angle<0):
        #  -55    = 116 - 166
        angle_to_turn = angle_to_goal + car_angle
      else:
        angle_to_turn = angle_to_goal - car_angle

      velocity = self.bound(0.0,0.5,distance_to_goal)

      Angle = interp(angle_to_turn,[-180,180],[2,-2])

      print("angle to goal = {} Angle_to_turn = {} Angle[Sim] {}".format(angle_to_goal,angle_to_turn,abs(Angle)))
      print("self.goal_not_reached_flag = ",self.goal_not_reached_flag)
      print("distance_to_goal = ",distance_to_goal)

      if (distance_to_goal>=2):
        self.velocity.angular.z = Angle
      if abs(Angle) < 0.3:
        self.velocity.linear.x = velocity
      
      #self.get_logger().info('Error_X: {:.2f} Error_Y: {:.2f} DistanceTG: {:.2f} Turning_goal: {:.2f} Velocity: {:.2f}'  . format(error_x,error_y,distance_to_goal,angle_to_turn,velocity))

      if self.goal_not_reached_flag:
        self.publisher.publish(self.velocity)
        #self.get_logger().info('Moving')

      if (distance_to_goal<=2):

        self.velocity.angular.z = 0.0
        if self.goal_not_reached_flag:
          self.publisher.publish(self.velocity)
        
        #self.goal_not_reached_flag=False

        self.goal_pose_x = 290#int(input("Set your x goal"))
        self.goal_pose_y = 160#int(input("Set your y goal"))

  def process_data(self, data):
    
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    
    # Saving frame to display roi's
    frame_disp = frame.copy()
    
    # Stage 1 => Localizing the Bot
    self.bot_localizer.localize_bot(frame,frame_disp)

    # Stage 2 => Converting maze (image) into maze (matrix)
    self.maze_converter.graphify(self.bot_localizer.extracted_maze,self.bot_localizer.unit_dim)

    # Testing
    # Car loc >   \/
    if not self.pta_taken:
      self.loc_i = self.bot_localizer.loc_car
      self.pta_taken = True

    if (self.count>50):
      self.velocity.linear.x = 0.0
      print("Car angle (Image) = {} at loc {}".format(self.angle_n_dist(self.loc_i,self.bot_localizer.loc_car),self.bot_localizer.loc_car ))
      if not self.car_angle_extracted:
        self.car_angle = self.angle_n_dist(self.loc_i,self.bot_localizer.loc_car)
        print("Car angle (Image) = {} ".format(self.car_angle))       
        self.car_angle_rel = self.car_angle - self.car_angle_s
        self.car_angle_extracted = True
        self.publisher.publish(self.velocity)
      else:
        # [For noob luqman] : Extracting Car Angle [From Simulation Angle & S-I Relation]
        # self.car_angle_rel = 185 deg ,self.car_angle_s = -92  ,self.car_angle  =  93  (93)
        # self.car_angle_rel = 185 deg ,self.car_angle_s = -170 ,self.car_angle  =  15  (15)
        # self.car_angle_rel = 185 deg ,self.car_angle_s =  170 ,self.car_angle  =  355 (30)
        # self.car_angle_rel = 185 deg ,self.car_angle_s =  20  ,self.car_angle  =  205 (-165)
        self.car_angle = 180 - (self.car_angle_rel + self.car_angle_s)
        #if (self.car_angle_s<0):
          #self.car_angle = 180 - (self.car_angle_rel + self.car_angle_s)
        #else:
          # 160  -360 = -200, 180 -360 = -180 .  90 - 360 = -270
        #  self.car_angle = 180 - (self.car_angle_rel + (self.car_angle_s - 360) )
        #self.car_angle = (((self.car_angle_s>0)*(-1))+((self.car_angle_s<0)*(1)))*self.car_angle_rel + self.car_angle_s
        print("Car angle (Image From Relation) = {} I-S Relation {}".format(self.car_angle,self.car_angle_rel))
        self.goal_movement(self.bot_localizer.loc_car,self.car_angle)

    else:       
      self.count+=1 
      self.velocity.linear.x = 1.0        
      self.publisher.publish(self.velocity)

    img_shortest_path = (self.maze_converter.img_shortest_path).copy()
    img_shortest_path = cv2.circle(img_shortest_path, self.bot_localizer.loc_car, 3, (0,0,255))
    img_shortest_path = cv2.circle(img_shortest_path, (270,140), 3, (0,255,0))
    img_shortest_path = cv2.circle(img_shortest_path, (290,160), 3, (0,255,0))
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