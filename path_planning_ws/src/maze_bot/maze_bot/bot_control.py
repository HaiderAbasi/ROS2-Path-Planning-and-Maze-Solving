from math import pow , atan2,sqrt , degrees,asin
from numpy import interp
import numpy as np
import cv2

class Control:

    def __init__(self):

        self.count = 0
        self.pt_i_taken = False
        self.loc_i = 0
        
        self.car_angle_extracted = False
        self.car_angle = 0
        self.car_angle_s = 0
        self.car_angle_rel = 0
        
        self.goal_not_reached_flag = True
        self.goal_pose_x = 270
        self.goal_pose_y = 140
        self.path_iter = 0


    @staticmethod
    def angle_n_dist(pt_a,pt_b):
        #print("pt_a {} and pt_b {} ".format(pt_a,pt_b))
        error_x= pt_b[0] - pt_a[0]
        error_y= pt_a[1] - pt_b[1]

        distance_to_goal = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )

        angle_to_goal=atan2(error_y,error_x)
        
        angle_to_goal_deg = degrees(angle_to_goal)


        if (angle_to_goal_deg>0):
            return (angle_to_goal_deg),distance_to_goal
        else:
            # 160  -360 = -200, 180 -360 = -180 .  90 - 360 = -270
            return (angle_to_goal_deg + 360),distance_to_goal
    
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
        
        yaw_deg = degrees(yaw)

        if (yaw_deg>0):
            self.car_angle_s = yaw_deg
        else:
            # 160  -360 = -200, 180 -360 = -180 .  90 - 360 = -270
            self.car_angle_s = yaw_deg + 360
        
        #print("Car Angle (Simulation) = {}".format(self.car_angle_s))


    def goal_movement(self,path,car_loc,velocity_obj,publisher_obj):
        
        angle_to_goal, distance_to_goal = self.angle_n_dist(car_loc,(self.goal_pose_x,self.goal_pose_y))


        angle_to_turn = angle_to_goal - self.car_angle

        #velocity = interp(distance_to_goal,[0,100],[0.1,1])
        velocity = interp(distance_to_goal,[0,100],[0.2,1.5])

        Angle = interp(angle_to_turn,[-360,360],[-4,4])
        
        print("angle to goal = {} Angle_to_turn = {} Angle[Sim] {}".format(angle_to_goal,angle_to_turn,abs(Angle)))
        #print("self.goal_not_reached_flag = ",self.goal_not_reached_flag)
        print("distance_to_goal = ",distance_to_goal)

        if (distance_to_goal>=2):
            #pass
            velocity_obj.angular.z = Angle

        if abs(Angle) < 0.2:
            #pass
            velocity_obj.linear.x = velocity
        elif((abs(Angle) < 0.4)):
            velocity_obj.linear.x = 0.02
        else:
            velocity_obj.linear.x = 0.0
        

        if (self.goal_not_reached_flag) or (distance_to_goal<=1):
            #pass
            publisher_obj.publish(velocity_obj)

        print("len(path) = ( {} ) , path_iter = ( {} )".format(len(path),self.path_iter) )
        if (distance_to_goal<=2):
            #pass
            velocity_obj.linear.x = 0.0
            velocity_obj.angular.z = 0.0
            if self.goal_not_reached_flag:
                #pass
                publisher_obj.publish(velocity_obj)

            if self.path_iter==(len(path)-1):
                self.goal_not_reached_flag=False
            else:
                self.path_iter += 1
                print("Current Goal (x,y) = ( {} , {} )".format(path[self.path_iter][0],path[self.path_iter][1]))
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]



    def move(self,path,loc_car,velocity_obj,publisher_obj):
        # Testing

        if (self.count>20):
            #velocity_obj.linear.x = 0.0

            if not self.car_angle_extracted:
                # Stopping Car
                publisher_obj.publish(velocity_obj)
                # Extracting Car Angle (Img) from car_InitLoc and car_FinalLoc after moving forward (50 iters)
                self.car_angle,_ = self.angle_n_dist(self.loc_i,loc_car)
                print("Car angle (Image) = {} at loc {}".format(self.car_angle,loc_car ))
                self.car_angle_i = self.car_angle
                # Finding relation coeffiecient between car_angle (Image <-> Simulation)
                self.car_angle_rel = self.car_angle_s - self.car_angle 

                self.car_angle_extracted = True
            else:
                # [For noob luqman] : Extracting Car Angle [From Simulation Angle & S-I Relation]
                #self.car_angle = 180 - (self.car_angle_rel + self.car_angle_s)
                self.car_angle = (self.car_angle_s - self.car_angle_rel)
                print("\n\nCar angle (Image From Relation) = {} I-S Relation {} Car Angle (Simulation) = {}".format(self.car_angle,self.car_angle_rel,self.car_angle_s))
                print("Car angle_Initial (Image) = ",self.car_angle_i)
                print("Car loc {}".format(loc_car))
                
                # Traversing through path ()
                self.goal_movement(path,loc_car,velocity_obj,publisher_obj)

        else:
            # Car loc >   \/
            if not self.pt_i_taken:
                self.loc_i = loc_car
                self.pt_i_taken = True

            # Keep moving forward for 50 iterations(count)
            velocity_obj.linear.x = 1.0        
            publisher_obj.publish(velocity_obj)
            self.count+=1 

    @staticmethod
    def bck_to_orig(pt,transform_arr,rot_mat):

        st_col = transform_arr[0] # cols X
        st_row = transform_arr[1] # rows Y
        tot_cols = transform_arr[2] # total_cols / width W
        tot_rows = transform_arr[3] # total_rows / height H
        
        # point --> (col(x),row(y)) XY-Convention For Rotation And Translated To MazeCrop (Origin)
        #pt_array = np.array( [pt[0]+st_col, pt[1]+st_row] )
        pt_array = np.array( [pt[0], pt[1]] )
        
        # Rot Matrix (For Normal XY Convention Around Z axis = [cos0 -sin0]) But for Image convention [ cos0 sin0]
        #                                                      [sin0  cos0]                           [-sin0 cos0]
        rot_center = (rot_mat @ pt_array.T).T# [x,y]
        
        # Translating Origin If neccasary (To get whole image)
        rot_cols = tot_cols#tot_rows
        rot_rows = tot_rows#tot_cols
        rot_center[0] = rot_center[0] + (rot_cols * (rot_center[0]<0) ) + st_col  
        rot_center[1] = rot_center[1] + (rot_rows * (rot_center[1]<0) ) + st_row 
        return rot_center


    def nav_path(self,loc_car,shortest_path,img_shortest_path_,publisher_obj,velocity_obj,bot_localizer_obj,frame_disp):

        Doing_pt = 0
        Done_pt = 0

        path_i = self.path_iter
        path = shortest_path

        if (type(path)!=int):
            if (self.path_iter==0):
                self.goal_pose_x = shortest_path[path_i][0]
                self.goal_pose_y = shortest_path[path_i][1]

        self.move(shortest_path,loc_car,velocity_obj,publisher_obj)
        
        img_shortest_path = (img_shortest_path_).copy()
        # Car Loc
        img_shortest_path = cv2.circle(img_shortest_path, loc_car, 3, (0,0,255))

        if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
            curr_goal = path[path_i]
            # Mini Goal Completed
            if path_i!=0:
                img_shortest_path = cv2.circle(img_shortest_path, path[path_i-1], 3, (0,255,0))
                Done_pt = path[path_i-1]
            # Mini Goal Completing   
            img_shortest_path = cv2.circle(img_shortest_path, curr_goal, 3, (0,140,255))
            Doing_pt = curr_goal
        else:
            # Only Display Final Goal completed
            img_shortest_path = cv2.circle(img_shortest_path, path[path_i], 5, (0,255,0))
            Done_pt = path[path_i]

        if Doing_pt!=0:
            Doing_pt = self.bck_to_orig(Doing_pt, bot_localizer_obj.transform_arr, bot_localizer_obj.rot_mat_rev)
            frame_disp = cv2.circle(frame_disp, (int(Doing_pt[0]),int(Doing_pt[1])), 3, (0,140,255))            
            
        if Done_pt!=0:
            Done_pt = self.bck_to_orig(Done_pt, bot_localizer_obj.transform_arr, bot_localizer_obj.rot_mat_rev)
            if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
                frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 3, (0,255,0))   
            else:
                frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 5, (0,255,0))   
        print("[Doing_pt & Done_pt] ImgSize = [ {} & {} ] ({}) ".format(Doing_pt,Done_pt,frame_disp.shape))         
        
        cv2.imshow("maze (Shortest Path + Car Loc)",img_shortest_path)