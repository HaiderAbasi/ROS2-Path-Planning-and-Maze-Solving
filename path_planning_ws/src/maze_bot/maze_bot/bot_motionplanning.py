'''
> Purpose :
Module to perform motionplanning for helping the vehicle navigate to the desired destination

> Usage :
You can perform motionplanning by
1) Importing the class (bot_motionplanner)
2) Creating its object
3) Accessing the object's function of (nav_path). 
E.g ( self.bot_motionplanner.nav_path(bot_loc, path, self.vel_msg, self.velocity_publisher) )


> Inputs:
1) Robot Current location
2) Found path to destination
3) Velocity object for manipulating linear and angular component of robot
4) Velocity publisher to publish the updated velocity object

> Outputs:
1) speed              => Speed with which the car travels at any given moment
2) angle              => Amount of turning the car needs to do at any moment

Author :
Haider Abbasi

Date :
6/04/22
'''
import cv2
import numpy as np
from math import pow , atan2,sqrt , degrees,asin

from numpy import interp
import pygame
import os
pygame.mixer.init()
pygame.mixer.music.load(os.path.abspath('src/maze_bot/resource/aud_chomp.mp3'))

from . import config

class bot_motionplanner():


    def __init__(self):

        # counter to move car forward for a few iterations
        self.count = 0
        # State Variable => Initial Point Extracted?
        self.pt_i_taken = False
        # [Container] => Store Initial car location
        self.init_loc = 0

        # State Variable => Angle relation computed?
        self.angle_relation_computed = False

        # [Container] => Bot Angle [Image]
        self.bot_angle = 0
        # [Container] => Bot Angle [Simulation]
        self.bot_angle_s = 0
        # [Container] => Angle Relation Bw(Image & Simulation)
        self.bot_angle_rel = 0
        # State Variable ==> (Maze Exit) Not Reached ?
        self.goal_not_reached_flag = True
        # [Containers] ==> Mini-Goal (X,Y)
        self.goal_pose_x = 0
        self.goal_pose_y = 0
        # [Iterater] ==> Current Mini-Goal iteration
        self.path_iter = 0

        # [Containers] Previous iteration Case [Angle or Dist or Mini_Goal]
        self.prev_angle_to_turn = 0
        self.Prev_distance_to_goal = 0
        self.prev_path_iter = 0

        # [Iterators] Iterations elapsed since no change or backpeddling
        self.angle_not_changed = 0
        self.dist_not_changed = 0
        self.goal_not_changed =0
        self.goal_not_changed_long =0
        self.backpeddling = 0

        # [State Variables] Stuck on Wall? Ready to backpeddle
        self.trigger_backpeddling = False
        # [State Variables] Can't reach goal? Try next appropriate one
        self.trigger_nxtpt = False

        self.curr_speed = 0
        self.curr_angle = 0


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

        # We get the bot_turn_angle in simulation Using same method as Gotogoal.py
        quaternions = data.pose.pose.orientation
        (roll,pitch,yaw)=self.euler_from_quaternion(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
        yaw_deg = degrees(yaw)

        # [Maintaining the Consistency in Angle Range]
        if (yaw_deg>0):
            self.bot_angle_s = yaw_deg
        else:
            # -160 + 360 = 200, -180 + 360 = 180 . -90 + 360 = 270
            self.bot_angle_s = yaw_deg + 360
        
        #              Bot Rotation 
        #      (OLD)        =>      (NEW) 
        #   [-180,180]             [0,360]
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

    def display_control_mechanism_in_action(self,bot_loc,path,img_shortest_path,bot_localizer,frame_disp):
        Doing_pt = 0
        Done_pt = 0

        path_i = self.path_iter
        
        # Circle to represent car current location
        img_shortest_path = cv2.circle(img_shortest_path, bot_loc, 3, (0,0,255))

        if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
            curr_goal = path[path_i]
            # Mini Goal Completed
            if path_i!=0:
                img_shortest_path = cv2.circle(img_shortest_path, path[path_i-1], 3, (0,255,0),2)
                Done_pt = path[path_i-1]
            # Mini Goal Completing   
            img_shortest_path = cv2.circle(img_shortest_path, curr_goal, 3, (0,140,255),2)
            Doing_pt = curr_goal
        else:
            # Only Display Final Goal completed
            img_shortest_path = cv2.circle(img_shortest_path, path[path_i], 10, (0,255,0))
            Done_pt = path[path_i]

        if Doing_pt!=0:
            Doing_pt = self.bck_to_orig(Doing_pt, bot_localizer.transform_arr, bot_localizer.rot_mat_rev)
            frame_disp = cv2.circle(frame_disp, (int(Doing_pt[0]),int(Doing_pt[1])), 3, (0,140,255),2)   
            #loc_car_ = self.bck_to_orig(loc_car, bot_localizer_obj.transform_arr, bot_localizer_obj.rot_mat_rev)
            #frame_disp = cv2.circle(frame_disp, (int(loc_car_[0]),int(loc_car_[1])), 3, (0,0,255))
         
            
        if Done_pt!=0:
            Done_pt = self.bck_to_orig(Done_pt, bot_localizer.transform_arr, bot_localizer.rot_mat_rev)
            if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
                pass
                #frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 3, (0,255,0),2)   
            else:
                frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 10, (0,255,0))  

        st = "len(path) = ( {} ) , path_iter = ( {} )".format(len(path),self.path_iter)        
        
        frame_disp = cv2.putText(frame_disp, st, (bot_localizer.orig_X+50,bot_localizer.orig_Y-30), cv2.FONT_HERSHEY_PLAIN, 1.2, (0,0,255))
        if config.debug and config.debug_motionplanning:
            cv2.imshow("maze (Shortest Path + Car Loc)",img_shortest_path)
        else:
            try:
                cv2.destroyWindow("maze (Shortest Path + Car Loc)")
            except:
                pass

    @staticmethod
    def angle_n_dist(pt_a,pt_b):
        # Trignometric rules Work Considering.... 
        #
        #       [ Simulation/Normal Convention ]      [ Image ]
        #
        #                    Y                    
        #                     |                     
        #                     |___                     ____ 
        #                          X                  |     X
        #                                             |
        #                                           Y
        #
        # Solution: To apply same rules , we subtract the (first) point Y axis with (Second) point Y axis
        error_x = pt_b[0] - pt_a[0]
        error_y = pt_a[1] - pt_b[1]

        # Calculating distance between two points
        distance = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )

        # Calculating angle between two points [Output : [-Pi,Pi]]
        angle = atan2(error_y,error_x)
        # Converting angle from radians to degrees
        angle_deg = degrees(angle)

        if (angle_deg>0):
            return (angle_deg),distance
        else:
            # -160 +360 = 200, -180 +360 = 180,  -90 + 360 = 270
            return (angle_deg + 360),distance
        
        #             Angle bw Points 
        #      (OLD)        =>      (NEW) 
        #   [-180,180]             [0,360]

    def check_gtg_status(self,angle_to_turn,distance_to_goal):

        # ******** Computing change in (angle) over the past iteration ********
        change_angle_to_turn = abs(angle_to_turn-self.prev_angle_to_turn)

        # If angle is large and the its not changing verymuch and not already self.backpeddling
        if( (abs(angle_to_turn) >5) and (change_angle_to_turn<0.4) and (not self.trigger_backpeddling) ):
            self.angle_not_changed +=1
            # For a significant time angle not changed ---> Trigger self.backpeddling [Move car Reverse]
            if(self.angle_not_changed>200):
                self.trigger_backpeddling = True
        else:
            self.angle_not_changed = 0
        print("[prev,change,not_changed_iter,self.trigger_backpeddling] = [{:.1f},{:.1f},{},{}] ".format(self.prev_angle_to_turn,change_angle_to_turn,self.angle_not_changed,self.trigger_backpeddling))
        self.prev_angle_to_turn = angle_to_turn

        # ******** Computing change in (dist) over the past iteration ********
        change_dist = abs(distance_to_goal-self.Prev_distance_to_goal)

        # If dist is large and the its not changing verymuch and not already self.backpeddling
        if( (abs(distance_to_goal) >5) and (change_dist<1.2) and (not self.trigger_backpeddling) ):
            self.dist_not_changed +=1
            # For a significant time dist not changed ---> Trigger self.backpeddling [Move car Reverse]
            if(self.dist_not_changed>200):
                self.trigger_backpeddling = True
        else:
            self.dist_not_changed = 0
        print("[prev_d,change_d,not_changed_iter,self.trigger_backpeddling] = [{:.1f},{:.1f},{},{}] ".format(self.Prev_distance_to_goal,change_dist,self.dist_not_changed,self.trigger_backpeddling))
        self.Prev_distance_to_goal = distance_to_goal

        # ******** Computing change in (mini-goal) over the past iteration ********
        change_goal = self.prev_path_iter - self.path_iter
        # If mini-goal not changing and within reasonable distance
        if( (change_goal==0) and (distance_to_goal<30) ):
            self.goal_not_changed +=1
            # For a significant time goal not changed ---> Trigger nxtPt [Maybe goal out of reach]
            if(self.goal_not_changed>500):
                self.trigger_nxtpt = True
        # If mini-goal not changing for a long time
        elif(change_goal==0):
            self.goal_not_changed_long+=1
            if(self.goal_not_changed_long>1500):
                self.trigger_nxtpt = True
        else:
            self.goal_not_changed_long = 0
            self.goal_not_changed = 0
        print("[prev_g,change_g,not_changed_iter] = [{:.1f},{:.1f},{}] ".format(self.prev_path_iter,change_goal,self.goal_not_changed))
        self.prev_path_iter = self.path_iter

    @staticmethod
    def dist(pt_a,pt_b):
        error_x= pt_b[0] - pt_a[0]
        error_y= pt_a[1] - pt_b[1]
        return( sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) ) )

    def get_suitable_nxtpt(self,car_loc,path):
        extra_i = 1
        test_goal = path[self.path_iter+extra_i]
        
        while(self.dist(car_loc, test_goal)<20):
            extra_i+=1
            test_goal = path[self.path_iter+extra_i]
        print("Loading {} pt ".format(extra_i))
        self.path_iter = self.path_iter + extra_i -1


    def go_to_goal(self,bot_loc,path,velocity,velocity_publisher):

        # Finding the distance and angle between (current) bot location and the (current) mini-goal
        angle_to_goal,distance_to_goal = self.angle_n_dist(bot_loc, (self.goal_pose_x,self.goal_pose_y))

        # Computing the angle the bot needs to turn to align with the mini goal
        angle_to_turn = angle_to_goal - self.bot_angle

        # Setting speed of bot proportional to its distance to the goal
        speed = interp(distance_to_goal,[0,100],[0.2,1.5])
        self.curr_speed = speed
        # Setting steering angle of bot proportional to the amount of turn it is required to take
        angle = interp(angle_to_turn,[-360,360],[-4,4])
        self.curr_angle = angle

        print("angle to goal = {} Angle_to_turn = {} angle[Sim] {}".format(angle_to_goal,angle_to_turn,abs(angle)))
        print("distance_to_goal = ",distance_to_goal)

        if self.goal_not_reached_flag:
            self.check_gtg_status(angle_to_turn, distance_to_goal)

        # If car is far away , turn towards goal
        if (distance_to_goal>=2):
            velocity.angular.z = angle

        # In view of limiation of differential drive, adjust speed of car with the amount of turn
        # E.g [Larger the turn  ==> Less the speed]
        if abs(angle) < 0.4:
            velocity.linear.x = speed
        elif((abs(angle) < 0.8)):
            velocity.linear.x = 0.02
        else:
            velocity.linear.x = 0.0

        # If  trigger_backpeddling ==> Make Car reverse [For a few moments]
        if self.trigger_backpeddling:
            print("###> backpeddling (",self.backpeddling,") <###")
            if self.backpeddling==0:
                self.trigger_nxtpt = True
            # Making car reverse by setting linear component negative
            velocity.linear.x = -0.16
            velocity.angular.z = angle
            self.backpeddling+=1
            # Stop backpeddling after some time
            if self.backpeddling == 100:
                self.trigger_backpeddling = False
                self.backpeddling = 0
                print("###> backpeddling DONE <###")
        
        # Keep publishing the updated velocity until Final goal not reached
        if (self.goal_not_reached_flag) or (distance_to_goal<=1):
            velocity_publisher.publish(velocity)

        # If car is within reasonable distance of mini-goal
        if ((distance_to_goal<=8) or self.trigger_nxtpt):
            if self.trigger_nxtpt:
                if self.backpeddling:
                    # [Stuck on a WAll?] ==> Look for appropriate next mini-goal
                    self.get_suitable_nxtpt(bot_loc,path)
                self.trigger_nxtpt = False
                

            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            # final goal not yet reached, stop moving
            if self.goal_not_reached_flag:
                velocity_publisher.publish(velocity)

            # Reached the final goal
            if self.path_iter==(len(path)-1):
                # First Time?
                if self.goal_not_reached_flag:
                    # Set goal_not_reached_flag to False
                    self.goal_not_reached_flag = False
                    
                    # Play the party song, Mention that reached goal
                    pygame.mixer.music.load(os.path.abspath('src/maze_bot/resource/Goal_reached.wav'))
                    pygame.mixer.music.play()
            # Still doing mini-goals?
            else:
                # Iterate over the next mini-goal
                self.path_iter += 1
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]
                #print("Current Goal (x,y) = ( {} , {} )".format(path[self.path_iter][0],path[self.path_iter][1]))
                
                if pygame.mixer.music.get_busy() == False:
                    pygame.mixer.music.play()

    def nav_path(self,bot_loc,path,velocity,velocity_publisher):

        # If valid path Founds
        if (type(path)!=int):
            # Trying to reach first mini-goal
            if (self.path_iter==0):
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]

        if (self.count >20):

            if not self.angle_relation_computed:

                velocity.linear.x = 0.0
                # Stopping our car
                velocity_publisher.publish(velocity)
                # Extracting Car angle (Img) from car_InitLoc and car_FinalLoc after moving forward (50 iters)
                self.bot_angle, _= self.angle_n_dist(self.init_loc, bot_loc)
                self.bot_angle_init = self.bot_angle
                # Finding relation coeffiecient between car_angle (Image <-> Simulation)
                self.bot_angle_rel = self.bot_angle_s - self.bot_angle
                self.angle_relation_computed = True

            else:
                # [For noob luqman] : Extracting Car angle [From Simulation angle & S-I Relation]
                self.bot_angle = self.bot_angle_s - self.bot_angle_rel

                print("\n\nCar angle (Image From Relation) = {} I-S Relation {} Car angle (Simulation) = {}".format(self.bot_angle,self.bot_angle_rel,self.bot_angle_s))
                print("Car angle_Initial (Image) = ",self.bot_angle_init)
                print("Car loc {}".format(bot_loc))

                # Traversing through found path to reach goal
                self.go_to_goal(bot_loc,path,velocity,velocity_publisher)


        else:
            # If bot initial location not already taken
            if not self.pt_i_taken:
                # Set init_loc = Current bot location
                self.init_loc = bot_loc
                self.pt_i_taken = True
                
            # Keep moving forward for 20 iterations(count)
            velocity.linear.x = 1.0
            velocity_publisher.publish(velocity)

            self.count+=1