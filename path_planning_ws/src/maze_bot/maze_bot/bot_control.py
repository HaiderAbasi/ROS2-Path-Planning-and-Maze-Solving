from math import pow , atan2,sqrt , degrees,asin
from numpy import interp
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
      
    
    def goal_movement_(self,path,car_loc,velocity_obj,publisher_obj):
        
        error_x = self.goal_pose_x - car_loc[0]
        error_y = self.goal_pose_y - car_loc[1]

        distance_to_goal = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )
        angle_to_goal = degrees(atan2(error_y,error_x))

        if(self.car_angle<0):
            angle_to_turn = angle_to_goal + self.car_angle
        else:
            angle_to_turn = angle_to_goal - self.car_angle

        velocity = self.bound(0.0,0.5,distance_to_goal)

        Angle = interp(angle_to_turn,[-180,180],[2,-2])

        print("angle to goal = {} Angle_to_turn = {} Angle[Sim] {}".format(angle_to_goal,angle_to_turn,abs(Angle)))
        print("self.goal_not_reached_flag = ",self.goal_not_reached_flag)
        print("distance_to_goal = ",distance_to_goal)

        if (distance_to_goal>=2):
            velocity_obj.angular.z = Angle
        if abs(Angle) < 0.3:
            velocity_obj.linear.x = velocity
        
        #self.get_logger().info('Error_X: {:.2f} Error_Y: {:.2f} DistanceTG: {:.2f} Turning_goal: {:.2f} Velocity: {:.2f}'  . format(error_x,error_y,distance_to_goal,angle_to_turn,velocity))

        if self.goal_not_reached_flag:
            publisher_obj.publish(velocity_obj)

        if (distance_to_goal<=2):

            velocity_obj.angular.z = 0.0
            if self.goal_not_reached_flag:
                publisher_obj.publish(velocity_obj)
            
            if self.path_iter==(len(path)):
                p = 1
                #pass
                #self.goal_not_reached_flag=False
            else:
                print("Current Goal (x,y) = ( {} , {} )".format(path[self.path_iter][0],path[self.path_iter][1]))
                self.path_iter += 1

            self.goal_pose_x = 290
            self.goal_pose_y = 160

    def goal_movement(self,path,car_loc,velocity_obj,publisher_obj):
        
        angle_to_goal, distance_to_goal = self.angle_n_dist(car_loc,(self.goal_pose_x,self.goal_pose_y))


        angle_to_turn = angle_to_goal - self.car_angle

        #velocity = interp(distance_to_goal,[0,100],[0.1,1])
        velocity = interp(distance_to_goal,[0,100],[0.1,1.5])

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

        

        if self.goal_not_reached_flag:
            #pass
            publisher_obj.publish(velocity_obj)

        if (distance_to_goal<=2):
            #pass
            velocity_obj.linear.x = 0.0
            velocity_obj.angular.z = 0.0
            if self.goal_not_reached_flag:
                #pass
                publisher_obj.publish(velocity_obj)

            self.path_iter += 1
            if self.path_iter==(len(path)):
                self.goal_not_reached_flag=False
            else:
                print("Current Goal (x,y) = ( {} , {} )".format(path[self.path_iter][0],path[self.path_iter][1]))
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]


            #self.goal_not_reached_flag=False
            #self.goal_pose_x = 290
            #self.goal_pose_y = 160




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
