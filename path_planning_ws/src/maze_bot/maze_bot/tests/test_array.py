import numpy as np
import cv2
from math import pow , atan2,sqrt , degrees,asin



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

def main():

    a = (1,1)
    b = (2,2)
    angle,_ = angle_n_dist(a,b)
    angle = np.interp(angle,[0,360],[360,0])
    print("angle with x axis by a line from {} to {} = {}".format(a,b,angle))

if __name__ == "__main__":
    main()