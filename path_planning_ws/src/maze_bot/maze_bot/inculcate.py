import rclpy 
import cv2 
from rclpy.node import Node 
from cv_bridge import CvBridge 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image 
import os

class Vision_drive(Node):
  def __init__(self):
    super().__init__('maze_solving_bot')# node name
    ## Created a Subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,60)
    vid_path = os.path.join(os.getcwd(),"ouster_robot.avi")
    self.out = cv2.VideoWriter(vid_path,cv2.VideoWriter_fourcc('M','J','P','G'), 30, (1280,720))
    self.bridge = CvBridge()
    ## Created a Publisher
    self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
    timer_period = 0.5;self.timer = self.create_timer(timer_period, self.send_cmd_vel) 
    self.velocity=Twist()

  def send_cmd_vel(self):
    ## you can set these values manually in other function calls as they are class variables
    self.velocity.linear.x = 1.0        
    self.velocity.angular.z = 0.5
    #self.publisher.publish(self.velocity)

  def process_data(self, data): 
      # Use OPENCV Here
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
    self.out.write(frame)
    cv2.imshow("output", frame) 
    cv2.waitKey(1) 
  

  
def main(args=None):
  rclpy.init(args=args)
  robot_drive_with_camera = Vision_drive()
  rclpy.spin(robot_drive_with_camera)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()