import rclpy 
import cv2 
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 


class Video_get(Node):
  def __init__(self):
    super().__init__('video_subscriber')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
    ## setting for writing the frames into a video
    self.out = cv2.VideoWriter('/home/luqman/output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (1280,720))
    self.bridge = CvBridge() # converting ros images to opencv data
 
  def process_data(self, data): 
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    self.out.write(frame)# write the frames to a video
    cv2.imshow("output", frame) # displaying what is being recorded 
    cv2.waitKey(1) # will save video until it is interrupted
  

  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()