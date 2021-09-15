from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
import rclpy 

class Car_driver(Node):
    def __init__(self):

        super().__init__('driving_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.5;self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity=Twist()
        
    def send_cmd_vel(self):

        self.velocity.linear.x = 2.0        
        self.velocity.angular.z = 0.5
        self.publisher.publish(self.velocity)

        
    
def main(args=None):
  rclpy.init(args=args)
  cmd_vel_publisher = Car_driver()
  rclpy.spin(cmd_vel_publisher)
  rclpy.shutdown()

if __name__ == '__main__':
	main()