import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
from std_msgs.msg import Int16
from std_msgs.msg import Int64


class ImagePublisher(Node):
  def __init__(self):
    super().__init__('image_publisher')
    self.detected = 0
    self.location = 0
    self.publisher_ = self.create_publisher(Image, 'image_cv', 10)
    timer_period = 0.1  # seconds
 

    self.timer = self.create_timer(timer_period, self.timer_callback)
  #  self.screenshot_timer = self.create_timer(5, self.screenshot_timer_callback)
    self.cap = cv2.VideoCapture(2)
    self.br = CvBridge()
   
  def timer_callback(self):
    ret, frame = self.cap.read()
          
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
    self.get_logger().info('Publishing video frame')

  def mode_callback(self, msg :Int16):
    self.detected = msg.data

  def location_callback(self, msg: Int64):
    self.location = msg.data

def main(args=None):
  
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
