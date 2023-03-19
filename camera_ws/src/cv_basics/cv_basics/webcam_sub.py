import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
from std_msgs.msg import Int64, Int16
from datetime import datetime
 
class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')

    self.img_subscription = self.create_subscription(Image, 'image_cv', self.listener_callback, 10)
    self.location_subscription = self.create_subscription(Int64, 'Location', self.location_callback, 10)
    self.assistHuman_subscription = self.create_subscription(Int16, "assist_human", self.assistHuman_callback, 10)

    self.wait = 0
    self.flag_l = 0
    self.flag_h = 0
    self.current_frame = None
    self.img_subscription 
    self.br = CvBridge()
   
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
    self.current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera", self.current_frame)
    if(self.flag_l > 4) and (self.flag_h == 1):
      self.wait = self.wait + 100
      if self.wait >= 8000:
          filename = '/home/meena/Desktop/logImages/Frame_' + str(datetime.now()) + '.jpg'
          cv2.imwrite(filename, self.current_frame)
          self.get_logger().info('screenshot taken')
          wait = 0
    cv2.waitKey(1)
  
  def location_callback(self, msg: Int64):
    self.location = msg.data
    if (self.location > 4):
      self.flag_l = msg.data
    else:
      self.flag_l = 0

  def assistHuman_callback(self, msg: Int16):
    self.human = msg.data
    if (self.human == 1):
      self.flag_h = msg.data
    else:
      self.flag_h = 0

      
    
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.screenshot()
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()