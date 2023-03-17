import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int8

class ImageSub(Node):
  def __init__(self):
    super().__init__('Location_pub')
    self.modesub = self.create_subscription(Int64, 'mode_state', self.mode_callback, 10)
    self.sub1 = self.create_subscription(Int8, 'Image1', self.listener1_callback, 10)
    self.sub2 = self.create_subscription(Int8, 'Image2', self.listener2_callback, 10)
    self.pub = self.create_publisher(Int64, 'Location', 10)
    self.timer = self.create_timer(30, self.timer_callback)
    self.patrol = self.create_timer(1, self.patrol_callback)
    self.image1_msg = Int8()
    self.image2_msg = Int8()
    self.mode = Int8()
    self.mode.data = 1
  
  def mode_callback(self, msg):
    self.mode.data = msg.data
  
  def listener1_callback(self, cam1):
    self.image1_msg.data = cam1.data
    self.get_logger().info('Image1 Boxes: %d' % cam1.data)

  def listener2_callback(self, cam2):
    self.image2_msg.data = cam2.data
    self.get_logger().info('Image2 Boxes: %d' % cam2.data)   
   
  def timer_callback(self):
    if self.mode.data == 1:
      location = Int64()
      location.data = 0
      if self.image1_msg.data > self.image2_msg.data:
        location.data = 5
      else :
        if self.image2_msg.data > 0:
          location.data = 6

      if location.data != 0:
        self.pub.publish(location)
   
  def patrol_callback(self):
    if self.mode.data == 0:
      location = Int64()
      location.data = 0
    
      if self.image1_msg.data != 0:
        location.data = 5
        self.pub.publish(location)
        self.image1_msg.data = 0
        
      if self.image2_msg.data != 0:
        location.data = 6
        self.pub.publish(location)
        self.image2_msg.data = 0
    
  
def main():
  
  rclpy.init()
  image_sub = ImageSub()
  rclpy.spin(image_sub)

  image_sub.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
