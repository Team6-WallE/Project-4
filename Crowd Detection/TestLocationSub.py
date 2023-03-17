import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class ImageSub(Node):
  def __init__(self):
    super().__init__('Location_sub')
    self.sub = self.create_subscription(Int64, 'Location', self.listener_callback, 10)
  
  def listener_callback(self, msg):
    self.get_logger().info('Location: %d' % msg.data)

  
def main():
  
  rclpy.init()
  image_sub = ImageSub()
  rclpy.spin(image_sub)

  image_sub.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
