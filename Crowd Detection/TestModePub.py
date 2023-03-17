import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int8

class ModePub(Node):
  def __init__(self):
    super().__init__('Mode_pub')
    self.pub = self.create_publisher(Int8, 'Mode', 10)
    self.timer = self.create_timer(60, self.timer_callback)
    self.mode = Int8()
    self.mode.data = 0
    
def timer_callback(self):
    if self.mode.data == 0:
        self.mode.data = 1
    else:
        self.mode.data = 0

    self.pub.publish(mode)

def main():
  
  rclpy.init()
  Mode_pub = ModePub()
  rclpy.spin(Mode_pub)

  Mode_pub.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()