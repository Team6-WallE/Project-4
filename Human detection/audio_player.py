from pydub import AudioSegment
from pydub.playback import play
from std_msgs.msg import Int64
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Int16

class AudioPublisher(Node):
    def __init__(self):
        super().__init__("audio_player")
        self.play_music = 0
        self.play = 0
        self.get_out= AudioSegment.from_wav ('get_out.wav')
        self.destination = AudioSegment.from_wav('destination.wav')
        self.song_subscriber = self.create_subscription(Int64, "/music", self.mode_callback, 10)
        self.dest_subscriber = self.create_subscription(Int64,"/goal_status", self.dest_callback, 10)
        self.create_timer(0.5, self.timer_callback)
        self.create_timer(0.2, self.dest_timer_callback)
    
    def dest_timer_callback(self):
        if self.play == 10:
            play(self.destination)
            self.play = 0

    def timer_callback(self):
        if self.play_music == 1:
            play(self.get_out)
            self.play_music = 0

    def mode_callback(self, msg :Int64):
        self.play_music = msg.data

    def dest_callback(self, msg: Int64):
        self.play = msg.data


def main(args=None):
    rclpy.init(args=args)
    audio_publisher = AudioPublisher()
    rclpy.spin(audio_publisher)
    audio_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()