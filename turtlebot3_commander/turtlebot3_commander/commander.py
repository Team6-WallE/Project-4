#!/urs/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import datetime
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Int64, Int16

class MainNode(Node):
    def __init__(self):
        super().__init__('turtlebot3_commander')

        #Creating variables
        self.location_number_ = 0
        self.waypoint_no = 0
        self.waypoints = [[0.0, 0.0], [-0.5, 1.5], [-0.5, 3.0], [1.0, 2.0], [1.0, 0.5], [0.0, 0.0], [-0.5, 2.0], [0.0, 0.5], [0.0, 3.0]]
        # 4d waypoints
        # self.waypoints = [[0.0, 0.0], [1.0, -1.0], [3.0, 0.0], [5.0, -1.0], [6.0, 0.0], [2.0, -1.0], [4.0, -1.0], [1.0, -1.0], [6.0, -1.0]]
        # Gazebo coordinates
        # self.waypoints = [[-2.0, -0.5], [0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5], [-1.5, 0.5], [-0.5, 1.5], [0.5, -2.0], [2.0, -0.5]]
        self.waypoint_check = [0,0,0,0,0,0,0,0,0]

        self.state_ = 0 # 1 for auto, 2 for manual
        self.assist_mode = 1
        self.patrol_mode = 2
        self.mode_ = 0 # 1 for assist, 2 for patrol
        self.wait = 0
        self.robot_wait = 0
        self.time_msg = Int64()
        self.time_msg.data = 1
        self.status_msg = Int64()

        self.no_goal = 0
        self.api_goal = 1
        self.crowd_goal = 2
        self.goal_source = self.no_goal

        self.time = 20
        self.start_timer = 0

        self.reached = 1
        self.failed = 2
        self.moving2pose = 3
        self.next_goal = 4
        self.patrol = 5
        self.goal_status = self.no_goal

        self.assist_human = 0
        self.patrol_human = 0
        self.patrol_start = 0

        self.human_detected = 0 # 0 if no human detected, 1 if human detected
    
        # Creating subscriber and publisher to nodes
        ## API subscriber and publisher
        self.api_goal_subscriber_ = self.create_subscription(
            Int64, 'goal_number', self.navigation_callback, 10)
        self.api_status_publisher_ = self.create_publisher(
            Int64, 'goal_status', 10)
        self.api_mode_subscriber_ = self.create_subscription(
            Int64, 'mode_state', self.mode_callback, 10)
        self.web_state_sub_ = self.create_subscription(
            Int64, 'state', self.state_callback, 10)

        ##Crowd Detection camera subscriber
        self.crowd_subscriber_ = self.create_subscription(
            Int64, 'Location', self.navigation_callback, 10)

        ##Human Detection camera subscriber
        self.human_subscriber_ = self.create_subscription(
            Int16, 'is_detected', self.human_callback, 10)

        self.assist_stop_sub_ = self.create_subscription(
            Int16, 'assist_human', self.assist_human_callback, 10)

        self.status_pub_ = self.create_publisher(
            Int64, 'goal_status', 10)

        ## Time update publisher to webpage
        self.time_pub_ = self.create_publisher(
            Int64, 'time_update', 10)
        
        self.wait_timer = self.create_timer(1, self.wait_timer_callback)
        self.update_time = self.create_timer(1, self.update_time_callback)
        self.timer_ = self.create_timer(5, self.timer_callback)

        # Initialising position of robot at waypoint[0]
        self.navigator = BasicNavigator()
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.waypoints[self.waypoint_no][0]
        initial_pose.pose.position.y = self.waypoints[self.waypoint_no][1]
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.get_logger().info('Turtlebot3 Commander has been initialised')

    def update_time_callback(self):
        current_time = datetime.datetime.now()
        hour = current_time.hour
        minute = current_time.minute
        if self.state_ == 1:      # auto
            if  minute > 11 and minute < 18: 
                self.mode_ = self.assist_mode
                self.get_logger().info('Assist Mode')
                self.patrol_start = 0                                   #hour >= 8 and hour < 18:                
            elif minute >= 18 and minute <= 25:                               #hour >= 20 and hour <= 23:
                if self.goal_source != self.api_goal:
                    if self.patrol_start == 0:
                        self.waypoint_no = 7
                        self.time_pub_.publish(self.time_msg)                     ## To send to webpage
                        self.get_logger().info("Time Published!!!")
                        self.patrol_start = 1
                    self.mode_ = self.patrol_mode
                    self.get_logger().info('Patrol Mode')

    def timer_callback(self):
        if self.mode_ == self.assist_mode:
            if self.goal_source == self.crowd_goal or self.goal_source == self.no_goal:
                if self.assist_human != 1:
                    self.navigation()
                elif self.assist_human == 1:
                    self.navigator.cancelTask()
                    self.assist_human = 0
            elif self.goal_source == self.api_goal:
                self.navigation()               

            if self.goal_status == self.reached:
                if self.goal_source == self.crowd_goal:
                    self.get_logger().info("Going to crowd location: %d" % self.waypoint_no)
                    self.waypoint_check[self.waypoint_no] = 1
                    self.navigation()
                # elif self.goal_source == self.no_goal or self.goal_source != self.crowd_goal:
                #     self.waypoint_no = 0
                #     self.get_logger().info("Going to home")
                #     self.waypoint_check[self.waypoint_no] = 1
                #     self.navigation()

        elif self.mode_ == self.patrol_mode:
            if self.assist_human != 1:
                if self.goal_source == self.no_goal:
                    self.waypoint_no = 7
                elif self.goal_status == self.next_goal:
                    self.waypoint_no = self.waypoint_no + 1
                    if self.waypoint_no == 9:
                        self.waypoint_no = 7
                self.waypoint_check[self.waypoint_no] = 1
                self.navigation()
            elif self.assist_human == 1:
                self.navigator.cancelTask()
                self.assist_human = 0
            elif self.goal_source == self.crowd_goal:
                self.waypoint_check[self.waypoint_no] = 1
                self.navigation()

    def navigation_callback(self, msg):
        if msg.data > 0 and msg.data < 5:
            self.goal_source = self.api_goal
            self.waypoint_no = msg.data
            self.waypoint_check[self.waypoint_no] = 1
        elif msg.data >= 5 and msg.data <= 6:
            if self.goal_source != self.api_goal:
                self.goal_source = self.crowd_goal
                self.waypoint_no = msg.data 
                self.waypoint_check[self.waypoint_no] = 1
        else:
            self.waypoint_no = 0
            self.goal_status = self.no_goal         

    def navigation(self):
        if self.waypoint_check[self.waypoint_no] == 1:
            self.goal_pose.pose.position.x = self.waypoints[self.waypoint_no][0]
            self.goal_pose.pose.position.y = self.waypoints[self.waypoint_no][1]
            self.navigator.goToPose(self.goal_pose)
            self.goal_status = self.moving2pose
            result = self.navigator.getResult()

            if self.navigator.isTaskComplete() and self.goal_status == self.moving2pose:
                if self.mode_ == self.assist_mode:
                    if result == TaskResult.SUCCEEDED:
                        if self.goal_source == self.api_goal:
                            self.status_msg.data = 10
                            self.api_status_publisher_.publish(self.status_msg)
                            self.status_msg.data = 0
                        self.goal_source = self.no_goal
                        self.goal_status = self.reached
                        self.waypoint_check[self.waypoint_no] = 0
                        self.get_logger().info('Assist goal Reached!')
                    elif result == TaskResult.FAILED:
                        goal.data = 11
                        self.goal_status = self.failed
                        self.get_logger().info('Goal Failed!!!')
                elif self.mode_ == self.patrol_mode:
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info('Patrol goal Reached!')
                        self.waypoint_check[self.waypoint_no] = 0
                        self.goal_source = self.patrol
                        self.goal_status = self.next_goal

    def state_callback(self, msg):
        self.state_ = msg.data
        if self.state_ == 2:
            self.robot_wait = 1
        self.get_logger().info("State: %d" % self.state_)

    def mode_callback(self, msg):
        if self.state_ == 2 and self.robot_wait == 1:  # manual
            if msg.data == 1:
                self.mode_ = self.assist_mode
                self.get_logger().info('Assist Mode')
            elif msg.data == 2:
                self.mode_ = self.patrol_mode 
                self.get_logger().info('Patrol Mode')    
            self.robot_wait = 0   

    def human_callback(self, msg):
        self.patrol_human = msg.data  

    def assist_human_callback(self, msg):
        self.assist_human = msg.data

    def wait_timer_callback(self):
        return

def main(args=None):
    rclpy.init(args=args)
    mainNode = MainNode()
    rclpy.spin(mainNode)
    rclpy.shutdown()


    