import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from enum import Enum

class StateManager(Node):
    def __init__(self):
        super().__init__('py_state_manager')
        self.state = self.State.LINE
        self.line_subscription = self.create_subscription(Twist, 'line_cmd_vel', self.line_callback, 10)
        self.laser_subscription = self.create_subscription(Twist, 'laser_cmd_vel', self.laser_callback, 10)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        
    class State(Enum):
        STOP = 1
        LINE = 2
        LASER = 3
        ROTATE = 4


    def line_callback(self, msg):
        print(self.state)

        if self.state == self.State.STOP:
            self.send_drive_cmd(0.0, 0.0)
            return

        if self.state == self.State.LINE:
            #self.get_logger().info('Line sends: "%s"' % msg.data)

            self.send_drive_cmd(msg.linear.x, msg.angular.z)

    def laser_callback(self, msg):
        print(self.state)
        if self.state == self.State.STOP:
            self.send_drive_cmd(0.0, 0.0)
            return
        
        if msg.angular.z != 0.0:
            self.state = self.State.LASER

            self.send_drive_cmd(msg.linear.x, msg.angular.z)
        else:
            self.state = self.State.LINE
    
    def send_drive_cmd(self, speed = 0.0, turn = 0.0):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        self.publisher_.publish(msg)

    def check_stop(self):
        if self.state == self.State.STOP:
            print("Enter stop state.")
            self.send_drive_cmd(0.0, 0.0)


def main(args=None):
    print("Start state_manager node.")

    rclpy.init(args=args)

    state_manager = StateManager()
    try:
        rclpy.spin(state_manager)
    except KeyboardInterrupt:
        state_manager.state = state_manager.State.STOP
    finally:
        state_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
