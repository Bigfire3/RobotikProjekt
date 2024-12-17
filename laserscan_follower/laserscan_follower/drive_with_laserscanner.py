"""
Simple driving node that is based on the driving behavior of a simple vacuum cleaner robot: The robot turns as long as
an obstacle is detected in the defined area, otherwise it drives straight ahead. To detect obstacles, only one measurement
value is used per scan of the laser scanner.
"""

"""
qos_policy = clpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1
    )
self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile=qos_policy)
"""

import rclpy
import rclpy.node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleDriving(rclpy.node.Node):

    def __init__(self):
        super().__init__('drive_with_scanner')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('distance_to_follow', 2.5)
        self.declare_parameter('laserscan_beam_to_use', 0)

        # variable for the last sensor reading
        self.min_distance = 0
        self.angle_to_closest_obj = 0

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for laser scan data with changed qos
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)


    # handling received laser scan data
    def scanner_callback(self, msg):
        non_zero_elements = [(i, val) for i, val in enumerate(msg.ranges) if val != 0.0]
        if non_zero_elements:
            self.angle_to_closest_obj, self.min_distance = min(non_zero_elements, key=lambda x: x[1])
        else:
            print("No close obstacles.")

        if self.angle_to_closest_obj > 180:
            self.angle_to_closest_obj = -(360 - self.angle_to_closest_obj)
        print(f"min_index: {self.angle_to_closest_obj}, min_value: {self.min_distance}")

    # driving logic
    def timer_callback(self):

        # caching the parameters for reasons of clarity
        distance_follow = self.get_parameter("distance_to_follow").get_parameter_value().double_value
        distance_stop = self.get_parameter("distance_to_stop").get_parameter_value().double_value

        # no or far away obstacle
        if (self.min_distance == 0) or (self.min_distance > distance_follow):
            speed = 0

        elif (self.min_distance <= distance_follow) and (self.min_distance > distance_stop):
            speed = self.min_distance / 5
        
        # obstacle close enough to stop
        else:
            speed = 0.0

        turn = self.angle_to_closest_obj / 100

        if speed > 0.15: speed = 0.15
        print("Speed:", speed)

        # create message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        # send message
        self.publisher_.publish(msg)


def main(args=None):

    print('Hi from robotik_projekt.')
    rclpy.init(args=args)

    node = SimpleDriving()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()