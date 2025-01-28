import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleDriving(Node):

    def __init__(self):
        super().__init__('drive_with_laserscanner')
        
        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('distance_to_follow', 2.5)

        # variable for the last sensor reading
        self.min_distance = 0
        self.angle_to_closest_obj = 0

        # control whether scanning is active
        self.scanning_enabled = True

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=1)

        # create subscribers for laser scan data
        self.cam_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        #self.odometry_subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'laser_cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        #timer_period = 0.5  # seconds
        #self.my_timer = self.create_timer(timer_period, self.timer_callback)

    def scanner_callback(self, msg):
        if not self.scanning_enabled:
            return  # Skip processing if scanning is disabled

        non_zero_elements = [(i, val) for i, val in enumerate(msg.ranges) if val != 0.0]
        if non_zero_elements:
            self.angle_to_closest_obj, self.min_distance = min(non_zero_elements, key=lambda x: x[1])
        else:
            print("No close obstacles.")
            return

        # Check if an object is directly in front
        if (self.angle_to_closest_obj > 330 or self.angle_to_closest_obj < 30) and self.min_distance < 0.3:
            self.get_logger().info("Object detected in front. Initiating 180° rotation.")
            self.rotate_180()
        else:
            print("No object in front.")
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

    def rotate_180(self):
        self.scanning_enabled = False  # Disable scanning during rotation

        msg = Twist()
        angular_speed = 1.0  # rad/s
        duration = 3 / angular_speed  # Time to rotate 180° (π radians)

        msg.linear.x = 0.0
        msg.angular.z = angular_speed  # Set angular velocity

        # Publish the rotation command for the required duration
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.publisher_.publish(msg)

        # Stop rotation
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

        self.scanning_enabled = True  # Re-enable scanning after rotation

    # def timer_callback(self):
    #     # Normal driving logic
    #     distance_follow = self.get_parameter("distance_to_follow").get_parameter_value().double_value
    #     distance_stop = self.get_parameter("distance_to_stop").get_parameter_value().double_value

    #     # no or far away obstacle
    #     if (self.min_distance == 0) or (self.min_distance > distance_follow):
    #         speed = 0.0
    #     elif (self.min_distance <= distance_follow) and (self.min_distance > distance_stop):
    #         speed = self.min_distance / 5
    #     else:
    #         speed = 0.0

    #     turn = self.angle_to_closest_obj / 100
    #     if speed > 0.15:
    #         speed = 0.15

    #     msg = Twist()
    #     msg.linear.x = speed
    #     msg.angular.z = turn
    #     self.publisher_.publish(msg)


def main(args=None):
    print('Start laser_scan node.')

    rclpy.init(args=args)
    node = SimpleDriving()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
