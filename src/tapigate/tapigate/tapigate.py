import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray  # Import message type for PWM
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            depth=10
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile
        )
        self.pwm_publisher = self.create_publisher(Int16MultiArray, '/pwm_output', 10)

        self.min_distance = 0.4  # Minimum distance in meters
        self.front_angle_range = 26  # ±26 degrees, total 52 degrees

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_readings = len(ranges)

        if num_readings == 0:
            return
        
        degrees_per_index = 360 / num_readings
        center_index = num_readings // 2
        range_offset = int(self.front_angle_range / degrees_per_index)

        front_ranges = ranges[center_index - range_offset : center_index + range_offset]
        left_ranges = ranges[center_index + 30 : center_index + 60]
        right_ranges = ranges[center_index - 60 : center_index - 30]

        front_distance = min(filter(lambda x: x > 0.0, front_ranges), default=float('inf'))
        left_distance = min(filter(lambda x: x > 0.0, left_ranges), default=float('inf'))
        right_distance = min(filter(lambda x: x > 0.0, right_ranges), default=float('inf'))

        pwm_values = Int16MultiArray()
        
        if front_distance < self.min_distance:
            self.get_logger().info("Obstacle detected! Choosing a turn direction...")
            if left_distance > right_distance:
                pwm_values.data = [150, -150]  # Turn left
                self.get_logger().info("Turning left...")
            else:
                pwm_values.data = [-150, 150]  # Turn right
                self.get_logger().info("Turning right...")
        else:
            pwm_values.data = [200, 200]  # Move forward
            self.get_logger().info("Moving forward...")

        self.pwm_publisher.publish(pwm_values)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()