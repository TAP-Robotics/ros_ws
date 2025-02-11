import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

class LazySLAM(Node):
    def __init__(self):
        super().__init__('lazy_slam')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/lazy_map', 10)

        # Rolling occupancy grid (10m × 10m, 10cm resolution)
        self.map_size = 10  
        self.resolution = 0.1  
        self.grid_size = int(self.map_size / self.resolution)
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

    def scan_callback(self, msg):
        scan = np.nan_to_num(msg.ranges, nan=10.0)  # Convert NaNs to max range
        angles = np.linspace(msg.angle_min, msg.angle_max, len(scan))

        # Reset old grid (rolling effect)
        self.occupancy_grid.fill(0)

        # Convert LiDAR scan into occupancy grid
        for r, theta in zip(scan, angles):
            if r < self.map_size / 2:  # Only keep nearby obstacles
                x = int((r * np.cos(theta)) / self.resolution + self.grid_size / 2)
                y = int((r * np.sin(theta)) / self.resolution + self.grid_size / 2)
                if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                    self.occupancy_grid[x, y] = 100  # Mark obstacle

        self.publish_map()

    def publish_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size
        grid_msg.info.origin.position.x = -self.map_size / 2
        grid_msg.info.origin.position.y = -self.map_size / 2
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        self.map_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LazySLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()