#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class SimpleConverter(Node):
    def __init__(self):
        super().__init__('simple_converter')
        
        # QoS for point cloud subscription
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for map publication - matching map_saver requirements
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points2',
            self.cloud_callback,
            cloud_qos
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )
        
        # Create timer for periodic map publishing
        self.create_timer(1.0, self.publish_map)
        self.latest_map = None
        
        self.get_logger().info('Simple converter initialized with corrected QoS')
    
    def extract_points(self, cloud_msg):
        points = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])
            points.append((x, y, z))
        return points

    def create_map_from_points(self, points, msg_header):
        points_array = np.array(points)
        x_points = points_array[:, 0]
        y_points = points_array[:, 1]
        
        # Find boundaries
        min_x, max_x = np.min(x_points), np.max(x_points)
        min_y, max_y = np.min(y_points), np.max(y_points)
        
        # Add padding
        padding = 0.5
        min_x -= padding
        min_y -= padding
        max_x += padding
        max_y += padding
        
        # Create grid
        resolution = 0.05
        width = int((max_x - min_x) / resolution) + 1
        height = int((max_y - min_y) / resolution) + 1
        
        # Create occupancy grid message
        grid_msg = OccupancyGrid()
        grid_msg.header = msg_header
        grid_msg.header.frame_id = 'map'
        
        grid_msg.info.resolution = resolution
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.origin.position.x = min_x
        grid_msg.info.origin.position.y = min_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Initialize grid data
        grid_data = np.full(width * height, -1, dtype=np.int8)
        
        # Fill grid
        for x, y, z in points:
            if abs(z) > 0.5:
                continue
                
            grid_x = int((x - min_x) / resolution)
            grid_y = int((y - min_y) / resolution)
            
            if 0 <= grid_x < width and 0 <= grid_y < height:
                idx = grid_y * width + grid_x
                grid_data[idx] = 100
                
                # Add padding around occupied cells
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        nx = grid_x + dx
                        ny = grid_y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            nidx = ny * width + nx
                            if grid_data[nidx] == -1:
                                grid_data[nidx] = 100
        
        grid_msg.data = grid_data.tolist()
        return grid_msg

    def cloud_callback(self, msg):
        self.get_logger().info('Received point cloud')
        try:
            points = self.extract_points(msg)
            if points:
                self.latest_map = self.create_map_from_points(points, msg.header)
                self.publish_map()
            else:
                self.get_logger().warn('No points in cloud')
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def publish_map(self):
        if self.latest_map is not None:
            self.latest_map.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.latest_map)
            self.get_logger().info('Published map')

def main():
    rclpy.init()
    converter = SimpleConverter()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()