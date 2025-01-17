#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import yaml

class DirectMapSaver(Node):
    def __init__(self):
        super().__init__('direct_map_saver')
        
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('height_threshold', 0.5)
        self.declare_parameter('padding', 0.5)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points2',
            self.cloud_callback,
            10
        )
        
        self.resolution = self.get_parameter('resolution').value
        self.height_threshold = self.get_parameter('height_threshold').value
        self.padding = self.get_parameter('padding').value
        
        self.get_logger().info('Waiting for point cloud data...')
    def clean_map(self, grid_data, width, height):
        # Reshape to 2D array
        map_2d = np.array(grid_data, dtype=np.int8).reshape(height, width)
        
        # Convert occupied cells to 255 (white)
        binary_map = np.zeros_like(map_2d, dtype=np.uint8)
        binary_map[map_2d == 100] = 255
        
        # Clean up noise
        kernel = np.ones((3,3), np.uint8)
        cleaned = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
        
        # Remove small objects
        num_labels, labels = cv2.connectedComponents(cleaned)
        min_size = 75
        
        for label in range(1, num_labels):
            if np.sum(labels == label) < min_size:
                cleaned[labels == label] = 0
        
        # Convert back to occupancy grid format
        result = np.full_like(map_2d, -1, dtype=np.int8)
        result[cleaned == 255] = 100
        return result.flatten()
    def save_map(self, grid_data, width, height, min_x, min_y, filename):
        # Save PGM file
        pgm_filename = f"{filename}.pgm"
        with open(pgm_filename, 'wb') as f:
            f.write(f"P5\n{width} {height}\n255\n".encode())
            
            pgm_data = np.zeros(grid_data.shape, dtype=np.uint8)
            pgm_data[grid_data == 100] = 0    # Occupied cells are black
            pgm_data[grid_data == 0] = 254    # Free cells are white
            pgm_data[grid_data == -1] = 205   # Unknown cells are gray
            
            f.write(pgm_data.tobytes())
        
        # Save YAML file
        yaml_filename = f"{filename}.yaml"
        yaml_data = {
            'image': pgm_filename,
            'resolution': self.resolution,
            'origin': [min_x, min_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(yaml_filename, 'w') as f:
            yaml.dump(yaml_data, f)
        
        self.get_logger().info(f'Saved map as {pgm_filename} and {yaml_filename}')
        
    def cloud_callback(self, msg):
        self.get_logger().info('Processing point cloud...')


        try:
            # Extract points
            x_coords = []
            y_coords = []
            
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if abs(float(p[2])) <= self.height_threshold and float(p[2]) > 0.25:  # Add height check
                    x_coords.append(float(p[0]))
                    y_coords.append(float(p[1]))
            
            if not x_coords:
                self.get_logger().warning('No valid points found')
                return
            
            # Convert to numpy arrays
            x_points = np.array(x_coords)
            y_points = np.array(y_coords)
            
            # Find boundaries
            min_x, max_x = np.min(x_points), np.max(x_points)
            min_y, max_y = np.min(y_points), np.max(y_points)
            
            # Add padding
            min_x -= self.padding
            min_y -= self.padding
            max_x += self.padding
            max_y += self.padding
            
            # Calculate grid size
            width = int((max_x - min_x) / self.resolution) + 1
            height = int((max_y - min_y) / self.resolution) + 1
            
            # Create occupancy grid
            grid_data = np.full(width * height, -1, dtype=np.int8)
            
            # Fill grid
            for x, y in zip(x_points, y_points):
                grid_x = int((x - min_x) / self.resolution)
                grid_y = int((y - min_y) / self.resolution)
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    idx = grid_y * width + grid_x
                    grid_data[idx] = 100
                    
                    # Add some padding around points
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            nx = grid_x + dx
                            ny = grid_y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                nidx = ny * width + nx
                                if grid_data[nidx] == -1:
                                    grid_data[nidx] = 100
            
            # Clean the map
            cleaned_data = self.clean_map(grid_data, width, height)
            
            # Save the map
            self.save_map(cleaned_data, width, height, min_x, min_y, 'map50')
            
            self.get_logger().info('Map saved successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

def main():
    rclpy.init()
    node = DirectMapSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()