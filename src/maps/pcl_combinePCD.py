#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import argparse
import sys
from pathlib import Path
import gc

class PCDCombiner:
    def __init__(self, resolution=0.05, height_threshold=0.3):
        self.resolution = resolution
        self.height_threshold = height_threshold
        
    def load_and_process_pcd(self, pcd_path):
        try:
            print(f"Loading {pcd_path}...")
            pcd = o3d.io.read_point_cloud(str(pcd_path))
            
            # Downsample to reduce memory usage
            print(f"Downsampling point cloud...")
            pcd = pcd.voxel_down_sample(voxel_size=self.resolution)
            
            # Remove statistical outliers
            print(f"Removing outliers...")
            pcd, _ = pcd.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)
            
            return pcd
            
        except Exception as e:
            print(f"Error processing {pcd_path}: {str(e)}")
            return None

    def combine_pcds(self, pcd_files):
        combined_cloud = o3d.geometry.PointCloud()
        
        for pcd_file in pcd_files:
            # Process one file at a time
            current_cloud = self.load_and_process_pcd(pcd_file)
            if current_cloud is not None:
                print(f"Combining {pcd_file}...")
                combined_cloud += current_cloud
                
                # Force garbage collection
                gc.collect()
        
        return combined_cloud

    def create_occupancy_grid(self, pcd):
        print("Creating occupancy grid...")
        points = np.asarray(pcd.points)
        
        # Find boundaries
        min_x, min_y = np.min(points[:, :2], axis=0)
        max_x, max_y = np.max(points[:, :2], axis=0)
        
        # Calculate grid dimensions
        width = int((max_x - min_x) / self.resolution)
        height = int((max_y - min_y) / self.resolution)
        
        print(f"Grid dimensions: {width}x{height}")
        
        # Create grid in smaller chunks
        grid = np.zeros((height, width), dtype=np.int8)
        chunk_size = 1000000  # Process points in chunks
        
        for i in range(0, len(points), chunk_size):
            chunk = points[i:i + chunk_size]
            for point in chunk:
                x, y, z = point
                if z < -1.0 or z > self.height_threshold:
                    continue
                    
                grid_x = int((x - min_x) / self.resolution)
                grid_y = int((y - min_y) / self.resolution)
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    grid[grid_y, grid_x] = 100
            
            # Force garbage collection after each chunk
            gc.collect()
        
        return grid, min_x, min_y

    def save_map(self, grid, min_x, min_y, output_path):
        try:
            # Save as PGM
            print(f"Saving map to {output_path}.pgm...")
            with open(f"{output_path}.pgm", 'wb') as f:
                height, width = grid.shape
                f.write(f"P5\n{width} {height}\n255\n".encode())
                f.write(grid.astype(np.uint8).tobytes())
            
            # Save YAML configuration
            print(f"Saving configuration to {output_path}.yaml...")
            import yaml
            yaml_content = {
                'image': f"{Path(output_path).name}.pgm",
                'resolution': self.resolution,
                'origin': [min_x, min_y, 0.0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196
            }
            
            with open(f"{output_path}.yaml", 'w') as f:
                yaml.dump(yaml_content, f)
                
            print("Map saved successfully!")
            
        except Exception as e:
            print(f"Error saving map: {str(e)}")

def main():
    parser = argparse.ArgumentParser(description='Combine multiple PCD files and create occupancy grid')
    parser.add_argument('pcd_files', nargs='+', help='Input PCD files')
    parser.add_argument('--resolution', type=float, default=0.05, help='Map resolution in meters/pixel')
    parser.add_argument('--height', type=float, default=0.3, help='Height threshold for obstacles')
    parser.add_argument('--output', type=str, default='combined_map', help='Output file prefix')
    
    args = parser.parse_args()
    
    combiner = PCDCombiner(resolution=args.resolution, height_threshold=args.height)
    
    try:
        # Combine point clouds
        print("Starting point cloud combination...")
        combined_cloud = combiner.combine_pcds(args.pcd_files)
        
        # Create occupancy grid
        print("Creating occupancy grid from combined cloud...")
        grid, min_x, min_y = combiner.create_occupancy_grid(combined_cloud)
        
        # Save the map
        print("Saving final map...")
        combiner.save_map(grid, min_x, min_y, args.output)
        
    except Exception as e:
        print(f"Error: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main()