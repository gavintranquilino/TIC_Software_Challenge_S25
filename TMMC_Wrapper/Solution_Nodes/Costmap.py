import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid 
from geometry_msgs.msg import Pose 
import numpy as np
import math # i need trig functions

class CostmapNode(Node):
    def __init__(self, resolution=0.05, map_width_pixels=200, map_height_pixels=200, 
                 inflation_radius_meters=0.1, max_obstacle_cost=100, unknown_cost=-1): 
        super().__init__('costmap_node')
        self.get_logger().info('Costmap Node Initialized with new parameters')

        # Costmap params
        self.map_width_pixels = map_width_pixels # units of cells
        self.map_height_pixels = map_height_pixels # units of cells
        self.resolution = resolution     # meters / cell
        self.inflation_radius_meters_ = inflation_radius_meters
        self.max_obstacle_cost_ = max_obstacle_cost 
        self.unknown_cost_ = unknown_cost # Typically -1 for unknown

        # origin of the map in the world frame (bottom-left corner of cell (0,0))
        self.origin_x = - (self.map_width_pixels * self.resolution) / 2.0 # Center the map
        self.origin_y = - (self.map_height_pixels * self.resolution) / 2.0 # Center the map
        self.map_frame_id = "base_footprint" # Frame ID for the map

        # Initialize the costmap as a 2D numpy array
        self.costmap_ = np.full((self.map_height_pixels, self.map_width_pixels), 0, dtype=np.int8) 
        self.get_logger().info(f'Initialized costmap with shape: {self.costmap_.shape}, resolution: {self.resolution}')

        # Publisher for the OccupancyGrid
        self.costmap_publisher_ = self.create_publisher(OccupancyGrid, '/costmap', 10) # Changed topic to /costmap

        # Subscriber to the /scan topic (LiDAR)
        self.lidar_subscription_ = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Timer to periodically publish the costmap
        self.timer_period_ = 1.0  # seconds
        self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)

        self.get_logger().info(f'Subscribed to /scan and will publish OccupancyGrid to {self.costmap_publisher_.topic_name}')

    def _init_costmap(self):
        """initializes the costmap grid to default cell values"""
        self.costmap_.fill(self.unknown_cost_)

    def _polar_to_cartesian(self, range_val, angle_val):
        """Converts a single polar coordinate (range, angle) to Cartesian (x, y)."""
        # TODO: These (x, y) coordinates are in the LiDAR's frame.
        x = range_val * math.cos(angle_val)
        y = range_val * math.sin(angle_val)
        return x, y

    def _world_to_grid_cell(self, world_x, world_y):
        """Converts world coordinates to grid cell indices (gx, gy)."""
        gx = int((world_x - self.origin_x) / self.resolution)
        gy = int((world_y - self.origin_y) / self.resolution)
        return gx, gy

    def _mark_obstacle(self, grid_x, grid_y):
        """Marks a given grid cell as an obstacle if within map bounds."""
        if 0 <= grid_x < self.map_width_pixels and 0 <= grid_y < self.map_height_pixels:
            self.costmap_[grid_y, grid_x] = self.max_obstacle_cost_
        else:
            pass

    def _distance_to_point_meters(self, g_x1, g_y1, g_x2, g_y2):
        """Calculates Euclidean distance between two grid points, in meters."""
        return math.sqrt(((g_x2 - g_x1)**2) + ((g_y2 - g_y1)**2)) * self.resolution

    def _calculate_inflation_cost(self, dist_to_obstacle_meters):
        """Calculates inflation cost based on distance to an obstacle."""
        if dist_to_obstacle_meters > self.inflation_radius_meters_:
            return 0 # Outside inflation radius

        # (1.0 - (euclidean_dist / inflation_radius_)) linear gradient 

        cost = self.max_obstacle_cost_ * (1.0 - (dist_to_obstacle_meters / self.inflation_radius_meters_))
        return int(max(0, min(self.max_obstacle_cost_, cost)))


    def inflate_obstacles(self):
        """Inflates obstacles in the costmap."""
        inflation_radius_cells = int(self.inflation_radius_meters_ / self.resolution)
        
        obstacle_cells = []
        for r in range(self.map_height_pixels):
            for c in range(self.map_width_pixels):
                if self.costmap_[r, c] == self.max_obstacle_cost_:
                    obstacle_cells.append((c, r)) # (x, y) format

        # Now iterate through all cells and calculate their inflation based on found obstacles
        new_inflated_costmap = np.copy(self.costmap_)

        for r_cell in range(self.map_height_pixels):
            for c_cell in range(self.map_width_pixels):
                if new_inflated_costmap[r_cell, c_cell] == self.max_obstacle_cost_:
                    continue

        for r_obs in range(self.map_height_pixels):
            for c_obs in range(self.map_width_pixels):
            # for each index of the grid

                if self.costmap_[r_obs, c_obs] == self.max_obstacle_cost_: # Found an original obstacle

                    # Inflate surrounding cells
                    for dr in range(-inflation_radius_cells, inflation_radius_cells + 1):
                        for dc in range(-inflation_radius_cells, inflation_radius_cells + 1):
                            
                            curr_r, curr_c = r_obs + dr, c_obs + dc

                            if 0 <= curr_r < self.map_height_pixels and 0 <= curr_c < self.map_width_pixels:
                                dist_meters = self._distance_to_point_meters(c_obs, r_obs, curr_c, curr_r)
                                
                                if dist_meters <= self.inflation_radius_meters_:
                                    calculated_cost = self._calculate_inflation_cost(dist_meters)
                                    if calculated_cost > new_inflated_costmap[curr_r, curr_c]:
                                        new_inflated_costmap[curr_r, curr_c] = calculated_cost

        self.costmap_ = new_inflated_costmap


    def lidar_callback(self, scan_msg: LaserScan):
        self._init_costmap() # Re-initialize map on each scan, as per C++ logic

        for i, range_val in enumerate(scan_msg.ranges):
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                
                # Convert polar to Cartesian (in LiDAR frame)
                world_x_lidar_frame, world_y_lidar_frame = self._polar_to_cartesian(range_val, angle)

                world_x_map_frame = world_x_lidar_frame 
                world_y_map_frame = world_y_lidar_frame

                # Convert world coordinates (relative to map origin) to grid cell
                grid_x, grid_y = self._world_to_grid_cell(world_x_map_frame, world_y_map_frame)
                
                self._mark_obstacle(grid_x, grid_y)
        
        self.inflate_obstacles()
        # The OccupancyGrid will be published by the timer_callback using self.costmap_

    def timer_callback(self): 
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.map_frame_id

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.map_width_pixels
        grid_msg.info.height = self.map_height_pixels
        
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0 # No rotation

        # Flatten the 2D costmap array into a 1D list (row-major order)
        grid_msg.data = self.costmap_.flatten().tolist()
        
        self.costmap_publisher_.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Parameters from C++ main, adjust as needed
    resolution = 0.1  # meters/cell (C++ used 0.3, Python default was 0.05)
    grid_width = 100  # cells
    grid_height = 100 # cells
    inflation_radius_m = 0.5 # meters (C++ used 1.0)
    max_cost = 100    # Max cost for occupied cells
    
    costmap_node = CostmapNode(
        resolution=resolution,
        map_width_pixels=grid_width,
        map_height_pixels=grid_height,
        inflation_radius_meters=inflation_radius_m,
        max_obstacle_cost=max_cost
    )
    
    try:
        rclpy.spin(costmap_node)
    except KeyboardInterrupt:
        costmap_node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        costmap_node.destroy_node()
        rclpy.shutdown()
