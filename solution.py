from TMMC_Wrapper import *
from TMMC_Wrapper.Solution_Nodes.Costmap import CostmapNode # Adjusted import for solution.py

import rclpy
from rclpy.parameter import Parameter # Added import
from rclpy.executors import MultiThreadedExecutor # Added import
import threading # Added import
import numpy as np
import math
import time
from ultralytics import YOLO

# --- Helper Functions ---
def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

challengeLevel = 2
is_SIM = False
Debug = False

# Initialization    
if not "robot" in globals():
    if is_SIM:
        # Set use_sim_time parameter for simulation
        robot = Robot(IS_SIM=is_SIM, DEBUG=Debug, node_init_parameters=[Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    else:
        robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
# Initialize CostmapNode
if is_SIM:
    # Set use_sim_time parameter for simulation
    costmap_node = CostmapNode(node_init_parameters=[Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
else:
    costmap_node = CostmapNode()

# Setup executor for CostmapNode and Robot node
executor = MultiThreadedExecutor()
executor.add_node(costmap_node)
executor.add_node(robot) # Add robot node to the executor

# Function to spin the executor in a separate thread
def spin_executor(executor):
    executor.spin()

# Start the executor in a daemon thread
executor_thread = threading.Thread(target=spin_executor, args=(executor,), daemon=True)
executor_thread.start()
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot) # Note: Consider using robot.get_logger() directly
lidar = Lidar(robot)

if challengeLevel <= 2:
    control.start_keyboard_control()

try:
    if challengeLevel == 0:
        while rclpy.ok():
            time.sleep(0.1)

    if challengeLevel == 1:
        # linear_speed = 0.2 # This was defined but not used after teleop-only change
        while rclpy.ok():
            time.sleep(0.1)

            # IMU data fetching can remain if needed for other parts or future development
            imu_sensor_data = imu.checkImu()
            if imu_sensor_data: # Ensure data is not None
                current_orientation_quat = imu_sensor_data.orientation
                _, _, current_heading_rad = imu.euler_from_quaternion(current_orientation_quat)
            else:
                robot.get_logger().warn("Challenge 1: Failed to get IMU data.")
                # time.sleep(0.1) # Optional: wait a bit before retrying or handling error
                # continue # If IMU is critical for this loop, otherwise can proceed

            if costmap_node.obstacle_imminently_close:
                robot.get_logger().info("Challenge 1: Imminent obstacle detected! Starting simplified avoidance maneuver.")
                control.stop_keyboard_control()

                # 1. Stop the robot for 0.3 seconds
                robot.get_logger().info("Challenge 1: Stopping robot.")
                control.set_cmd_vel(0.0, 0.0, duration=0.5) 

                # 2. Move backward at speed of -2.0 m/s for 5 seconds
                robot.get_logger().info("Challenge 1: Moving backward")
                control.set_cmd_vel(-5.0, 0.0, duration=5)
                
                # 3. Ensure robot is stopped after backward movement
                robot.get_logger().info("Challenge 1: Ensuring robot is stopped after backward movement.")
                control.set_cmd_vel(0.0, 0.0, duration=0.5) # Brief stop command

                robot.get_logger().info("Challenge 1: Avoidance maneuver complete. Restarting keyboard control.")
                control.start_keyboard_control()
            
            # No default movement, robot is controlled by teleop unless obstacle detected
            pass

    if challengeLevel == 2:
        while True:
            try:
                if not camera.ML_predict_stop_sign(camera.checkImage())[0]:
                    print("stop not detected")
                
                else:
                    print("stop detected")
                    # robot.get_logger().info("Challenge 2: stop sign detected")
            except:
                print(Exception)
                pass

        pass # Placeholder, remove or replace with your code

    if challengeLevel == 3:
        # --- Challenge Level 3 --- #
        # Add your code for Challenge Level 3 here
        pass # Placeholder, remove or replace with your code

finally:
    if challengeLevel <= 2:
        print("Stopping keyboard control...")
        control.stop_keyboard_control()
    
    print("Shutting down ROS 2...")
    rclpy.shutdown()
    
    # Wait for the executor thread to finish
    if 'executor_thread' in globals() and executor_thread.is_alive():
        print("Waiting for executor thread to join...")
        executor_thread.join()
    print("Program ended.")
