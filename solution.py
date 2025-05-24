from TMMC_Wrapper import *
from TMMC_Wrapper.Solution_Nodes.Costmap import CostmapNode # Adjusted import for solution.py

import rclpy
from rclpy.executors import MultiThreadedExecutor # Added import
import threading # Added import
import numpy as np
import math
import time
from ultralytics import YOLO

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 1

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = True

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
# Initialize CostmapNode
costmap_node = CostmapNode() # Added CostmapNode instantiation

# Setup executor for CostmapNode
executor = MultiThreadedExecutor()
executor.add_node(costmap_node)

# Function to spin the executor in a separate thread
def spin_executor(executor):
    executor.spin()

# Start the executor in a daemon thread
executor_thread = threading.Thread(target=spin_executor, args=(executor,), daemon=True)
executor_thread.start()
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing

    if challengeLevel == 1:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            
            time.sleep(0.1)
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall

    if challengeLevel == 2:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            
            time.sleep(0.1)
            # Write your solution here for challenge level 2
            
    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    
    # Shutdown CostmapNode and its executor
    if 'executor' in globals() and executor:
        executor.shutdown()
    if 'costmap_node' in globals() and costmap_node:
        costmap_node.destroy_node()
        
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
