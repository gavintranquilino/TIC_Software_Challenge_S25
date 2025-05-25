from TMMC_Wrapper import *
from TMMC_Wrapper.Solution_Nodes.Costmap import CostmapNode # Adjusted import for solution.py

import rclpy
from rclpy.parameter import Parameter # Added import
from rclpy.executors import MultiThreadedExecutor # Added import
import threading # Added import
import numpy as np
import math
import time
import collections # Added import for deque
from ultralytics import YOLO

# --- Helper Functions ---
def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def perform_safety_maneuver(control, robot, current_lidar_instance): # Added current_lidar_instance
    """Performs a safety maneuver when an imminent obstacle is detected."""
    robot.get_logger().info("Safety Maneuver: Imminent obstacle detected! Starting simplified avoidance maneuver.")
    # control.stop_keyboard_control() # stop_keyboard_control is called by the main loop before this if needed

    # 1. Stop the robot
    robot.get_logger().info("Safety Maneuver: Stopping robot.")
    control.set_cmd_vel(0.0, 0.0, duration=2)

    # Parameters for backward clearance check
    BACKWARD_CLEARANCE_THRESHOLD_M = 0.3
    BACKWARD_CLEARANCE_CONE_CENTER_DEG = 180.0
    BACKWARD_CLEARANCE_CONE_OFFSET_DEG = 60.0

    ROTATION_STEP_DEG_FOR_CLEARANCE = 15.0
    ROTATION_DIRECTION_FOR_CLEARANCE = -1

    robot.get_logger().info("Safety Maneuver: Checking path behind before reversing...")
    while True:
        current_scan = current_lidar_instance.checkScan() # Use passed lidar instance
        if not current_scan:
            robot.get_logger().warn("Safety Maneuver: Failed to get Lidar scan for backward check. Proceeding to reverse with caution.")
            break 

        obstacle_behind_dist, _ = current_lidar_instance.detect_obstacle_in_cone(
            current_scan,
            BACKWARD_CLEARANCE_THRESHOLD_M,
            BACKWARD_CLEARANCE_CONE_CENTER_DEG,
            BACKWARD_CLEARANCE_CONE_OFFSET_DEG
        )

        if obstacle_behind_dist == -1:
            robot.get_logger().info("Safety Maneuver: Path behind is clear.")
            break
        else:
            robot.get_logger().info(f"Safety Maneuver: Path behind is blocked (obstacle at {obstacle_behind_dist:.2f}m). Rotating {ROTATION_STEP_DEG_FOR_CLEARANCE} degrees.")
            control.rotate(ROTATION_STEP_DEG_FOR_CLEARANCE, ROTATION_DIRECTION_FOR_CLEARANCE)
            time.sleep(0.2)

    robot.get_logger().info("Safety Maneuver: Moving backward")
    control.set_cmd_vel(-0.3, 0.0, duration=1.5) # Adjusted for more noticeable backward movement
    
    robot.get_logger().info("Safety Maneuver: Ensuring robot is stopped after backward movement.")
    control.set_cmd_vel(0.0, 0.0, duration=1)

    robot.get_logger().info("Safety Maneuver: Avoidance maneuver complete.")
    global challengeLevel
    if challengeLevel != 3:
        robot.get_logger().info("Safety Maneuver: Restarting keyboard control for C1/C2.")
        control.start_keyboard_control()
    else:
        robot.get_logger().info("Safety Maneuver: In C3, not restarting keyboard control.")

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
        OBSTACLE_DETECTION_DISTANCE_THRESHOLD = 0.2  # meters
        OBSTACLE_DETECTION_CONE_CENTER_DEG = 0.0  # degrees, front of the robot
        OBSTACLE_DETECTION_CONE_OFFSET_DEG = 30.0 # degrees, +/- 30 degrees from center (total 60 degree cone)

        while rclpy.ok():
            time.sleep(0.1)

            imu_sensor_data = imu.checkImu()
            if imu_sensor_data: 
                current_orientation_quat = imu_sensor_data.orientation
                _, _, current_heading_rad = imu.euler_from_quaternion(current_orientation_quat)
            else:
                robot.get_logger().warn("Challenge 1: Failed to get IMU data.")

            current_scan = lidar.checkScan()
            if current_scan:
                obstacle_dist, _ = lidar.detect_obstacle_in_cone(
                    current_scan,
                    OBSTACLE_DETECTION_DISTANCE_THRESHOLD,
                    OBSTACLE_DETECTION_CONE_CENTER_DEG,
                    OBSTACLE_DETECTION_CONE_OFFSET_DEG 
                )
                if obstacle_dist > -1: 
                    perform_safety_maneuver(control, robot, lidar) # Pass lidar instance
            else:
                robot.get_logger().warn("Challenge 1: Failed to get Lidar scan data.")
            
            pass

    if challengeLevel == 2:
        OBSTACLE_DETECTION_DISTANCE_THRESHOLD = 0.3  # meters
        OBSTACLE_DETECTION_CONE_CENTER_DEG = 0.0  # degrees, front of the robot
        OBSTACLE_DETECTION_CONE_OFFSET_DEG = 30.0 # degrees, +/- 30 degrees from center (total 60 degree cone)

        STOP_SIGN_DURATION = 3.0  # seconds to stay stopped
        STOP_SIGN_COOLDOWN = 10.0 # seconds to ignore the sign after stopping
        last_stop_sign_action_time = 0.0
        
        STOP_SIGN_APPROACH_WIDTH_THRESHOLD = 100  # pixels, adjust this based on testing
        approaching_stop_sign = False 
        stop_sign_detected_previously = False 

        while True: 
            time.sleep(0.1) 
            current_time = time.time()

            current_scan = lidar.checkScan()
            if current_scan:
                obstacle_dist, _ = lidar.detect_obstacle_in_cone(
                    current_scan,
                    OBSTACLE_DETECTION_DISTANCE_THRESHOLD,
                    OBSTACLE_DETECTION_CONE_CENTER_DEG,
                    OBSTACLE_DETECTION_CONE_OFFSET_DEG
                )
                if obstacle_dist > -1: 
                    if approaching_stop_sign:
                        robot.get_logger().info("C2: Safety maneuver triggered during stop sign approach. Cancelling approach and resuming keyboard.")
                        approaching_stop_sign = False
                        control.start_keyboard_control() 
                    perform_safety_maneuver(control, robot, lidar) # Pass lidar instance
                    last_stop_sign_action_time = time.time() 
                    stop_sign_detected_previously = False 
                    continue 
            else:
                robot.get_logger().warn("Challenge 2: Failed to get Lidar scan data.")

            cv_image = camera.rosImg_to_cv2() 
            
            if cv_image is not None:
                try:
                    is_stop_sign_visible, _, _, box_width, _ = camera.ML_predict_stop_sign(cv_image)
                    
                    if approaching_stop_sign:
                        if not is_stop_sign_visible:
                            robot.get_logger().info("C2: Stop sign lost during approach. Resuming keyboard control.")
                            approaching_stop_sign = False
                            control.start_keyboard_control() 
                        elif box_width > STOP_SIGN_APPROACH_WIDTH_THRESHOLD:
                            control.set_cmd_vel(1.0, 0.0, duration=1) # Ensure robot is stopped

                            robot.get_logger().info(f"C2: Approached stop sign (width {box_width}px). Stopping for {STOP_SIGN_DURATION}s.")

                            # Keyboard is already stopped from when approaching_stop_sign was set to True.
                            # Use set_cmd_vel for the timed stop, replacing time.sleep().
                            control.set_cmd_vel(0.0, 0.0, duration=STOP_SIGN_DURATION)

                            control.send_cmd_vel(0.0, -1.0)
                            control.set_cmd_vel(1.0, 0.0, duration=0.5) # Ensure robot is stopped after turn
                            control.send_cmd_vel(0.0, 1.0)

                            last_stop_sign_action_time = time.time() # Record time after stop is complete
                            approaching_stop_sign = False
                            robot.get_logger().info("C2: Stop sign action complete. Resuming keyboard control.")
                            control.start_keyboard_control() 
                        else: 
                            robot.get_logger().info(f"C2: Approaching stop sign (width {box_width}px). Keyboard remains paused. User should drive closer.")
                            control.set_cmd_vel(0.0, 0.0, duration=0.1) # Hold position briefly

                    elif is_stop_sign_visible and (current_time - last_stop_sign_action_time > STOP_SIGN_COOLDOWN):
                        if not stop_sign_detected_previously:
                             robot.get_logger().info(f"C2: New stop sign detected (width {box_width}px). Pausing keyboard. User should approach.")
                             approaching_stop_sign = True
                             control.stop_keyboard_control() 
                             last_stop_sign_action_time = current_time 

                    stop_sign_detected_previously = is_stop_sign_visible

                except Exception as e:
                    robot.get_logger().error(f"Challenge 2: An error occurred during stop sign detection: {e}")
                    if approaching_stop_sign: 
                        control.start_keyboard_control()
                    approaching_stop_sign = False 
                    stop_sign_detected_previously = False
            else:
                robot.get_logger().warn("Challenge 2: Failed to get camera image for stop sign detection.")
                if approaching_stop_sign: 
                    robot.get_logger().info("Challenge 2: Camera image lost during stop sign approach. Resuming keyboard control.")
                    approaching_stop_sign = False
                    control.start_keyboard_control()
                stop_sign_detected_previously = False
        
        pass 

    if challengeLevel == 3:
        robot.get_logger().info("Challenge Level 3: Simplified Autonomous Navigation Mode Activated.")

        AUTONOMOUS_FORWARD_SPEED = 0.3
        AUTONOMOUS_FORWARD_BURST_DURATION = 0.3 
        AUTONOMOUS_OBSTACLE_DIST_M = 0.4
        AUTONOMOUS_OBSTACLE_CONE_CENTER_DEG = 0.0
        AUTONOMOUS_OBSTACLE_CONE_OFFSET_DEG = 30.0
        REVERSE_DURATION = 0.1
        REVERSE_SPEED = -0.15
        TURN_ANGLE_DEG = 60.0

        STOP_SIGN_DURATION = 3.0
        STOP_SIGN_COOLDOWN = 10.0
        last_stop_sign_action_time = 0.0
        STOP_SIGN_APPROACH_WIDTH_THRESHOLD = 100
        approaching_stop_sign = False
        
        turn_history = collections.deque(maxlen=3) # Max history of 3 turns
        FORCED_TURN_DIRECTION_ON_STUCK = -1 # Clockwise

        while True:
            time.sleep(0.1)
            current_time = time.time()
            current_scan = lidar.checkScan()
            cv_image = camera.rosImg_to_cv2() 

            # --- Priority 1: Stop Sign Logic ---
            if cv_image is not None:
                try:
                    is_stop_sign_visible, _, _, box_width, _ = camera.ML_predict_stop_sign(cv_image)
                    
                    if is_stop_sign_visible and not approaching_stop_sign and \
                       (current_time - last_stop_sign_action_time > STOP_SIGN_COOLDOWN):
                        robot.get_logger().info(f"C3: Stop sign detected (width {box_width}px). Initiating approach.")
                        approaching_stop_sign = True
                        control.set_cmd_vel(0.0, 0.0, duration=0.1) 
                    
                    if approaching_stop_sign:
                        if not is_stop_sign_visible: 
                            robot.get_logger().warn("C3: Stop sign lost during approach. Cancelling.")
                            approaching_stop_sign = False
                        elif box_width > STOP_SIGN_APPROACH_WIDTH_THRESHOLD:
                            robot.get_logger().info(f"C3: Approached stop sign (width {box_width}px). Stopping for {STOP_SIGN_DURATION}s.")
                            control.set_cmd_vel(0.0, 0.0, duration=STOP_SIGN_DURATION) 
                            last_stop_sign_action_time = time.time()
                            approaching_stop_sign = False
                            robot.get_logger().info("C3: Stop sign action complete. Resuming navigation.")
                            turn_history.clear() # Clear history after stop sign interaction
                        else: 
                            robot.get_logger().info(f"C3: Approaching stop sign (width {box_width}px). Holding position.")
                            control.set_cmd_vel(0.0, 0.0, duration=0.1) 
                except Exception as e:
                    robot.get_logger().error(f"C3: Error in stop sign detection: {e}")
                    approaching_stop_sign = False
            
            if approaching_stop_sign: 
                robot.get_logger().debug("C3: Actively handling stop sign. Ensuring robot is stopped and skipping autonomous movement.")
                control.set_cmd_vel(0.0, 0.0, duration=0.1) 
                continue 
            # --- End Stop Sign Logic ---

            # --- Priority 2: Autonomous Obstacle Avoidance and Movement ---
            if current_scan:
                obstacle_dist, obstacle_angle_rad = lidar.detect_obstacle_in_cone(
                    current_scan, 
                    AUTONOMOUS_OBSTACLE_DIST_M, 
                    AUTONOMOUS_OBSTACLE_CONE_CENTER_DEG, 
                    AUTONOMOUS_OBSTACLE_CONE_OFFSET_DEG
                )

                if obstacle_dist > -1: 
                    robot.get_logger().info(f"C3: Obstacle at {obstacle_dist:.2f}m, angle {math.degrees(obstacle_angle_rad):.1f} deg. Reversing.")
                    control.set_cmd_vel(REVERSE_SPEED, 0.0, duration=REVERSE_DURATION)

                    determined_turn_direction = 0 
                    if obstacle_angle_rad is not None and obstacle_angle_rad != 0: 
                        determined_turn_direction = -1 if obstacle_angle_rad > 0 else 1 # -1 CW, 1 CCW
                    else: 
                        determined_turn_direction = 1 # Default to turning CCW (can be tuned)
                    
                    actual_turn_direction = determined_turn_direction

                    # Ping-pong detection: if history is [A, -A, A]
                    if (len(turn_history) == 3 and 
                        turn_history[0] == determined_turn_direction and # Current intended turn is same as oldest
                        turn_history[1] == -determined_turn_direction and # Middle turn was opposite
                        turn_history[2] == determined_turn_direction and # Most recent (before this one) was same as oldest
                        determined_turn_direction != 0):
                        robot.get_logger().warn("C3: Ping-pong detected (A, -A, A pattern with current intent)! Forcing a turn.")
                        actual_turn_direction = FORCED_TURN_DIRECTION_ON_STUCK
                        turn_history.clear() # Clear history to allow fresh evaluation after forced turn
                    
                    robot.get_logger().info(f"C3: Turning {TURN_ANGLE_DEG} degrees, actual direction: {actual_turn_direction} (intended: {determined_turn_direction}).")
                    if actual_turn_direction != 0:
                        control.rotate(TURN_ANGLE_DEG, actual_turn_direction)
                    
                    # Add the *actual* turn direction that was taken to history if it was a valid turn
                    # Or determined_turn_direction if we want to track intent before override?
                    # Let's track determined_turn_direction to detect the *pattern of intent* leading to stuck state.
                    if determined_turn_direction != 0:
                        turn_history.append(determined_turn_direction)
                    
                else: # Path is clear
                    robot.get_logger().info(f"C3: Path clear. Moving forward incrementally at {AUTONOMOUS_FORWARD_SPEED} m/s for {AUTONOMOUS_FORWARD_BURST_DURATION}s.")
                    control.set_cmd_vel(AUTONOMOUS_FORWARD_SPEED, 0.0, duration=AUTONOMOUS_FORWARD_BURST_DURATION)
                    if len(turn_history) > 0: # If it was turning before and now path is clear
                        robot.get_logger().debug("C3: Path cleared, clearing turn history.")
                        turn_history.clear() 
            
            else: # No Lidar scan
                robot.get_logger().warn("C3: No Lidar scan available. Stopping.")
                control.set_cmd_vel(0.0, 0.0, duration=0.1)
                turn_history.clear() # Clear history if Lidar fails
        
        pass
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
