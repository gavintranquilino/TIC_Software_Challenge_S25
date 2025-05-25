from .IMU import IMU
from irobot_create_msgs.action import Dock,Undock
from geometry_msgs.msg import Twist
import time
import rclpy
import math
import threading
from pynput.keyboard import Listener
from .Robot import Robot

class Control:
    def __init__(self, robot : Robot):
        ''' Initializes the control object by storing the robot reference. '''
        self.robot = robot
        self.imu = IMU(self.robot)

    def set_cmd_vel(self, velocity_x : float, velocity_phi : float, duration : float):
        """ Sets up and initiates a timer-based process to continuously publish velocity commands for the specified duration. """
        self.velocity_x = velocity_x
        self.velocity_phi = velocity_phi
        self.end_time = time.time() + duration
        self.cmd_vel_future = rclpy.Future()
        timer_period = 0.01  # seconds
        self.cmd_vel_terminate = False
        self.cmd_vel_timer = self.robot.create_timer(timer_period, self.cmd_vel_timer_callback)
        rclpy.spin_until_future_complete(self.robot,self.cmd_vel_future)  

    def cmd_vel_timer_callback(self):
        ''' Acts as the timer callback that continuously publishes velocity commands until the duration expires. '''
        if self.cmd_vel_terminate:
            self.cmd_vel_future.set_result(None)
            self.cmd_vel_timer.cancel()
            return
        msg = Twist()
        if self.end_time<time.time():
            self.cmd_vel_terminate = True
        if self.cmd_vel_terminate:
            msg.linear.x = 0.
            msg.angular.z = 0.
        else:
            msg.linear.x = float(self.velocity_x)
            msg.angular.z = float(self.velocity_phi)
        self.robot.cmd_vel_publisher.publish(msg)
        
    def undock(self) -> Undock.Result:
        ''' Sends an asynchronous undock goal to the robot, waits for the action to complete, and returns the corresponding result. '''
        if not self.robot.IS_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.robot.undock_client.send_goal_async(Undock.Goal())
            rclpy.spin_until_future_complete(self.robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self.robot,action_completed_future)
            return action_completed_future.result()
        
    def dock(self) -> Dock.Result:
        ''' Sends an asynchronous dock goal to the robot, waits for the action to complete, and returns the resulting outcome. '''
        if not self.robot.IS_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.robot.dock_client.send_goal_async(Dock.Goal())
            rclpy.spin_until_future_complete(self.robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self.robot,action_completed_future)
            return action_completed_future.result()

    def rotate(self, angle : float, direction : int):
        ''' Continuously monitors the robot’s orientation using its IMU, issuing turning commands until the robot has rotated by the desired angle, then stops the rotation. '''
        q_initial = self.imu.checkImu().orientation
        _, _, yaw_start = self.imu.euler_from_quaternion(q_initial)
        yaw_start_deg = math.degrees(yaw_start)
        
        def minimal_angle_diff(start, current):
            diff = (current - start + 180) % 360 - 180
            return abs(diff)
        
        current_diff = 0.0
        
        while current_diff < abs(angle):
            q_current = self.imu.checkImu().orientation
            _, _, yaw_current = self.imu.euler_from_quaternion(q_current)
            yaw_current_deg = math.degrees(yaw_current)
            current_diff = minimal_angle_diff(yaw_start_deg, yaw_current_deg)
            
            self.send_cmd_vel(0.0, direction * 0.75)
            rclpy.spin_once(self.robot, timeout_sec=0.1)
        
        self.send_cmd_vel(0.0, 0.0)
        if self.robot.DEBUG:
            print("turn complete")

        
    def send_cmd_vel(self, linear_x : float, angular_z : float):
        ''' Publishes a twist message to robot\'s command velocity topic using the provided linear and angular velocity values. '''
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.robot.cmd_vel_publisher.publish(msg)

    def stop_keyboard_input(self):
        ''' Disables the processing of keyboard inputs by setting the robot\'s input flag to false. '''
        self.robot.input = False

    def start_keyboard_input(self):
        ''' Enables keyboard input processing by setting the robot\'s input flag to true. '''
        self.robot.input = True

    def start_keyboard_control(self):
        ''' Initiates a keyboard listener and a dedicated update thread that continuously checks pressed keys to translate them into movement commands. '''
        if self.robot.keyboard_listener is None:
            # This set holds keys that are currently pressed.
            pressed_keys = set()

            self.robot.stop_event = threading.Event()

            def update_command():
                # Check for combined key movements first.
                if 'w' in pressed_keys and 'a' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(0.5, 1.0)
                    return
                elif 'w' in pressed_keys and 'd' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(0.5, -1.0)
                    return
                elif 's' in pressed_keys and 'a' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(-0.5, 1.0)
                    return
                elif 's' in pressed_keys and 'd' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(-0.5, -1.0)
                    return

                # Process individual keys.
                if 'w' in pressed_keys:
                    if self.robot.input:
                        self.move_forward()
                elif 's' in pressed_keys:
                    if self.robot.input:
                        self.move_backward()
                elif 'a' in pressed_keys:
                    if self.robot.input:
                        self.turn_left()
                elif 'd' in pressed_keys:
                    if self.robot.input:
                        self.turn_right()
                else:
                    # If no keys are pressed, stop the movement.
                    self.send_cmd_vel(0.0, 0.0)

            def key_control_loop():
                while not self.robot.stop_event.is_set() and rclpy.ok():
                    update_command()
                    time.sleep(0.05)

            def on_press(key):
                try:
                    key_char = key.char
                except AttributeError:
                    key_char = str(key)

                pressed_keys.add(key_char)

            def on_release(key):
                try:
                    key_char = key.char
                except AttributeError:
                    key_char = str(key)

                pressed_keys.discard(key_char)


            # Start the keyboard listener.
            self.robot.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
            self.robot.keyboard_listener.start()
            
            # Start the continuous update thread. Make sure it is a daemon thread so it doesn't block shutdown.
            self.robot.update_thread = threading.Thread(target=key_control_loop, daemon=True)
            self.robot.update_thread.start()
            self.robot.get_logger().info('Keyboard control listener started.')
            self.sub_keyboard = None # Make sure this is reset

    def stop_keyboard_control(self):
        """Stops the keyboard control listener and its associated update thread."""
        listener_stopped = False
        thread_stopped = False

        # Stop the pynput listener
        if hasattr(self.robot, 'keyboard_listener') and self.robot.keyboard_listener is not None:
            self.robot.get_logger().info('Attempting to stop pynput keyboard listener...')
            try:
                # The pynput listener runs in its own thread. stop() signals it.
                self.robot.keyboard_listener.stop()
                # Join the listener's thread to ensure it has exited.
                # The Listener object itself is a subclass of threading.Thread.
                self.robot.keyboard_listener.join(timeout=1.0)
                if self.robot.keyboard_listener.is_alive():
                    self.robot.get_logger().warn('Pynput keyboard listener thread did not stop after join.')
                else:
                    self.robot.get_logger().info('Pynput keyboard listener stopped.')
                    listener_stopped = True
            except Exception as e:
                self.robot.get_logger().error(f'Exception while stopping pynput listener: {e}')
            finally:
                self.robot.keyboard_listener = None
        else:
            self.robot.get_logger().debug('No active pynput keyboard listener to stop (or already None).')
            listener_stopped = True # Effectively stopped if it doesn't exist or is None

        # Stop the command update thread
        if hasattr(self.robot, 'update_thread') and self.robot.update_thread is not None:
            self.robot.get_logger().info('Attempting to stop command update thread...')
            if hasattr(self.robot, 'stop_event') and self.robot.stop_event is not None:
                self.robot.stop_event.set() # Signal the loop to exit
            
            if self.robot.update_thread.is_alive():
                self.robot.update_thread.join(timeout=1.0)
                if self.robot.update_thread.is_alive():
                    self.robot.get_logger().warn('Command update thread did not stop after join.')
                else:
                    self.robot.get_logger().info('Command update thread stopped.')
                    thread_stopped = True
            else:
                self.robot.get_logger().info('Command update thread was not alive.')
                thread_stopped = True # Effectively stopped if not alive
            self.robot.update_thread = None
        else:
            self.robot.get_logger().debug('No active command update thread to stop (or already None).')
            thread_stopped = True # Effectively stopped if it doesn't exist or is None

        # Clear the stop event
        if hasattr(self.robot, 'stop_event'):
            self.robot.stop_event = None
        
        # Clean up sub_keyboard (from original code, start_keyboard_control sets it to None)
        if hasattr(self, 'sub_keyboard') and self.sub_keyboard is not None:
            self.robot.get_logger().info('Destroying keyboard subscriber (sub_keyboard).')
            try:
                self.sub_keyboard.destroy()
            except Exception as e:
                self.robot.get_logger().error(f'Error destroying sub_keyboard: {e}')
            finally:
                self.sub_keyboard = None
            
        if listener_stopped and thread_stopped:
            self.robot.get_logger().info('Keyboard control stopping process completed.')
        else:
            self.robot.get_logger().warn('Keyboard control may not have been fully stopped.')

    def teleop_twist_callback(self, msg):
        """Receives and processes Twist messages for robot movement control."""
        if self.robot.input:  # Check if input processing is enabled
            # Extract linear and angular velocities from the Twist message
            linear_x = msg.linear.x
            angular_z = msg.angular.z

            # Log the received velocities at the DEBUG level
            self.robot.get_logger().debug(f'Received linear_x: {linear_x}, angular_z: {angular_z}')

            # Scale the velocities for finer control
            scaled_linear_x = linear_x * self.robot.CONST_speed_control
            scaled_angular_z = angular_z * self.robot.CONST_speed_control

            # Log the scaled velocities at the DEBUG level
            self.robot.get_logger().debug(f'Scaled linear_x: {scaled_linear_x}, angular_z: {scaled_angular_z}')

            # Send the scaled velocities to the robot
            self.send_cmd_vel(scaled_linear_x, scaled_angular_z)
        else:
            self.robot.get_logger().info('Input processing is disabled. Twist messages will not control the robot.')
