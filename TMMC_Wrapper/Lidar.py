from .Robot import Robot
from sensor_msgs.msg import LaserScan

class Lidar:
    def __init__(self, robot : Robot):
        '''Initializes the Lidar object by storing a reference to the provided robot. '''
        self.robot = robot

    def checkScan(self) -> LaserScan:
        ''' Waits until a new scan message is received and returns that scan message. '''
        self.robot.spin_until_future_completed(self.robot.scan_future)
        return self.robot.last_scan_msg

    def detect_obstacle_in_cone(self, scan : LaserScan, distance : float, center : float, offset_angle : float) -> tuple[float, float]:
        ''' Analyzes the scan data within the cone to detect and return the distance and angle of the closest obstacle. '''
        if offset_angle < 0 or offset_angle > 180:
            raise ValueError("offset_angle must be between 0 and 180 degrees")
        
        if scan is None:
            print("Warning: No LIDAR Data")
            return -1,-1
                
        # Map center angle to the range [0, 360] instead of [-180, 180]
        while center >= 360:
            center = center - 360
        while center < 0:
            center = center + 360

        # Need to seperate the logic for simulation and real robot
        # because the angles are different in the two cases.
        # In simulation, the angles are in degrees and range from 0 to 360.
        # In the real robot, the angles are in degrees and range from 0 to 720.
        if self.robot.IS_SIM:  
            # Simulation case  
            # Simulated lidar starts with 0 degrees at the front of the robot and goes clockwise so no need to adjust the center angle
            center = int(center)
            offset_angle = int(offset_angle)        
            right = center - offset_angle
            left = center + offset_angle

            # Get the desired range
            if left >= 360:
                left = left - 360
            if right < 0:
                right = right + 360

            # Look from left to right going clockwise
            if right == left:
                relevent_range = scan.ranges[0:360]
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
            elif right > left:
                relevent_1 = scan.ranges[0:left]
                relevent_2 = scan.ranges[right: 360]
                relevent_range = relevent_1 + relevent_2
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
                # print(min_dist_index)
                if min_dist_index > left:
                    min_dist_index = min_dist_index + right - left
            else:
                relevent_range = scan.ranges[right:left]
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
                min_dist_index = min_dist_index + right
            
            min_dist_angle = min_dist_index
        else:
            # Non-simulation case
            # Non-simulated lidar starts with 0 degrees at the right side of the robot and goes counter-clockwise
            # so we need to adjust the center angle by 90 degrees
            if center < 270:
                center += 90
            else:
                center -= 270
                
            # need to map everything to half-angles
            center = int(center * 2)
            offset_angle = int(offset_angle * 2)
            
            # Get the relevent range
            right = center - offset_angle
            left = center + offset_angle
            if left >= 720:
                left = left - 720
            if right < 0:
                right = right + 720
                
            # Look from left to right going clockwise
            if right == left:
                relevent_range = scan.ranges[0:720]
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
            elif right > left:
                relevent_1 = scan.ranges[0:left]
                relevent_2 = scan.ranges[right: 720]
                relevent_range = relevent_1 + relevent_2
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
                if min_dist_index > left:
                    min_dist_index = min_dist_index + right - left
            else:
                relevent_range = scan.ranges[right:left]
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
                min_dist_index = min_dist_index + right
            
            # Remap the half angle to the full angle
            min_dist_angle = min_dist_index / 2.0 - 90
            if min_dist_angle < 0:
                min_dist_angle = min_dist_angle + 360
            
        # Put result back to the original range of [-180, 180]
        if min_dist_angle >= 180:
            min_dist_angle = min_dist_angle - 360

        # Check if the minimum distance is less than or equal to the specified distance
        if min_dist <= distance:
            return min_dist, min_dist_angle
        
        # If no obstacle is detected, return -1 for both distance and angle
        return -1, -1
