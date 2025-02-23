import math
import numpy as np

class DifferentialDriveVehicle:
    """
    A differential drive vehicle model that uses four motors 
    (two on each side) to control the vehicle.
    """
    def __init__(self,
                 position=[0, 0],
                 yaw=0,
                 track_width=0.2,     # distance between left and right wheels (meters)
                 max_motor_speed=0.5, # maximum wheel speed (m/s)
                 deterministic=True):
        """
        Args:
            position (list): Initial [x, y] position.
            yaw (float): Initial heading (radians).
            track_width (float): Distance between left and right wheels.
            max_motor_speed (float): Maximum speed for each motor.
            deterministic (bool): If False, you can add noise.
        """
        self.x = position[0]
        self.y = position[1]
        self.yaw = yaw
        self.track_width = track_width
        self.max_motor_speed = max_motor_speed
        self.deterministic = deterministic

    def step(self, left_motor_speed, right_motor_speed, dt):
        """
        Update the vehicle's state based on the left and right motor speeds.
        
        Args:
            left_motor_speed (float): Speed of left motors (m/s).
            right_motor_speed (float): Speed of right motors (m/s).
            dt (float): Time step (s).
        
        Returns:
            ((x, y), yaw): The updated state.
        """
        # Clamp motor speeds to the maximum
        left_motor_speed = np.clip(left_motor_speed, -self.max_motor_speed, self.max_motor_speed)
        right_motor_speed = np.clip(right_motor_speed, -self.max_motor_speed, self.max_motor_speed)
        
        # Compute forward velocity and yaw rate.
        v = (left_motor_speed + right_motor_speed) / 2.0
        yaw_rate = (right_motor_speed - left_motor_speed) / self.track_width
        
        # Update state
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt
        
        return ((self.x, self.y), self.yaw)
