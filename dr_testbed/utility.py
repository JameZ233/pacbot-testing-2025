from dr_testbed.simulator import Simulator
from dr_testbed.render import Renderer
import numpy as np
import pygame

class WayPoint:
    """
    A class to represent a waypoint.
    """
    def __init__(self, 
                 pos,
                 dir, 
                 ):
        """
        Args:
            pos (tuple): position of the waypoint.
            norm (tuple): direction of the waypoint.
        """
        assert is_coordinate(pos), "Position is not a valid position vector"
        assert is_coordinate(dir), "Direction is not a valid direction vector"

        self.pos = pos
        self.dir = (-dir[0], dir[1])
        
        self.next_waypoints = []

    def set_next_waypoints(self, next_waypoints):
        for waypoint in next_waypoints:
            assert isinstance(waypoint, WayPoint), "Next waypoint is not a \
                Waypoint object"

        self.next_waypoints = next_waypoints

def is_coordinate(coordinate):
    """
    Check if a coordinate is valid.
    
    Args:
        coordinate (tuple): the coordinate to check.
        
    Returns:
        bool: True if the coordinate is valid, False otherwise.
    """
    if not isinstance(coordinate, tuple):
        return False
    if len(coordinate) != 2:
        return False
    if not all(isinstance(i, float) for i in coordinate):
        return False
    return True

def calc_traj(point_0, point_1, result_type = 'list'):
    """
    Calculate the trajectory between two waypoints.
    
    Args:
        point_0 (WayPoint): the first waypoint.
        point_1 (WayPoint): the second waypoint.
        result_type (str): the type of result to return. Can be 'list' or 
            'function'. If 'list', a list of points is returned. If 'function',
            a lambda function for the trajectory is returned.

    """
    # Points
    P0 = np.array(point_0.pos)
    P1 = np.array(point_1.pos)

    # Direction vectors
    D0 = np.array(point_0.dir)
    D1 = -np.array(point_1.dir)

    # Control points (adjust the t parameter as needed)
    t = 0.5
    C0 = P0 + t * D0
    C1 = P1 + t * D1

    # Generate the curve
    t_values = np.linspace(0, 1, 100)
    curve_np = (1-t_values)**3 * P0[:,None] + \
            3*(1-t_values)**2 * t_values * C0[:,None] + \
            3*(1-t_values) * t_values**2 * C1[:,None] + \
            t_values**3 * P1[:,None]
    
    curve = []
    for i in range(len(curve_np[0])):
        curve.append((curve_np[0][i], curve_np[1][i]))
    return curve

class Joy():
    """
    Joystick class that takes input from a joystick and return the speed and
    steering angle of the vehicle.
    """
    def __init__(self, max_speed=0.5, max_steering_angle=15, verbose=False):
        """
        Args:
            max_speed (float): maximum speed of the vehicle
            max_steering_angle (float): maximum steering angle of the vehicle
            verbose (boolean): print the input values
        """
        self.max_speed = max_speed
        self.max_steering_angle = max_steering_angle
        self.verbose = verbose
        self.speed = 0
        self.steering_angle = 0

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0) 
            self.joystick.init()
            print(f"Joystick {self.joystick.get_name()} initialized.")
        else:
            print("No joystick found.")
            pygame.quit()
            exit()

    def get_input(self):
        """
        Get the input from the joystick and return the speed and steering angle
        of the vehicle.

        Returns:
            speed (float): speed of the vehicle
            steering_angle (float): steering angle of the vehicle
        """

        joy4 = self.joystick.get_axis(4)
        joy0 = self.joystick.get_axis(0)
        if abs(joy4) < 0.1:
            joy4 = 0
        if abs(joy0) < 0.1:
            joy0 = 0
        self.speed = - joy4 * self.max_speed
        self.steering_angle = - joy0 * self.max_steering_angle
        if self.verbose:
            print(f"Speed: {self.speed}, Steering Angle: {self.steering_angle}")
        return (self.speed, np.radians(self.steering_angle))
        