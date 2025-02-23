import numpy as np
from .vehicle import Vehicle
import math

class Simulator:
    """
    A lightweight simulator for the deepracer testbed.
    """

    def __init__(self,
                 num_vehicle = 1,
                 vehicle_states = None,
                 map_layout = 'map_1',
                 map_size = [6.0, 6.0],
                 time_step = 1/60,
                 ):
        """
        Args:
            num_controlled_vehicle (int): number of vehicle controlled through 
                commands received from outside.
            num_automated_vehicle (int): number of vehicle controlled by 
                autopilot.
            visulize (boolean): visulize the simulation using pygame
            
        """

        self.num_vehicle = num_vehicle
        self.map_layout = map_layout
        self.map_size = map_size
        self.time_step = time_step

        self.vehicle_size = [0.3/ map_size[0] * 800, 0.2/ map_size[0] * 800] 

        self.vehicle = []
        if vehicle_states is not None:
            for state in vehicle_states:
                self.vehicle.append(Vehicle(*state))
        for i in range(num_vehicle):
            self.vehicle.append(Vehicle())

    def step(self,
             vehicle_actions,):
        """
        Update the state of the simulation base on the input actions.

        Args:
            vehicle_actions (list): list of actions for each vehicle. Each 
                action is a tuple of (velocity, steering).
        
        Returns:
            vehicle_states (list): list of states for each vehicle. Each state
                is a tuple of (x, y, yaw).
        """

        assert len(vehicle_actions) == self.num_vehicle, \
            "Number of actions should match number of vehicles."
        
        vehicle_states = []

        for i in range(self.num_vehicle):
            pos = self.vehicle[i].step(*vehicle_actions[i], self.time_step)
            vehicle_states.append(pos)

        return vehicle_states


        
