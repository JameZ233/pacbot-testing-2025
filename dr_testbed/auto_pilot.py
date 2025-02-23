import numpy as np
class AutoPilot:
    """
    AutoPilot that navigate a vehicle to follow a predtermined set of waypoints 
    or randomly cruse around the map.

    To be complete
    """

    def __init__(self,):
        pass


class DummyPilot:
    """
    A dummy pilot that run a fixed traj.
    """

    def __init__(self,):
        self.actions = np.load('actions.npy')
        self.count = 0

    def step(self, vehicle_states, traj):
        """
        Update the state of the simulation base on the input actions.

        Args:
            vehicle_states (list): list of states for each vehicle. Each state
                is a tuple of (x, y, yaw).
        
        Returns:
            vehicle_actions (list): list of actions for each vehicle. Each 
                action is a tuple of (velocity, steering).
        """
        if self.count >= len(self.actions):
            self.count = 0
        action = self.actions[self.count]
        self.count += 1
        return action