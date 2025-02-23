from .diff_vehicle import DifferentialDriveVehicle

class DifferentialDriveSimulator:
    def __init__(self,
                 num_vehicle=1,
                 vehicle_states=None,
                 time_step=1/60):
        self.num_vehicle = num_vehicle
        self.time_step = time_step
        self.vehicle = []
        if vehicle_states is not None:
            for state in vehicle_states:
                # Each state is a tuple ((x,y), yaw)
                self.vehicle.append(DifferentialDriveVehicle(position=state[0],
                                                              yaw=state[1]))
        # If not provided, create default vehicles.
        for i in range(num_vehicle - len(self.vehicle)):
            self.vehicle.append(DifferentialDriveVehicle())

    def step(self, vehicle_actions):
        """
        Update the simulation state.
        
        Args:
            vehicle_actions (list): List of (left_motor_speed, right_motor_speed) actions.
        
        Returns:
            vehicle_states (list): Updated states of vehicles.
        """
        assert len(vehicle_actions) == self.num_vehicle, \
            "Number of actions must match number of vehicles."
        
        vehicle_states = []
        for i in range(self.num_vehicle):
            state = self.vehicle[i].step(vehicle_actions[i][0],
                                         vehicle_actions[i][1],
                                         self.time_step)
            vehicle_states.append(state)
        return vehicle_states
