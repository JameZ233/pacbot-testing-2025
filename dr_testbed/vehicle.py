import numpy as np

class Vehicle:
    """
    Vehicle class that base on a modified bicycle model that takes steering 
    angel and velocity as inputs. Metrics used in this model are meter for 
    length, meter/second for velocity and radian for yaw and steering angel.
    """
    def __init__(self, 
                 position = [0, 0],
                 yaw = 0,
                 wheel_base = 0.15,
                 max_steering_angel = np.radians(15),
                 max_velocity = 0.5,
                 deterministic = True
                 ):
        """
        Args:
            position (float, float): initial position of the vehicle
            yaw (float): initial yaw of the vehicle
            wheel_base (float): the distance between front and back axel of the 
                vehicle
            max_steering_angel (float): the maximum steering angel of the 
                vehicle
            deterministic (boolean): 
        """
        self.x = position[0]
        self.y = position[1]
        self.yaw = yaw
        self.wheel_base = wheel_base
        self.max_steering_angel = max_steering_angel
        self.max_velocity = max_velocity
        self.deterministic = deterministic

    def step(self,
             velocity,
             steering,
             dt):
        """
        Update the position and heading of the vehicle base on the input
        velocity and steering.
        """
        assert velocity >= -self.max_velocity and velocity <= self.max_velocity
        assert steering <= self.max_steering_angel \
            and steering >= -self.max_steering_angel
        
        if not self.deterministic:
            # Randomize speed, steering and time step
            pass

        yaw_rate = velocity * np.tan(steering) / self.wheel_base
        self.yaw += yaw_rate * dt
        self.x += velocity * np.cos(self.yaw) * dt
        self.y += velocity * np.sin(self.yaw) * dt
        
        output = ((self.x, self.y), self.yaw)

        if not self.deterministic:
            # Randomize the output
            pass
        
        return output
        