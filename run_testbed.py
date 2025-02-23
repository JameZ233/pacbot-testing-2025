import pygame
import math
import numpy as np
from dr_testbed.simulator import Simulator  # Your simulator module
from dr_testbed.render import Renderer        # Your renderer module
from dr_testbed.diff_simulator import DifferentialDriveSimulator        # Your vehicle module

# Define simulation parameters
time_step = 1/60
num_vehicle = 1
desired_wall_distance = 0.5  # target distance in meters
constant_velocity = 0.3      # m/s
wheel_base = 0.2

# Record the simulation start time
start_ticks = pygame.time.get_ticks()  # milliseconds

def simulate_infrared_sensor(vehicle_state, sensor_offset=(0, -0.15)):
    (x, y), yaw = vehicle_state
    # Rotate the sensor offset by the vehicle's yaw.
    offset_x = sensor_offset[0] * math.cos(yaw) - sensor_offset[1] * math.sin(yaw)
    offset_y = sensor_offset[0] * math.sin(yaw) + sensor_offset[1] * math.cos(yaw)
    sensor_pos = (x + offset_x, y + offset_y)
    
    wall_x = 1.0  
    distance = abs(wall_x - sensor_pos[0])
    return distance, sensor_pos

def wall_following_pid_controller(sensor_distance, desired_distance=0.5, dt=1/60,
                                  Kp=1.0, Ki=0.0, Kd=0.1,
                                  integral=0, last_error=0):
    """
    PID controller for wall following.
    
    Args:
        sensor_distance (float): current distance measured by the sensor.
        desired_distance (float): target distance to the wall.
        dt (float): time step.
        Kp, Ki, Kd (float): PID gains.
        integral (float): accumulated integral error.
        last_error (float): error at previous time step.
    
    Returns:
        steering (float): steering command in radians.
        integral (float): updated integral term.
        last_error (float): updated last error.
    """
    error = sensor_distance - desired_distance
    integral += error * dt
    derivative = (error - last_error) / dt if dt > 0 else 0
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error

    # Invert the output to correct the steering direction.
    steering = -output

    # Clamp the output to the vehicle's max steering angle.
    max_steering = np.radians(15)
    steering = np.clip(steering, -max_steering, max_steering)
    return steering, integral, last_error

def compute_wheel_commands(forward_velocity, desired_yaw_rate, wheel_base):
    """
    Map a desired forward velocity and yaw rate into motor commands for a square vehicle with four wheels:
    front, right, rear, and left.
    
    For forward movement, we only activate the front and rear wheels:
      - Front wheel speed = forward_velocity - (desired_yaw_rate * (wheel_base)/2)
      - Rear wheel speed  = forward_velocity + (desired_yaw_rate * (wheel_base)/2)
    The side wheels (right and left) are set to 0.
    
    Args:
        forward_velocity (float): desired forward speed (m/s).
        desired_yaw_rate (float): desired yaw rate (rad/s).
        wheel_base (float): distance between front and rear wheels.
        
    Returns:
        (v_front, v_right, v_rear, v_left): motor speeds for the four wheels.
    """
    v_left = forward_velocity - (desired_yaw_rate * wheel_base / 2.0)
    v_right  = forward_velocity + (desired_yaw_rate * wheel_base / 2.0)
    v_front = 0.0
    v_rear  = 0.0
    return v_left, v_right


# Create Simulator and Renderer instances
initial_state = ([0.4, -2.5], np.pi/2)
sim = DifferentialDriveSimulator(num_vehicle=num_vehicle, vehicle_states=[initial_state], time_step=time_step)
renderer = Renderer(map_layout='map_1', map_size=[6.0, 6.0])

# Variables for PID controller
pid_integral = 0
last_error = 0

# Record the simulation start time
start_ticks = pygame.time.get_ticks()  # milliseconds

# Main simulation loop
running = True
while running:
    dt = time_step  # time step of simulation
    current_ticks = pygame.time.get_ticks()
    elapsed_time = (current_ticks - start_ticks) / 1000.0  # convert to seconds
    
    # Check for quit events (using Pygame event loop)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Get current vehicle state ((x,y), yaw)
    state = ((sim.vehicle[0].x, sim.vehicle[0].y), sim.vehicle[0].yaw)
    
    # --- Pause for the first 1 seconds ---
    if elapsed_time < 1.0:
        # For 1 seconds, hold the vehicle in its initial state.
        wheel_commands = (0.0, 0.0)
    else:
        # Simulate sensor reading from the right side of the vehicle
        sensor_distance, sensor_pos = simulate_infrared_sensor(state, sensor_offset=(0, -0.15))
        
        # Compute steering command using the PID controller.
        steering_command, pid_integral, last_error = wall_following_pid_controller(
            sensor_distance,
            desired_distance=desired_wall_distance,
            dt=dt,
            Kp=0.8,
            Ki=0.05,
            Kd=0.2,
            integral=pid_integral,
            last_error=last_error
        )

         # Map the forward velocity and yaw rate to four-wheel commands.
        wheel_commands = compute_wheel_commands(constant_velocity, steering_command, wheel_base)
        
        # Optionally, print sensor data and controller output for debugging.
        print(f"Time: {elapsed_time:.2f} s, Sensor: {sensor_distance:.2f}, Steering: {steering_command:.2f}")
        
        # Construct vehicle action tuple: (velocity, steering)
        vehicle_action = (constant_velocity, steering_command)
    
    # Update the simulator with this action
    # vehicle_states = sim.step([vehicle_action])
    # Update the simulation using the computed four-wheel commands.
    vehicle_states = sim.step([wheel_commands])
    
    # Render the simulation.
    renderer.render(vehicle_states)
    
pygame.quit()
