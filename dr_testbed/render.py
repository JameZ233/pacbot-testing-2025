import pygame
import math
import os

class Renderer:
    """
    A class to render the simulation.
    """

    def __init__(self,
                 map_layout,
                 vehicle_size = [0.3, 0.2],
                 map_size = [6.0, 6.0],
                 frame_rate = 60,
                 ):
        """
        Args:
            num_controlled_vehicle (int): number of vehicle controlled through 
                commands received from outside.
            num_automated_vehicle (int): number of vehicle controlled by 
                autopilot.
            visulize (boolean): visulize the simulation using pygame
            
        """

        self.map_layout = map_layout
        self.map_size = map_size
        self.frame_rate = frame_rate

        self.vehicle_size = [0.3/ map_size[0] * 800, 0.2/ map_size[0] * 800] 

        self.vehicle = []

        pygame.init()
        self.screen = pygame.display.set_mode((800, 800))
        self.clock = pygame.time.Clock()
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        file_path = os.path.join(base_dir, 'maps', map_layout + '.jpeg')
        # self.track_image = \
        #     pygame.image.load(file_path)
        # self.track_image = \
        #     pygame.transform.scale(self.track_image, (800, 800))
        
    def render(self, vehicle_states, waypoints = None, traj = None):
        """
        Render the simulation.
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        # self.screen.blit(self.track_image, (0, 0))
        # Use the programmatically drawn map:
        self.draw_map()
        for state in vehicle_states:
            self.draw_vehicle(state)
        if waypoints is not None:
            self.draw_waypoints(waypoints)
        if traj is not None:
            self.draw_traj(traj)
        pygame.display.flip()
        self.clock.tick(self.frame_rate)

    def draw_vehicle(self, state):
        """
        Draw a vehicle on the screen.
        """
        center = (int(state[0][0]/self.map_size[0]*800)+400, 
                  int(-state[0][1]/self.map_size[0]*800+ 400))
        # center = (0, 0)

        points = [(self.vehicle_size[0]/2, self.vehicle_size[1]/2),
                  (-self.vehicle_size[0]/2, self.vehicle_size[1]/2),
                  (-self.vehicle_size[0]/2, -self.vehicle_size[1]/2),
                  (self.vehicle_size[0]/2, -self.vehicle_size[1]/2)]

        rotated_points = []
        cos_angle = math.cos(-state[1])
        sin_angle = math.sin(-state[1])

        for point in points:
            x = point[0] * cos_angle - point[1] * sin_angle
            y = point[0] * sin_angle + point[1] * cos_angle
            rotated_points.append((x + center[0], y + center[1]))

            # Draw the rectangle
        pygame.draw.polygon(self.screen, (0, 255, 0), rotated_points)

    def draw_waypoints(self, waypoints):
        """
        Draw waypoints on the screen.
        """
        for waypoint in waypoints:
            center = (int(waypoint.pos[0]/self.map_size[0]*800)+400, 
                      int(-waypoint.pos[1]/self.map_size[0]*800+ 400))
            pygame.draw.circle(self.screen, (255, 0, 0), center, 5)

    def draw_traj(self, traj):
        """
        Draw trajectory on the screen.
        """
        traj_ = []
        for point in traj:
            center = (int(point[0]/self.map_size[0]*800)+400, 
                      int(-point[1]/self.map_size[0]*800+ 400))
            traj_.append(center)
        pygame.draw.lines(self.screen, (0, 0, 255), False, traj_, 5)

    def draw_map(self):
        # Fill the background with a light gray color.
        self.screen.fill((200, 200, 200))
        
        # Convert simulation coordinate x = 1.0 to screen coordinate.
        # In the renderer, simulation x is scaled by (800 / map_size[0]) and then shifted by 400.
        wall_x = int((1.0 / self.map_size[0]) * 800 + 400)
        
        # Draw a vertical wall (black line) spanning the height of the screen.
        pygame.draw.line(self.screen, (0, 0, 0), (wall_x, 0), (wall_x, 800), 5)
