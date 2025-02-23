import numpy as np
from dr_testbed.utility import WayPoint

class DummyPlanner:
    def __init__(self, way_point, plan_horizon=0.3):
        assert isinstance(way_point, WayPoint), \
            "way_point is not a WayPoint object"
        self.way_point = way_point
        self.plan_horizon = plan_horizon

    def plan(self, vehicle_state):
        vehicle_pos = vehicle_state[0]
        vehicle_heading = (np.cos(vehicle_state[1]), np.sin(vehicle_state[1]))

        dist_to_waypoint = np.linalg.norm(np.array(vehicle_pos)\
                                           - np.array(self.way_point.pos))
        if dist_to_waypoint < self.plan_horizon:
            assert isinstance(self.way_point.next_waypoints[0], WayPoint), \
                "Next waypoint is not a WayPoint object"
            self.way_point = self.way_point.next_waypoints[0]
            dist_to_waypoint = np.linalg.norm(np.array(vehicle_pos)\
                                              - np.array(self.way_point.pos))

        P0 = np.array(vehicle_pos)
        P1 = np.array(self.way_point.pos)

        # Direction vectors
        D0 = np.array(vehicle_heading)
        D1 = -np.array(self.way_point.dir)

        # Control points (adjust the t parameter as needed)
        t = dist_to_waypoint/2
        C0 = P0 + t * D0
        C1 = P1 + t * D1

        # Generate the curve
        t_values = np.linspace(0, 1, int(100))
        curve_np = (1-t_values)**3 * P0[:,None] + \
                3*(1-t_values)**2 * t_values * C0[:,None] + \
                3*(1-t_values) * t_values**2 * C1[:,None] + \
                t_values**3 * P1[:,None]
        
        curve = []
        for i in range(len(curve_np[0])):
            dist = np.linalg.norm(np.array(curve_np[:,i]) - np.array(vehicle_pos))
            if dist > self.plan_horizon:
                break
            curve.append((curve_np[0][i], curve_np[1][i]))
        return curve

