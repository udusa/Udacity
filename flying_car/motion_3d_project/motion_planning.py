import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 3.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 10

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        filename = 'colliders.csv'
        #['lat0', '37.792480', 'lon0', '-122.397450']
        line0 = open(filename).readlines()[0].replace(',','').split()
        lat0 = float(line0[1])
        lon0 = float(line0[3])
        print('lat0 {0}, lon0 {1}'.format(lat0,lon0))
        
        # TODO: set home position to (lat0, lon0, 0)
        self.set_home_position(lon0,lat0,0)

        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        local_pos = global_to_local(self.global_position,self.global_home)
        print('local_pos {0}'.format(local_pos))
        
        print('global home {0},\nposition {1},\nlocal position {2}'.format(self.global_home,
                                                                         self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        # grid, north_offset, east_offset, _ = create_grid_25(data, SAFETY_DISTANCE)
        voxel_size = 10
        grid, north_offset, east_offset  = create_voxmap(data, voxel_size,SAFETY_DISTANCE)
        print('Grid size : {0}'.format(grid.shape))
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        location_on_grid=[(local_pos[0] - north_offset),(local_pos[1] - east_offset)]
        print('grid location {0}'.format(location_on_grid))
        # TODO: convert start position to current position rather than map center
        grid_start = (int(location_on_grid[0]), 
            int(location_on_grid[1]))
        # Set goal as some arbitrary position on the grid
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        # tmp_maps_latlon = [37.793493, -122.397615] #NEAR POINT
        # tmp_maps_latlon = [37.795834, -122.393040] #OVER TREES
        tmp_maps_latlon = [37.795659, -122.396507] #OVER BUILDING
        lonlatalt_goal = [tmp_maps_latlon[1], tmp_maps_latlon[0] , 0]
        goal_grid_offset=global_to_local(lonlatalt_goal,self.global_home)
        print('goal_grid_offset : {0}'.format(goal_grid_offset))

        grid_goal = (int(-north_offset+goal_grid_offset[0]),
                    int(-east_offset+goal_grid_offset[1]))
        # grid_goal = find_closest_valid_cell(grid,grid_goal)+(TARGET_ALTITUDE,)
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        # alt_padding = 5
        # max_alt = TARGET_ALTITUDE + alt_padding
        # min_alt = TARGET_ALTITUDE - alt_padding
        start3d = tuple([int(x/voxel_size) for x in grid_start])+((TARGET_ALTITUDE/voxel_size),)
        goal3d = tuple([int(x/voxel_size) for x in grid_goal])+((TARGET_ALTITUDE/voxel_size),)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal,min_alt,max_alt)
        path, _ = a_star(grid, heuristic, start3d, goal3d)

        
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        path = prune_path(path)
        path.extend(spiral_path(path[len(path)-1]))


        # print('Path : ',path)
        # print('Spiral',spiral_path(path[len(path)-1]))

        waypoints = []
        for i,p in enumerate(path[1:len(path)-1]):
            p1,p2,p3 = (path[i-1],path[i],path[i+1])
            yaw_p3 = np.arctan2((p2[1]-p1[1]), (p2[0]-p1[0]))
            waypoints.append([int(p[0]*voxel_size + north_offset),
                     int(p[1]*voxel_size + east_offset), 
                     int(p[2]*voxel_size), yaw_p3])

        print('Waypoints',waypoints)

        # Convert path to waypoints
        # waypoints = [[p[0]*voxel_size + north_offset,
        #              p[1]*voxel_size + east_offset, 
        #              int(p[2]*voxel_size), 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        input('Waypoints ready, continue? (press any key)')

        # TODO: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()
        # Only required if they do threaded
        # while self.in_mission:
        #    pass
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
