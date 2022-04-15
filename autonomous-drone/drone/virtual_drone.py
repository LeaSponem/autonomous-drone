import numpy as np
import time
import sys
from inspection_drone import InspectionDrone
from simulation_position import SimulationPosition
sys.path.insert(0, '../sensors')
from virtual_tf_mini import VirtualTFMiniPlus


class VirtualDrone(InspectionDrone):
    def __init__(self, connection_string, baudrate, two_way_switches, three_way_switches, critical_distance_lidar=1):
        InspectionDrone.__init__(self, connection_string, baudrate,
                                 two_way_switches, three_way_switches, critical_distance_lidar=1)
        self._drone_x = 0
        self._drone_y = 0
        self._yaw = self.vehicle.attitude.yaw
        self._initial_angle = (180/np.pi)*(np.pi/2 + self._yaw)
        self._location = self.vehicle.location.global_relative_frame
        self._local_frame = self._init_local_frame()
        self._lidar = VirtualTFMiniPlus(critical_distance_lidar)

    def _init_local_frame(self):
        local_frame = SimulationPosition(self._location.lat, self._location.lon, (180 / np.pi) * (np.pi / 2 + self._yaw))
        return local_frame

    def _update_location(self):
        self._location = self.vehicle.location.global_relative_frame

    def _update_yaw(self):
        self._yaw = self.vehicle.attitude.yaw

    def _update_virtual_position(self):
        self._update_location()
        self._drone_x, self._drone_y = 0.001*self._local_frame.get_position(self._location)

    def get_angle(self):
        """Return the angle between the drone direction and the X axis"""
        self._update_yaw()
        return self._initial_angle - self._yaw

    def get_virtual_position(self):
        self._update_virtual_position()
        return self._drone_x, self._drone_y

    def update_detection(self, use_lidar=True, debug=False, walls=None):
        self._update_virtual_position()
        if self._lidar.read_distance(self._drone_x, self._drone_y, self.get_angle(), walls) and debug:
            print("Lidar range:" + str(self._lidar.get_distance()))
        if use_lidar and self._lidar.critical_distance_reached():
            if self.obstacle_detected():
                self._time_last_obstacle_detected = time.time()
            self._obstacle_detected = True
        else:
            self._obstacle_detected = False
