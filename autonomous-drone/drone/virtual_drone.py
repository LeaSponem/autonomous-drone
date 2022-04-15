import numpy as np
from inspection_drone import InspectionDrone
from simulation_position import SimulationPosition


class VirtualDrone(InspectionDrone):
    def __init__(self, connection_string, baudrate, two_way_switches, three_way_switches):
        InspectionDrone.__init__(self, connection_string, baudrate,
                                 two_way_switches, three_way_switches, critical_distance_lidar=1)
        self._drone_x = 0
        self._drone_y = 0
        self._yaw = self.vehicle.attitude.yaw
        self._initial_angle = (180/np.pi)*(np.pi/2 + self._yaw)
        self._location = self.vehicle.location.global_relative_frame
        self._local_frame = self._init_local_frame(self)


    def _init_local_frame(self):
        local_frame = SimulationPosition(self._location.lat, self._location.lon, (180 / np.pi) * (np.pi / 2 + self._yaw)
        return local_frame

    def _update_location(self):
        self._location = self.vehicle.location.global_relative_frame

    def _update_yaw(self):
        self._yaw = self.vehicle.attitude.yaw

    def _update_virtual_position(self):
        self._update_location()
        self._drone_x, self._drone_y = self._local_frame.get_position(self._location)

    def get_angle(self):
        """Return the angle between the drone direction and the X axis"""
        self._update_yaw()
        return self._initial_angle - self._yaw

    def get_virtual_position(self):
        return self._drone_x, self._drone_y
