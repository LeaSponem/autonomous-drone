from tf_mini import TFMiniPlus


class DroneLidarSensors(object):
    """
    Class of lidar sensors
    Used to deal with multiple sensors on the drone
    """
    def __init__(self, lidar_address, lidar_angle, critical_distance_lidar=50):
        self._lidar_number = len(lidar_address)  # Number of lidar sensors
        self._critical_distance_lidar = critical_distance_lidar
        # Initialize a list with all the lidar sensors
        self.lidar_sensors = self._init_lidar_sensors(lidar_address, lidar_angle)

    def _init_lidar_sensors(self, lidar_address, lidar_angle):
        """
        Initialize a list of lidar sensors with their address and angle
        Separate the front lidar from the others
        """
        lidar_sensors = []  # List of lidar sensors
        if lidar_address is not None:
            index = dict(zip(lidar_address, lidar_angle))  # Dictionary of address and angle
            for address, angle in index.items():
                # Initialize a lidar
                self.lidar_sensors.append(TFMiniPlus(address, angle, self._critical_distance_lidar))
            self._sort_sensors()
        return lidar_sensors

    def get_front_lidar(self):
        return self.lidar_sensors[1]

    def get_side_lidar(self):
        return [self.lidar_sensors[0], self.lidar_sensors[2]]

    def _sort_sensors(self):
        lidar_sensors = [None]*3
        for lidar in self.lidar_sensors:
            if lidar.angle == 0:
                lidar.name = "Front"
                lidar_sensors[1] = lidar
            elif lidar.angle == 90:
                lidar.name = "Left"
                lidar_sensors[0] = lidar
            elif lidar.angle == -90:
                lidar.name = "Right"
                lidar_sensors[2] = lidar
        self.lidar_sensors = lidar_sensors

    def read_right_distance(self):
        self.lidar_sensors[2].read_distance()

    def read_left_distance(self):
        self.lidar_sensors[0].read_distance()
