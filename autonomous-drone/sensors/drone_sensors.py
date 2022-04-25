from tf_mini import TFMiniPlus


class DroneSensors(object):
    """
    Class of sensors
    Used to deal with multiple sensors on the drone
    """
    def __init__(self, lidar_address, lidar_angle, critical_distance_lidar=50):
        self._lidar_number = len(lidar_address)  # Number of lidar sensors
        self._critical_distance_lidar = critical_distance_lidar
        self._front_lidar = None  # Lidar on the front of the drone
        self._side_lidar = []
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
                new_lidar = TFMiniPlus(address, angle, self._critical_distance_lidar)  # Initialize a lidar
                self.lidar_sensors.append(new_lidar)
                if angle == 0:
                    self._front_lidar = new_lidar
                else:
                    self._side_lidar.append(new_lidar)
        return lidar_sensors

    def get_front_lidar(self):
        return self._front_lidar

    def get_side_lidar(self):
        return self._side_lidar
