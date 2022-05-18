from tf_mini import TFMiniPlus


class DroneLidarSensors(object):
    """
    Class of lidar sensors
    Used to deal with multiple sensors on the drone
    """
    def __init__(self, lidar_address, lidar_angle, critical_distance_lidar=1):
        self._lidar_number = len(lidar_angle)  # Number of lidar sensors
        self._critical_distance_lidar = critical_distance_lidar
        # Initialize a list with all the lidar sensors
        self.lidar_sensors = self._init_lidar_sensors(lidar_address, lidar_angle)

    def _init_lidar_sensors(self, lidar_address, lidar_angle):
        """
        Initialize a list of lidar sensors with their address and angle
        """
        lidar_sensors = []  # List of lidar sensors
        if lidar_address is not None:
            index = dict(zip(lidar_address, lidar_angle))  # Dictionary of address and angle
            for address, angle in index.items():
                # Initialize a lidar
                lidar_sensors.append(TFMiniPlus(address, angle, self._critical_distance_lidar))
        return lidar_sensors


class ThreeLidarSensorsDetection(object):
    def __init__(self, lidar_address=None, lidar_angle=[0], critical_distance_lidar=100):
        self._lidar_sensors = DroneLidarSensors(lidar_address, lidar_angle, critical_distance_lidar).lidar_sensors
        self._right_lidar = None
        self._left_lidar = None
        self._front_lidar = None
        self._sort_sensors()
        self._obstacle_detected_right = False
        self._obstacle_detected_left = False

    def _sort_sensors(self):
        for lidar in self._lidar_sensors:
            if lidar.angle == 0:
                lidar.name = "Front lidar"
                self._front_lidar = lidar
            elif lidar.angle == -90:
                lidar.name = "Left lidar"
                self._left_lidar = lidar
            elif lidar.angle == 90:
                lidar.name = "Right lidar"
                self._right_lidar = lidar

    def get_front_lidar(self):
        return self._front_lidar

    def get_right_lidar(self):
        return self._right_lidar

    def get_left_lidar(self):
        return self._left_lidar

    def read_distance(self):
        return self._front_lidar.read_distance()

    def read_right_distance(self):
        return self._right_lidar.read_distance()

    def read_left_distance(self):
        return self._left_lidar.read_distance()

    def get_distance(self):
        return self._front_lidar.get_distance()

    def critical_distance_reached(self):
        return self._front_lidar.critical_distance_reached()

    def obstacle_detected_right(self):
        return self._obstacle_detected_right

    def obstacle_detected_left(self):
        return self._obstacle_detected_left

    def lidar_reading(self):
        return self._front_lidar.lidar_reading()
