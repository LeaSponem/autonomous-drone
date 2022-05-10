from virtual_tf_mini import VirtualTFMiniPlus
from drone_sensors import DroneLidarSensors, ThreeLidarSensorsDetection


class VirtualDroneLidarSensors(object):
    """
    Class of lidar sensors
    Used to deal with multiple sensors on the drone
    """
    def __init__(self, lidar_angle, critical_distance_lidar=50):
        self._lidar_number = len(lidar_angle)  # Number of lidar sensors
        self._critical_distance_lidar = critical_distance_lidar
        # Initialize a list with all the lidar sensors
        self.lidar_sensors = self._init_lidar_sensors(lidar_angle)

    def _init_lidar_sensors(self, lidar_angle):
        """
        Initialize a list of lidar sensors with their address and angle
        """
        lidar_sensors = []  # List of lidar sensors
        if lidar_angle is not None:
            for angle in lidar_angle:
                # Initialize a lidar
                lidar_sensors.append(VirtualTFMiniPlus(angle, self._critical_distance_lidar))
        return lidar_sensors


class VirtualThreeLidarSensorsDetection(ThreeLidarSensorsDetection):
    def __init__(self, lidar_angle, critical_distance_lidar=50):
        ThreeLidarSensorsDetection.__init__(self, lidar_angle=lidar_angle,
                                            critical_distance_lidar=critical_distance_lidar)
        self._lidar_sensors = VirtualDroneLidarSensors(lidar_angle, critical_distance_lidar).lidar_sensors
        self._sort_sensors()

    def update_right_detection(self, debug=False):
        if self.read_right_distance() and debug:
            print("Right lidar range:" + str(self._right_lidar.get_distance()))
        if self._right_lidar.critical_distance_reached():
            self._obstacle_detected_right = True
        else:
            self._obstacle_detected_right = False

    def update_left_detection(self, debug=False):
        if self.read_left_distance() and debug:
            print("Left lidar range:" + str(self._left_lidar.get_distance()))
        if self._left_lidar.critical_distance_reached():
            self._obstacle_detected_left = True
        else:
            self._obstacle_detected_left = False


sensors = VirtualThreeLidarSensorsDetection(lidar_angle=[0, 90, -90])
