from range_sensors import RangeSensor
import numpy as np

DEFAULT_CRITICAL_DISTANCE = 1
DEFAULT_DISTANCE_DETECTION = 5


class VirtualTFMiniPlus(RangeSensor):
    """
    Class for a specific range sensor : the TFMini Plus
    For simulator use only
    """
    def __init__(self, critical_distance=DEFAULT_CRITICAL_DISTANCE, distance_detection=DEFAULT_DISTANCE_DETECTION):
        """
        Constructor
        Inputs :
        - The critical distance of the sensor under which we detect an obstacle
        - The distance under which the lidar can read a distance
        """
        # Calls the constructor of the parent class
        RangeSensor.__init__(self, critical_distance)
        self._name = "Lidar"
        # Ask a reading every 20 ms
        self._time_between_readings = 0.02
        self._distance_detection = distance_detection

    def read_distance(self, x_drone=0, y_drone=0, angle_drone=0, walls=None):
        """
        Read artificially the distance between the drone and the obstacles
        Inputs :
        - x_drone, y_drone: virtual drone coordinates
        - angle_drone: angle between the lidar direction and the X axis
        - walls: list of wall obstacles
        """
        for wall in walls:
            x_i, y_i = wall.intersection(x_drone, y_drone, angle_drone)
            if np.sqrt((x_i - x_drone)**2 + (y_i - y_drone)**2) < self._distance_detection:
                self.set_distance(np.sqrt((x_i - x_drone)**2 + (y_i - y_drone)**2))
                return True
        return False

    def lidar_reading(self):
        """
        Check if the time since the last reading is superior to the default time between two readings
        """
        return self.time_since_last_reading() > self._time_between_readings
