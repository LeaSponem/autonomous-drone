from range_sensors import RangeSensor
import numpy as np

DEFAULT_CRITICAL_DISTANCE = 1
DEFAULT_DISTANCE_DETECTION = 5


class VirtualTFMiniPlus(RangeSensor):
    """Class for a specific range sensor : the TFMini Plus"""

    def __init__(self, critical_distance=DEFAULT_CRITICAL_DISTANCE, distance_detection=DEFAULT_DISTANCE_DETECTION):
        """Constructor : can take as input the critical distance of the sensor under which we detect an obstacle"""
        RangeSensor.__init__(self, critical_distance)  # Calls the constructor of the parent class
        self._name = "Lidar"
        self._time_between_readings = 0.02  # ask a reading every 20 ms
        self._distance_detection = distance_detection

    def read_distance(self, x_drone=0, y_drone=0, angle_drone=0, walls=None):
        """Read the distance return by the TFMiniPlus"""
        for wall in walls:
            x_i, y_i = wall.intersection(x_drone, y_drone, angle_drone)
            if np.sqrt((x_i - x_drone)**2 + (y_i - y_drone)**2) < self._distance_detection:
                self.set_distance(np.sqrt((x_i - x_drone)**2 + (y_i - y_drone)**2))
                return True
        return False

    def lidar_reading(self):
        return self.time_since_last_reading() > self._time_between_readings
