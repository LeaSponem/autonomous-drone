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

    def read_distance(self, x_drone=0, y_drone=0, walls=None):
        """Read the distance return by the TFMiniPlus"""
        for wall in walls:
            x_0, y_0 = wall.get_obstacle_origin()
            x_f, y_f = wall.get_obstacle_position()
            if x_0 < x_drone < x_f:
                if np.abs(wall.obstacle_equation(x_drone)-y_drone) < self._distance_detection:
                    self.set_distance((np.abs(wall.obstacle_equation(x_drone)-y_drone)))
                    return True
        return False

    def lidar_reading(self):
        return self.time_since_last_reading() > self._time_between_readings
