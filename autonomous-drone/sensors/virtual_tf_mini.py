from virtual_sensors import VirtualSensor
import numpy as np

DEFAULT_CRITICAL_DISTANCE = 50


class VirtualTFMiniPlus(VirtualSensor):
    """Class for a specific range sensor : the TFMini Plus"""

    def __init__(self, critical_distance=DEFAULT_CRITICAL_DISTANCE):
        """Constructor : can take as input the critical distance of the sensor under which we detect an obstacle"""
        VirtualSensor.__init__(self, critical_distance)  # Calls the constructor of the parent class
        self._name = "Lidar"
        self._time_between_readings = 0.02  # ask a reading every 20 ms

    def read_distance(self, x_drone, y_drone, walls):
        """Read the distance return by the TFMiniPlus"""
        for wall in walls:
            x_0, y_O = wall.get_obstacle_origin()
            x_f, y_f = wall.get_obstacle_position()
            if x_0 < x_drone < x_f:
                if np.abs(wall.obstacle_equation(x_drone)-y_drone) < self._distance_detection:
                    self.set_distance(np.abs(wall.obstacle_equation(x_drone)-y_drone))
                    return True
        return False

    def lidar_reading(self):
        return self.time_since_last_reading() > self._time_between_readings
