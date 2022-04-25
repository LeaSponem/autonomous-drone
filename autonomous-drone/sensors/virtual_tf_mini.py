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
        Return True if an obstacle is within the sensor range, False otherwise
        Update the distance value to the min distance from an obstacle or to 0 otherwise
        """
        if walls is not None:
            distance = self._distance_detection + 1
            for wall in walls:
                # Check that the drone and the obstacle are not parallel
                if np.abs(wall.angle - angle_drone) > 0.5:
                    x_i, y_i = wall.intersection(x_drone, y_drone, angle_drone)
                    # Only keep the obstacles in front of the drone direction
                    if self._check_obstacle_orientation(x_drone, y_drone, angle_drone, x_i, y_i):
                        # Only keep the obstacle with the minimum distance from the drone
                        if np.sqrt((x_i - x_drone) ** 2 + (y_i - y_drone) ** 2) < distance:
                            distance = np.sqrt((x_i - x_drone) ** 2 + (y_i - y_drone) ** 2)
            # Check if the obstacle is within the sensor range
            if distance <= self._distance_detection:
                self.set_distance(distance)
                return True
            self.set_distance(0)
            return False

    def _get_max_range_coordinates(self, x_drone, y_drone, angle_drone):
        """
        Compute the coordinates of the max range point in the drone current direction
        """
        x_max_range = x_drone + self._distance_detection * np.cos(angle_drone)
        y_max_range = y_drone + self._distance_detection * np.sin(angle_drone)
        return x_max_range, y_max_range

    def _check_obstacle_orientation(self, x_drone, y_drone, angle_drone, x_intersection, y_intersection):
        """
        Check if the distance is measured in front of the drone
        Return True if the intersection point is in front of the drone, False otherwise
        """
        x_max_range, y_max_range = self._get_max_range_coordinates(x_drone, y_drone, angle_drone)
        if np.sqrt((x_max_range - x_intersection)**2 + (y_max_range - y_intersection)**2) < self._distance_detection:
            return True
        return False

    def lidar_reading(self):
        """
        Check if the time since the last reading is superior to the default time between two readings
        """
        return self.time_since_last_reading() > self._time_between_readings
