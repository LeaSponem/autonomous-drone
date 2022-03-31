import time


DEFAULT_CRITICAL_DISTANCE = 50  # in cm

# -------- Parent range sensor class ---------


class RangeSensor:
    """ Class that defines a general range sensor. It cannot read data because this function is specific to each sensors.
    Thus this class cannot be used by itself but is very useful for the lidar and sonar child classes """

    def __init__(self, _critical_distance=DEFAULT_CRITICAL_DISTANCE):
        self.name = "Range sensor"
        self.range = 999
        self.critical_distance = _critical_distance
        self.log = []
        # log : stores range values for visualization
        self.time_log = []
        # time_log : stores the time the range was stored in the log (in ms)
        self.start_time = time.time()  # startTime : stores the time at which the object was created (in ms)

    def critical_distance_reached(self):
        """ Checks if the range is inferior to the critical distance of the sensor
        Returns True if that's the case, False otherwise"""

        if 0 < self.range < self.critical_distance:
            return True
        return False

    def get_distance(self):
        """Returns the last distance read by the sensor"""
        return self.range

    def set_distance(self, distance_value):
        """Modifies the range value stored by the object.
        In the meantime, it stores this value and the time it was written in a list"""
        self.range = distance_value
        self.log.append(distance_value)
        self.time_log.append(time.time() - self.start_time)

    def time_since_last_reading(self):
        return time.time()-self.time_log[-1]
