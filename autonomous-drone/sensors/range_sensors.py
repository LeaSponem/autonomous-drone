import serial
import time

SER = serial.Serial("/dev/ttyACM0", 115200)  # change ACM number as found from ls /dev/tty/ACM*
SER.baudrate = 115200
DEFAULT_CRITICAL_DISTANCE = 50  # in cm


def read_sensors_line():
    """Function that reads the last line of data given by the sensors"""
    line = str(SER.readline())
    # We are going to avoid some reading mistakes caused by an incomplete line
    if len(line) not in [15, 16, 17]:  # The length of a line can be 20, 21 or 22, depending on the value of the range
        line = "No Data"  # If the length of the line read is not 20, 21 nor 22, there is an error
    return line


def debug_serial():
    print(read_sensors_line())


# -------- Parent range sensor class ---------

class RangeSensor:
    """ Class that defines a general range sensor. It cannot read data because this function is specific to each sensors.
    Thus this class cannot be used by itself but is very useful for the lidar and sonar child classes """

    def __init__(self, _critical_distance=DEFAULT_CRITICAL_DISTANCE):
        self.name = "Range sensor"
        self.range = 999
        self.critical_distance = _critical_distance
        self.log = []  # type: list[int]
        # log : stores range values for visualization
        self.time_log = []  # type: list[float]
        # time_log : stores the time the range was stored in the log (in ms)
        self.startTime = time.time()  # startTime : stores the time at which the object was created (in ms)

    def get_last_index_range(self, range_string, first_index_range):
        """ Function that takes as input a string to read and the index of the first byte of the data we want to get (in cm)
        and returns the index of the last byte containing the data in cm.
        This function is used with the data given in the following example format : b'Sonar_Range:57\\r\\n'
        where we want to recover the '57'
        """

        first_index_range += 1  # first index range becomes, from here, the last_index_range
        while True:
            if range_string[first_index_range:first_index_range + 1] == " ":
                return first_index_range
            first_index_range += 1

    def critical_distance_reached(self):
        """ Checks if the range is inferior to the critical distance of the sensor
        Returns True if that's the case, False otherwise"""

        if self.range < self.critical_distance:
            return True
        return False

    def get_distance(self):
        """Returns the last distance read by the sensor"""
        return self.range

    def set_distance(self, distance_value):
        """Modifies the range value stored by the object.
        In the meantimes, it stores this value and the time it was written in a list"""
        self.range = distance_value
        self.log.append(distance_value)
        self.time_log.append(time.time() - self.startTime)


# -------- Childs : special range sensors classes ---------


class Lidar(RangeSensor):
    """Class for a specific range sensor : the TFMini Plus"""

    def __init__(self, _critical_distance=DEFAULT_CRITICAL_DISTANCE):
        """Constructor : can take as input the critical distance of the sensor under which we detect an obstacle"""
        RangeSensor.__init__(self,
                             _critical_distance)  # Calls the constructor of the parent class, which define a general range sensor
        self.name = "Lidar"

    def read_distance(self, read_ser=read_sensors_line()):
        """Gets the range given by the sensor connected to the Arduino
        Returns False if no data is read and True otherwise.

        Takes as input the line containing the data we want to read from the Arduino.
        If no line is given as input, we read a new line.
        """
        if read_ser[0] == 'L':
            # The line with the data looks like that : "b'Lidar_Range:174\\r\\n'"
            # Thus, a line starting by S contains data from the Sonar
            # last_index_range = self.get_last_index_range(read_ser, 12)
            self.set_distance(int(read_ser[12::]))
            return True
        return False
